import machine
from machine import Pin, I2C, Timer
import gc
import time
import network
import ujson
import utime
import asyncio
import math
from microdot import Microdot, send_file
from microdot.websocket import with_websocket
from hx711 import HX711

# -------------------------- 全局配置 --------------------------
ap_ssid = 'PicoW-ForceSensor'
ap_password = '12345678'
ap_ip = '192.168.4.1'

# 重力加速度常数 (m/s²)
GRAVITY = 9.80665

# 采样率配置（使用固定值）
SAMPLE_RATE_HZ = 20  # 20Hz稳定采样
SAMPLE_INTERVAL_MS = 50  # 50ms
SAMPLE_RATE_FIXED = 20.0

# 小缓存配置
BUFFER_SIZE = 20

# HX711校准配置
REFERENCE_UNIT_1 = 2400  # 传感器1初始参考单位
REFERENCE_UNIT_2 = 2400  # 传感器2初始参考单位

# -------------------------- 全局变量初始化 --------------------------
led = machine.Pin("LED", machine.Pin.OUT)
app = Microdot()

# 启用垃圾回收
gc.enable()
gc.collect()

print(f"初始内存: {gc.mem_free()} bytes free")

# 硬件定时器
sampling_timer = None
sampling_event = asyncio.Event()

# 小缓冲区
data_buffer = []
buffer_lock = asyncio.Lock()

# 性能监控
sample_count = 0
last_sample_time = 0
sample_rate = 20.0
actual_sample_rate = 0.0

# 传感器偏移量
force1_offset = 0
force2_offset = 0
reference_unit_1 = REFERENCE_UNIT_1
reference_unit_2 = REFERENCE_UNIT_2

# 时间戳记录
last_cycle_start = 0

# -------------------------- 传感器初始化 --------------------------
print("初始化双HX711拉力传感器...")

# 传感器1初始化
print("初始化传感器1 (主传感器)...")
dout_pin_1 = 17
sck_pin_1 = 16
hx1 = HX711(dout_pin_1, sck_pin_1)
hx1.set_reading_format("MSB", "MSB")
hx1.set_reference_unit(reference_unit_1)
time.sleep(0.5)
hx1.reset()

# 传感器2初始化
print("初始化传感器2 (相互作用力传感器)...")
dout_pin_2 = 19
sck_pin_2 = 18
hx2 = HX711(dout_pin_2, sck_pin_2)
hx2.set_reading_format("MSB", "MSB")
hx2.set_reference_unit(reference_unit_2)
time.sleep(0.5)
hx2.reset()

# 精确置零 - 双传感器
print("双拉力传感器精确置零...")
try:
    # 传感器1置零
    print("传感器1置零...")
    for i in range(5):
        hx1.read_long()
        time.sleep(0.1)
    
    force1_offset = hx1.tare_A(20)
    print(f"✅ 传感器1置零完成，偏移量: {force1_offset}")
    
    # 传感器2置零
    print("传感器2置零...")
    for i in range(5):
        hx2.read_long()
        time.sleep(0.1)
    
    force2_offset = hx2.tare_A(20)
    print(f"✅ 传感器2置零完成，偏移量: {force2_offset}")
    
    # 验证置零效果
    test_samples_1 = []
    test_samples_2 = []
    for i in range(10):
        raw_val_1 = hx1.read_long()
        raw_val_2 = hx2.read_long()
        test_samples_1.append(raw_val_1 - force1_offset)
        test_samples_2.append(raw_val_2 - force2_offset)
        time.sleep(0.05)
    
    avg_offset_1 = sum(test_samples_1) / len(test_samples_1)
    avg_offset_2 = sum(test_samples_2) / len(test_samples_2)
    print(f"传感器1置零验证 - 平均剩余偏移: {avg_offset_1} 原始值")
    print(f"传感器2置零验证 - 平均剩余偏移: {avg_offset_2} 原始值")
    
    # 计算相互作用力对称性误差
    symmetry_error = abs(avg_offset_1 + avg_offset_2) / 2
    print(f"初始对称性误差: {symmetry_error} 原始值")
    
except Exception as e:
    print(f"❌ 置零失败: {e}")
    force1_offset = 0
    force2_offset = 0

led.on()

print(f"初始化完成后内存: {gc.mem_free()} bytes free")

# -------------------------- 硬件定时器 --------------------------
def init_hardware_timer():
    """初始化硬件定时器用于同步采样"""
    global sampling_timer
    
    try:
        sampling_timer = Timer()
        
        def timer_callback(timer):
            """定时器中断回调 - 设置采样事件"""
            sampling_event.set()
        
        sampling_timer.init(mode=Timer.PERIODIC, period=50, callback=timer_callback)
        print(f"✅ 硬件定时器初始化: {SAMPLE_RATE_FIXED}Hz")
        return True
    except Exception as e:
        print(f"❌ 硬件定时器初始化失败: {e}")
        return False

# 采样任务
async def synchronized_sampling_task():
    """硬件定时器触发的同步采样任务 - 双传感器"""
    global sample_count, sample_rate, last_sample_time, data_buffer, actual_sample_rate
    global force1_offset, force2_offset, reference_unit_1, reference_unit_2
    global last_cycle_start
    
    print("🚀 启动硬件定时器同步采样 - 双传感器")
    
    # 滤波器状态
    force1_filter = 0.0
    force2_filter = 0.0
    
    # 采样率监控变量
    rate_samples = []
    MAX_RATE_SAMPLES = 10
    
    # 时间差统计
    time_diff_history = []
    MAX_TIME_DIFF_HISTORY = 50
    
    # 相互作用力差值统计
    force_diff_history = []
    MAX_FORCE_DIFF_HISTORY = 50
    avg_force_diff = 0.0
    
    try:
        while True:
            try:
                # 等待硬件定时器事件
                await sampling_event.wait()
                sampling_event.clear()
                
                # 记录采样周期开始时间
                cycle_start = utime.ticks_us()
                last_cycle_start = cycle_start
                
                # 读取传感器1
                force1_raw = 0.0
                force1_read_start = utime.ticks_us()
                try:
                    raw_force1 = hx1.read_long()
                    if raw_force1 is not None:
                        calibrated_value1 = raw_force1 - force1_offset
                        force1_raw = calibrated_value1 / reference_unit_1 * GRAVITY / 1000
                        
                        # 简单滤波
                        alpha = 0.3
                        force1_filter = alpha * force1_raw + (1 - alpha) * force1_filter
                        force1_raw = force1_filter
                        
                        # 零值死区
                        if abs(force1_raw) < 0.005:
                            force1_raw = 0.0
                except Exception as e:
                    print(f"传感器1读取错误: {e}")
                    pass
                
                force1_read_end = utime.ticks_us()
                
                # 读取传感器2（作用力方向相反，取负值）
                force2_raw = 0.0
                force2_read_start = utime.ticks_us()
                try:
                    raw_force2 = hx2.read_long()
                    if raw_force2 is not None:
                        calibrated_value2 = raw_force2 - force2_offset
                        # 相互作用力方向相反，所以取负值
                        force2_raw = -(calibrated_value2 / reference_unit_2 * GRAVITY / 1000)
                        
                        # 简单滤波
                        alpha = 0.3
                        force2_filter = alpha * force2_raw + (1 - alpha) * force2_filter
                        force2_raw = force2_filter
                        
                        # 零值死区
                        if abs(force2_raw) < 0.005:
                            force2_raw = 0.0
                except Exception as e:
                    print(f"传感器2读取错误: {e}")
                    pass
                
                force2_read_end = utime.ticks_us()
                
                # 计算相互作用力差值（理论上应为0，牛顿第三定律）
                force_diff = abs(force1_raw - abs(force2_raw))
                force_diff_history.append(force_diff)
                if len(force_diff_history) > MAX_FORCE_DIFF_HISTORY:
                    force_diff_history.pop(0)
                
                avg_force_diff = sum(force_diff_history) / len(force_diff_history) if force_diff_history else 0.0
                
                # 计算读取时间差
                read_time_diff = utime.ticks_diff(max(force1_read_end, force2_read_end), 
                                                 min(force1_read_start, force2_read_start))
                time_diff_history.append(read_time_diff)
                if len(time_diff_history) > MAX_TIME_DIFF_HISTORY:
                    time_diff_history.pop(0)
                
                # 计算平均时间差
                avg_time_diff = sum(time_diff_history) / len(time_diff_history) if time_diff_history else 0
                
                # 计算实际采样率
                current_time = utime.ticks_us()
                if last_sample_time > 0:
                    interval_us = utime.ticks_diff(current_time, last_sample_time)
                    if interval_us > 0:
                        instant_rate = 1000000 / interval_us
                        rate_samples.append(instant_rate)
                        if len(rate_samples) > MAX_RATE_SAMPLES:
                            rate_samples.pop(0)
                        
                        if len(rate_samples) >= MAX_RATE_SAMPLES // 2:
                            actual_sample_rate = sum(rate_samples) / len(rate_samples)
                
                last_sample_time = current_time
                sample_count += 1
                
                # 添加到缓冲区，包含双传感器数据
                async with buffer_lock:
                    data_buffer.append({
                        "f1": round(force1_raw, 4),      # 传感器1力值 (N) - 正值
                        "f2": round(force2_raw, 4),      # 传感器2力值 (N) - 负值（相互作用力）
                        "n": sample_count,               # 采样序号
                        "r": SAMPLE_RATE_FIXED,          # 固定采样率
                        "t": cycle_start,                # 采样周期开始时间
                        "dt": avg_time_diff,             # 平均读取时间差
                        "fd": round(avg_force_diff, 4),  # 相互作用力平均差值
                        "fs": round(abs(force1_raw) - abs(force2_raw), 4)  # 力差值
                    })
                    
                    # 限制缓冲区大小
                    if len(data_buffer) > BUFFER_SIZE:
                        data_buffer.pop(0)
                
                # LED闪烁指示
                if sample_count % 10 == 0:
                    led.off()
                    await asyncio.sleep(0)
                    led.on()
                
                # 定期输出统计信息
                if sample_count % 100 == 0:
                    print(f"采样: {sample_count}, 固定速率: {SAMPLE_RATE_FIXED}Hz")
                    print(f"传感器1: {force1_raw:.4f}N, 传感器2: {force2_raw:.4f}N")
                    print(f"相互作用力差值: {avg_force_diff:.4f}N, 对称误差: {force_diff:.4f}N")
                    print(f"读取时间差: {avg_time_diff:.1f}μs, 内存: {gc.mem_free()}B")
                    gc.collect()
                
                # 控制CPU使用率
                await asyncio.sleep_ms(0)
                
            except Exception as e:
                print(f"采样异常: {e}")
                await asyncio.sleep_ms(20)
    except asyncio.CancelledError:
        print("采样任务被取消")
    finally:
        print("采样任务已停止")

# 数据发送任务
async def data_sending_task(ws):
    """数据发送任务 - 双传感器数据"""
    global sample_rate
    
    print(f"🔌 启动数据发送任务 - 双传感器")
    
    # 时间戳基准
    base_timestamp = utime.ticks_us()
    
    try:
        while True:
            # 从缓冲区获取数据
            async with buffer_lock:
                if data_buffer:
                    # 获取最新的数据
                    data = data_buffer[-1]
                    
                    # 计算相对时间戳
                    relative_time_us = utime.ticks_diff(data["t"], base_timestamp)
                    relative_time_ms = relative_time_us / 1000.0
                    
                    # 创建数据包，包含双传感器信息
                    packet = {
                        "t": f"{data['n']}",
                        "f1": data['f1'],      # 传感器1力值
                        "f2": data['f2'],      # 传感器2力值（负值）
                        "r": SAMPLE_RATE_FIXED,
                        "n": data['n'],
                        "ts": round(relative_time_ms, 2),
                        "dt": round(data['dt'], 1),
                        "fd": data['fd'],      # 相互作用力差值
                        "fs": data['fs'],      # 力对称性差值
                        "sync": 1
                    }
                    
                    # 清空缓冲区（只保留最新数据）
                    data_buffer.clear()
                    data_buffer.append(data)
                else:
                    packet = None
            
            # 发送数据
            if packet:
                try:
                    await ws.send(ujson.dumps(packet))
                except Exception as e:
                    print(f"发送失败: {e}")
                    break
            
            # 控制发送频率
            await asyncio.sleep_ms(50)
            
    except asyncio.CancelledError:
        print("数据发送任务被取消")
    except Exception as e:
        print(f"数据发送异常: {e}")

# -------------------------- Web路由配置 --------------------------
@app.route('/')
async def index(request):
    """加载前端主页"""
    return send_file("index.html")

@app.route('/static/<path:path>')
def static_file(request, path):
    """提供静态文件"""
    if '..' in path:
        return 'Not found', 404
    return send_file(f'static/{path}', max_age=86400)

@app.route('/sensor')
@with_websocket
async def sensor_handler(request, ws):
    """WebSocket核心 - 双传感器处理"""
    global sampling_timer, sample_rate, sample_count, force1_offset, force2_offset
    global reference_unit_1, reference_unit_2
    
    print(f"🔌 新设备连接")
    
    sampling_task = None
    send_task = None
    
    try:
        while True:
            cmd = await ws.receive()
            print(f"📩 收到指令：{cmd}")
            
            if cmd == "Flag2":
                print("📌 启动硬件定时器同步测量 - 双传感器")
                
                # 重置采样率和计数
                sample_rate = SAMPLE_RATE_FIXED
                sample_count = 0
                
                # 启动硬件定时器
                if sampling_timer is None:
                    init_hardware_timer()
                
                # 启动采样任务
                if sampling_task is None or sampling_task.done():
                    sampling_task = asyncio.create_task(synchronized_sampling_task())
                    print("✅ 双传感器采样任务已启动")
                
                # 启动发送任务
                if send_task is None or send_task.done():
                    send_task = asyncio.create_task(data_sending_task(ws))
                    print("✅ 发送任务已启动")
                
                await ws.send(ujson.dumps({
                    "t": "0",
                    "f1": 0.0,
                    "f2": 0.0,
                    "r": SAMPLE_RATE_FIXED,
                    "n": 0,
                    "msg": f"开始双传感器测量，采样率: {SAMPLE_RATE_FIXED}Hz"
                }))
            
            elif cmd == "Flag1":
                print("📌 执行双传感器精确手动置零")
                try:
                    # 传感器1置零
                    force1_offset = hx1.tare_A(20)
                    time.sleep(0.5)
                    
                    # 传感器2置零
                    force2_offset = hx2.tare_A(20)
                    time.sleep(0.5)
                    
                    # 验证置零效果
                    test_samples_1 = []
                    test_samples_2 = []
                    for i in range(10):
                        raw_val_1 = hx1.read_long()
                        raw_val_2 = hx2.read_long()
                        test_val_1 = (raw_val_1 - force1_offset) / reference_unit_1 * GRAVITY / 1000
                        test_val_2 = (raw_val_2 - force2_offset) / reference_unit_2 * GRAVITY / 1000
                        test_samples_1.append(test_val_1)
                        test_samples_2.append(-test_val_2)  # 注意：传感器2输出取负
                        await asyncio.sleep_ms(50)
                    
                    avg_force_1 = sum(test_samples_1) / len(test_samples_1)
                    avg_force_2 = sum(test_samples_2) / len(test_samples_2)
                    
                    print(f"传感器1置零完成，新偏移量: {force1_offset}")
                    print(f"传感器2置零完成，新偏移量: {force2_offset}")
                    print(f"传感器1剩余力: {avg_force_1:.4f}N")
                    print(f"传感器2剩余力: {avg_force_2:.4f}N")
                    
                    symmetry_error = abs(avg_force_1 + avg_force_2) / 2
                    print(f"对称性误差: {symmetry_error:.4f}N")
                    
                    await ws.send(ujson.dumps({
                        "t": "0",
                        "f1": 0.0,
                        "f2": 0.0,
                        "r": SAMPLE_RATE_FIXED,
                        "n": sample_count,
                        "msg": f"双传感器置零完成，对称误差: {symmetry_error:.4f}N"
                    }))
                except Exception as e:
                    print(f"置零失败: {e}")
                    await ws.send(ujson.dumps({
                        "t": "0",
                        "f1": 0.0,
                        "f2": 0.0,
                        "r": SAMPLE_RATE_FIXED,
                        "n": sample_count,
                        "msg": f"置零失败: {e}"
                    }))
            
            elif cmd == "Flag3":
                print("📌 停止连续测量")
                
                if sampling_task and not sampling_task.done():
                    sampling_task.cancel()
                    try:
                        await sampling_task
                    except:
                        pass
                    print("✅ 采样任务已停止")
                
                if send_task and not send_task.done():
                    send_task.cancel()
                    try:
                        await send_task
                    except:
                        pass
                    print("✅ 发送任务已停止")
                
                if sampling_timer:
                    sampling_timer.deinit()
                    sampling_timer = None
                    print("✅ 硬件定时器已停止")
                
                await ws.send(ujson.dumps({
                    "t": "0",
                    "f1": 0.0,
                    "f2": 0.0,
                    "r": SAMPLE_RATE_FIXED,
                    "n": sample_count,
                    "msg": "已停止测量"
                }))
            
            elif cmd == "gc":
                # 手动垃圾回收
                freed = gc.collect()
                await ws.send(ujson.dumps({
                    "t": "0",
                    "f1": 0.0,
                    "f2": 0.0,
                    "r": SAMPLE_RATE_FIXED,
                    "n": sample_count,
                    "msg": f"内存回收完成: {gc.mem_free()}B"
                }))
                print(f"垃圾回收，当前内存: {gc.mem_free()} bytes free")
            
    except Exception as e:
        print(f"连接异常：{e}")
    
    finally:
        print("设备断开连接，清理资源")
        
        if sampling_task and not sampling_task.done():
            sampling_task.cancel()
            try:
                await sampling_task
            except:
                pass
        
        if send_task and not send_task.done():
            send_task.cancel()
            try:
                await send_task
            except:
                pass
        
        if sampling_timer:
            sampling_timer.deinit()
            sampling_timer = None
        
        data_buffer.clear()
        gc.collect()
        print(f"资源清理完成")

# -------------------------- WiFi AP模式 --------------------------
def setup_ap_mode():
    """开启板载WiFi AP模式"""
    ap = network.WLAN(network.AP_IF)
    
    sta = network.WLAN(network.STA_IF)
    sta.active(False)
    
    ap.config(essid=ap_ssid, password=ap_password, channel=6)
    ap.ifconfig((ap_ip, '255.255.255.0', ap_ip, ap_ip))
    ap.active(True)
    
    print("正在启动板载WiFi AP...")
    start_time = time.time()
    timeout = 10
    
    while not ap.active():
        led.on()
        time.sleep(0.5)
        led.off()
        time.sleep(0.5)
        
        if time.time() - start_time > timeout:
            print("AP启动失败！")
            return False
    
    ap_info = ap.ifconfig()
    print(f"✅ 板载WiFi AP启动成功！")
    print(f"📶 SSID: {ap_ssid}")
    print(f"🔑 密码: {ap_password}") 
    print(f"🌐 访问地址: http://{ap_info[0]}")
    
    led.on()
    return True

# -------------------------- 程序入口 --------------------------
if __name__ == "__main__":
    print(f"程序启动，初始内存: {gc.mem_free()} bytes free")
    
    if not setup_ap_mode():
        print("❌ AP模式启动失败，程序退出")
        exit()
    
    print("\n🚀 双传感器牛顿第三定律实验系统启动")
    print(f"🌐 请访问: http://{ap_ip}")
    print(f"📊 当前内存: {gc.mem_free()} bytes free")
    print(f"📊 缓冲区大小: {BUFFER_SIZE} 个数据点")
    print(f"📊 固定采样率: {SAMPLE_RATE_FIXED}Hz")
    print(f"📊 传感器1校准: 参考单位={reference_unit_1}, 偏移量={force1_offset}")
    print(f"📊 传感器2校准: 参考单位={reference_unit_2}, 偏移量={force2_offset}")
    print("\n💡 系统特点:")
    print("   ✅ 双HX711传感器同步测量")
    print("   ✅ 相互作用力对称显示")
    print("   ✅ 硬件定时器同步采样（20Hz）")
    print("\n🔧 支持指令:")
    print("   Flag1 - 双传感器置零")
    print("   Flag2 - 启动测量")
    print("   Flag3 - 停止测量")
    print("   gc - 手动垃圾回收")
    
    app.run(debug=False, port=80, host='0.0.0.0')