"""
HX711 Load Cell Amplifier Library for Raspberry Pi Pico
优化版本 - 针对跳变问题
"""

import machine
import utime

class HX711:
    def __init__(self, dout_pin, sck_pin, gain=128):
        self.dout = machine.Pin(dout_pin, machine.Pin.IN, machine.Pin.PULL_DOWN)
        self.sck = machine.Pin(sck_pin, machine.Pin.OUT)
        self.gain = gain
        
        # 设置初始值
        self.OFFSET = 0
        self.reference_unit = 1
        self.last_valid_value = 0  # 上次有效值
        self.error_count = 0
        
        # 设置增益并初始化
        self.set_gain(gain)
        utime.sleep_ms(10)
        
    def set_gain(self, gain):
        if gain == 128:
            self.gain = 1
        elif gain == 64:
            self.gain = 3
        elif gain == 32:
            self.gain = 2
            
        # 脉冲时钟以设置增益
        self.sck.value(0)
        self._read_single()
        
    def is_ready(self):
        return self.dout.value() == 0
    
    def _read_single(self):
        """单次快速读取"""
        # 等待数据就绪（短超时）
        timeout = 0
        while not self.is_ready():
            timeout += 1
            if timeout > 100:  # 减少超时时间
                return self.last_valid_value
            utime.sleep_us(2)
            
        # 读取24位数据
        data = 0
        for i in range(24):
            self.sck.value(1)
            # 移除utime.sleep_us(1)以减少延迟
            self.sck.value(0)
            data = (data << 1) | self.dout.value()
            
        # 设置通道和增益
        for i in range(self.gain):
            self.sck.value(1)
            # 移除utime.sleep_us(1)
            self.sck.value(0)
            
        # 转换为有符号整数
        if data & 0x800000:
            data -= 0x1000000
            
        return data
    
    def read_fast(self):
        """快速读取，立即返回，不做复杂处理"""
        try:
            val = self._read_single()
            
            # 简单有效性检查
            if abs(val) > 0x7FFFFF or val == 0:  # 24位有符号范围
                self.error_count += 1
                if self.error_count < 3:
                    return self.last_valid_value
                else:
                    self.error_count = 0
                    return 0
            
            self.error_count = 0
            self.last_valid_value = val
            return val
            
        except Exception:
            return self.last_valid_value
    
    def read_long(self):
        """读取长整型值 - 用于高频读取"""
        return self.read_fast()
    
    def read_multiple(self, times=3):
        """多次读取，返回中值"""
        values = []
        for i in range(times):
            val = self._read_single()
            if val != 0:
                values.append(val)
        
        if not values:
            return self.last_valid_value
        
        # 取中值
        if len(values) >= 3:
            values.sort()
            return values[len(values)//2]
        else:
            return values[0]
    
    def read_average(self, times=5):
        """读取平均值，用于校准"""
        values = []
        for i in range(times):
            val = self.read_multiple(3)  # 每次读取使用中值
            values.append(val)
            utime.sleep_ms(1)
        
        if not values:
            return 0
        
        # 排序并去掉异常值
        if len(values) >= 3:
            values.sort()
            values = values[1:-1]  # 去掉最小和最大
        
        avg = sum(values) // len(values)
        self.last_valid_value = avg
        return avg
    
    def get_value(self, times=1):
        return self.read_long() - self.OFFSET
    
    def get_units(self, times=1):
        return self.get_value(times) / self.reference_unit
    
    def tare(self, times=10):
        avg = self.read_average(times)
        self.OFFSET = avg
        self.last_valid_value = avg
        return self.OFFSET
    
    def tare_A(self, times=10):
        """专门为通道A设计的tare函数"""
        avg = self.read_average(times)
        self.OFFSET = avg
        self.last_valid_value = avg
        return self.OFFSET
    
    def set_offset(self, offset):
        self.OFFSET = offset
        
    def set_reference_unit(self, reference_unit):
        self.reference_unit = reference_unit
        
    def power_down(self):
        self.sck.value(0)
        self.sck.value(1)
        
    def power_up(self):
        self.sck.value(0)
        
    def reset(self):
        self.power_down()
        self.power_up()
        
    def set_reading_format(self, byte_format="MSB", bit_format="MSB"):
        """设置读取格式（为了兼容性保留）"""
        pass