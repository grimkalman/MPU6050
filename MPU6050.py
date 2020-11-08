import smbus
import RPi.GPIO as gpio
from statistics import mean
import sys

# Register addresses
SMPRT_DIV = 0x19
CONFIG = 0x1A
TEMP_OUT0 = 0x41
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B
PWR_MGMT_1 = 0x6B

class mpu6050():

    def __init__(self, device_address, bus=1):
        # Open communications
        self.bus = smbus.SMBus(bus)
        self.address = device_address
        
        # Wake up sensor with PLL with X axis gyroscope reference clock source
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 1)
        
        # Set digital low pass filter to 44/42 Hz (accel, gyro)
        self.bus.write_byte_data(self.address, CONFIG, 3)
        
        # Set sample rate to 200 Hz
        self.bus.write_byte_data(self.address, SMPRT_DIV, 4)
        
        # Offsets are set manually or by using the calibrate() function
        self.offsets = [656.676, -134.984, -758.4240000000009, -4670.28, -310.733, 134.869, 9.955]

    def get_raw_data(self):
        # Read all sensor registers at once in order:
        # ax, ay, az, temp, gx, gy, gz
        data = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)
        
        values = []
        for hi, lo in zip(data[0::2], data[1::2]):
            # Merge high  bits with low bits
            value  = (hi << 8) | lo
            
            # Get signed data
            if value & 0x8000:  # (faster than compare)
                value -= 0x10000
            values.append(value)
        return values
    
    def get_accel(self):
        # Read all sensor registers at once in order:
        # ax, ay, az
        data = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 6)
        
        values = []
        for idx, (hi, lo) in enumerate(zip(data[0::2], data[1::2])):
            # Merge high  bits with low bits
            value  = (hi << 8) | lo
            
            # Get signed data
            if value & 0x8000:  # (faster than compare)
                value -= 0x10000

            # Scale to G and save data
            values.append((value - self.offsets[idx]) / 16384.0)
        return values
    
    def get_gyro(self):
        # Read all sensor registers at once in order:
        # gx, gy, gz
        data = self.bus.read_i2c_block_data(self.address, GYRO_XOUT_H, 6)
        
        values = []
        for idx, (hi, lo) in enumerate(zip(data[0::2], data[1::2])):
            # Merge high  bits with low bits
            value  = (hi << 8) | lo
            
            # Get signed data
            if value & 0x8000:  # (faster than compare)
                value -= 0x10000
                
            # Scale to °/s and save data
            values.append((value - self.offsets[idx + 4]) / 131.0)
        return values
    
    def get_temp(self):
        # Read sensor registers
        hi, lo = self.bus.read_i2c_block_data(self.address, TEMP_OUT0, 2)
        
        # Merge high bits with low bits
        raw_temp  = (hi << 8) | lo
        
        # Get signed data
        if raw_temp & 0x8000:  # (faster than compare)
            raw_temp -= 0x10000
            
        # Scale to °C and return
        return (raw_temp / 340.0) + 36.53
    
    def calibrate(self):
        print("Calibrating MPU6050 - do not disturb")

        # Gather raw data to calculate the offset
        data = []
        for i in range(2000):
            data.append(self.get_raw_data())
            
            # Print progress bar
            if not i%100:
                print("[{}{}] - {}% \r".format(int(i/100) * "■", (19 - int(i/100)) * ".", int(i / 20)), end = '')

        

        # Use the mean as offsets
        for i in range(7):
            self.offsets[i] = mean([time_point[i] for time_point in data])
            
        # Subtract gravity in z-axis
        self.offsets[2] -= 16384
