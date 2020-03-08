#!/usr/bin/python

# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
from ctypes import *
import smbus2 as smbus
# import smbus


class VL53L0XError(RuntimeError):
    pass


class VL53L0XAccuracyMode:
    GOOD = 0  # 33 ms timing budget 1.2m range
    BETTER = 1  # 66 ms timing budget 1.2m range
    BEST = 2  # 200 ms 1.2m range
    LONG_RANGE = 3  # 33 ms timing budget 2m range
    HIGH_SPEED = 4  # 20 ms timing budget 1.2m range


class VL53L0XDeviceMode:
    SINGLE_RANGING = 0
    CONTINUOUS_RANGING = 1
    SINGLE_HISTOGRAM = 2
    CONTINUOUS_TIMED_RANGING = 3
    SINGLE_ALS = 10
    GPIO_DRIVE = 20
    GPIO_OSC = 21


class VL53L0XGpioAlarmType:
    OFF = 0
    THRESHOLD_CROSSED_LOW = 1
    THRESHOLD_CROSSED_HIGH = 2
    THRESHOLD_CROSSED_OUT = 3
    NEW_MEASUREMENT_READY = 4


class VL53L0XInterruptPolarity:
    LOW = 0
    HIGH = 1

# Load VL53L0X shared lib
_TOF_LIBRARY = CDLL("bin/vl53l0x_python.so")

# Create read write function pointer
_I2C_READ_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
_I2C_WRITE_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)


class VL53L0X(object):
    object_number = 0

    def __init__(self, i2c_bus=1, address=0x29):
        """Initialize the VL53L0X ToF Sensor from ST"""
        self.my_object_number = VL53L0X.object_number
        VL53L0X.object_number += 1

        self._i2c_bus = i2c_bus
        self._device_addr = address
        self._i2c = smbus.SMBus()
        self._dev = None

    def open(self):
        # if self._device_addr != 0x29:
        #     self.change_address(self._device_addr)

        self._i2c.open(bus=self._i2c_bus)
        self._configure_i2c_library_functions()
        self._dev = True
        
    
    def close(self):
        self._i2c.close()
        self._dev = None
        
    def _configure_i2c_library_functions(self):
        def _i2c_read(address, reg, data_p, length):
            ret_val = 0
            result = []

            try:
                result = self._i2c.read_i2c_block_data(address, reg, length)
            except IOError:
                ret_val = -1

            if ret_val == 0:
                for index in range(length):
                    data_p[index] = result[index]
            return ret_val

        def _i2c_write(address, reg, data_p, length):
            ret_val = 0
            data = []

            for index in range(length):
                data.append(data_p[index])
            try:
                self._i2c.write_i2c_block_data(address, reg, data)
            except IOError:
                ret_val = -1
            return ret_val
        
        self._i2c_read_func = _I2C_READ_FUNC(_i2c_read)
        self._i2c_write_func = _I2C_WRITE_FUNC(_i2c_write)

        _TOF_LIBRARY.VL53L0X_set_i2c(self._i2c_read_func, self._i2c_write_func)

    def start_ranging(self, mode=VL53L0XAccuracyMode.GOOD):
        """Start VL53L0X ToF Sensor Ranging"""
        _TOF_LIBRARY.startRanging(self.my_object_number, mode, self._device_addr,255, 0)

    def stop_ranging(self):
        """Stop VL53L0X ToF Sensor Ranging"""
        _TOF_LIBRARY.stopRanging(self.my_object_number)

    def get_distance(self):
        """Get distance from VL53L0X ToF Sensor"""
        return _TOF_LIBRARY.getDistance(self.my_object_number)

    # This function included to show how to access the ST library directly
    # from python instead of through the simplified interface
    def get_timing(self):
        Dev = POINTER(c_void_p)
        Dev = _TOF_LIBRARY.getDev(self.my_object_number)
        budget = c_uint(0)
        budget_p = pointer(budget)
        Status = _TOF_LIBRARY.VL53L0X_GetMeasurementTimingBudgetMicroSeconds(
            Dev, budget_p)
        if (Status == 0):
            return (budget.value + 1000)
        else:
            return 0
    

    def change_address(self, new_addr):
         # Resgiter Address
        ADDR_UNIT_ID_HIGH = 0x16 # Serial number high byte
        ADDR_UNIT_ID_LOW = 0x17 # Serial number low byte
        ADDR_I2C_ID_HIGH = 0x18 # Write serial number high byte for I2C address unlock
        ADDR_I2C_ID_LOW = 0x19 # Write serial number low byte for I2C address unlock
        ADDR_I2C_SEC_ADDR = 0x8a # Write new I2C address after unlock

        if self._dev is not None:
            raise VL53L0XError('Error changing VL53L0X address')

        self._i2c.open(bus=self._i2c_bus)

        # read value from 0x16,0x17
        high = self._i2c.read_byte_data(self._device_addr, ADDR_UNIT_ID_HIGH)
        low = self._i2c.read_byte_data(self._device_addr, ADDR_UNIT_ID_LOW)

        # write value to 0x18,0x19
        self._i2c.write_byte_data(self._device_addr, ADDR_I2C_ID_HIGH, high)
        self._i2c.write_byte_data(self._device_addr, ADDR_I2C_ID_LOW, low)

        # write new_address to 0x1a
        self._i2c.write_byte_data(self._device_addr, ADDR_I2C_SEC_ADDR, new_addr)

        self._device_addr = new_addr
        
        self._i2c.close()