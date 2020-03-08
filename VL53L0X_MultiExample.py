import time

import RPi.GPIO as GPIO
from VL53L0X import VL53L0X, VL53L0XAccuracyMode

sensor1_shutdown = 23
sensor2_shutdown = 22
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)

GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)
time.sleep(0.5)

if __name__ == "__main__":
    try:
        tof = VL53L0X(i2c_bus=5, address=0x29)

        GPIO.output(sensor1_shutdown, GPIO.HIGH)
        time.sleep(0.5)
        tof.change_address(0x33)
        tof.open()
        tof.start_ranging(mode=VL53L0XAccuracyMode.BETTER)
        for i in range(5):
            print("Test sensor 1 {} mm.".format(tof.get_distance()))

        GPIO.output(sensor2_shutdown, GPIO.HIGH)
        time.sleep(0.5)
        tof1 = VL53L0X(i2c_bus=5, address=0x29)

        # time.sleep(10)
        # tof.change_address(0x30)
        tof1.open()
        tof1.start_ranging(mode=VL53L0XAccuracyMode.BETTER)    
        for i in range(5):
            print("Test sensor 2 {} mm.".format(tof.get_distance()))

        time.sleep(5)
        for i in range(500):
            print("{} mm./ {} mm.".format(tof.get_distance(),tof1.get_distance()))
     
        tof.close()
        tof1.close()
    except KeyboardInterrupt:
        pass

    finally:

        GPIO.cleanup()

