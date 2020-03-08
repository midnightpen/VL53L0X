from VL53L0X import VL53L0X, VL53L0XAccuracyMode


if __name__ == "__main__":
    try:
        tof = VL53L0X(i2c_bus=5, address=0x29)

        tof.open()
        tof.start_ranging(mode=VL53L0XAccuracyMode.BETTER)

        for i in range(100):
            print("{} mm.".format(tof.get_distance()))

    except KeyboardInterrupt:
        pass
