#!/usr/bin/env python3

# BEGIN IMPORT
import busio
import time
from board import SCL_1, SDA_1
import adafruit_pca9685 as PCA9685
# END IMPORT

# BEGIN SETUP
i2c = busio.I2C(SCL_1, SDA_1)
pca = PCA9685.PCA9685(i2c, address=0x40)
pca.frequency = 60  # Hz
# END SETUP
try:
    loop = MainLoop()
    loop.arm_seq()
except KeyboardInterrupt:
    loop.clo_seq()
