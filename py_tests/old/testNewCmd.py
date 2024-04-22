# BEGIN IMPORT
import rospy
import busio
import time
from board import SCL, SDA
import adafruit_pca9685 as PCA9685


# ROS setup
rospy.init_node("motor_commander")
rate = rospy.Rate(100)

# I2C and PCA setup
i2c = busio.I2C(SCL, SDA)
pca = PCA9685.PCA9685(i2c, address=0x40) # I don't know what address I need rn
pca.frequency = 280  # Hz


class Motor():
    def __init__(self, pca = None):
        # Init the PCA channel defaulted to None
        self.pca = pca

    def setPCA(self, pca):
        # For now don't know if this setter function is neccessary
        self.pca = pca

class Commander():
    def __init__(self):
        