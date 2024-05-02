#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
import busio
import time
from board import SCL, SDA
import adafruit_pca9685 as PCA9685
# END IMPORT

# BEGIN STD_MSGS
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
# END STD_MSGS

# BEGIN SETUP
rospy.init_node("motor_commander")
rate = rospy.Rate(100)
i2c=busio.I2C(SCL,SDA)
pca = PCA9685.PCA9685(i2c, address = 0x40)
pca.frequency = 280 # Hz
# END SETUP

class MainLoop():
    def __init__(self):
        self.callback_count = 201
        self.motorNum = 8
        self.motor1 = pca.channels[7] # Correct
        self.motor2 = pca.channels[3] # Correct
        self.motor3 = pca.channels[6] # Correct
        self.motor4 = pca.channels[2] # Correct
        self.motor5 = pca.channels[0] # Correct
        self.motor6 = pca.channels[4] # Correct
        self.motor7 = pca.channels[1] # Correct
        self.motor8 = pca.channels[5] # Correct

    def microSec_to_duty(self,microSec,freq):
        samp_time = (1/freq) * 1000 * 1000 # Convert to Micro Sec
        duty_cycle = int((65536 * microSec)/(samp_time))
    #    print(duty_cycle)
        return duty_cycle

    def data_type(self,num):
        data = []
        for i in range(0,self.motorNum):
            data.append(0)

        for i in range(0,self.motorNum):
            data[i] = num

        return data

    def callback(self,message_rec):
        print("Data received is: " + str(message_rec.data))
        self.motor1.duty_cycle = self.microSec_to_duty(message_rec.data[0],pca.frequency)
        self.motor2.duty_cycle = self.microSec_to_duty(message_rec.data[1],pca.frequency)
        self.motor3.duty_cycle = self.microSec_to_duty(message_rec.data[2],pca.frequency)
        self.motor4.duty_cycle = self.microSec_to_duty(message_rec.data[3],pca.frequency)
        self.motor5.duty_cycle = self.microSec_to_duty(message_rec.data[4],pca.frequency) #this was also 3, why?
        self.motor6.duty_cycle = self.microSec_to_duty(message_rec.data[5],pca.frequency)
        self.motor7.duty_cycle = self.microSec_to_duty(message_rec.data[6],pca.frequency)
        self.motor8.duty_cycle = self.microSec_to_duty(message_rec.data[7],pca.frequency)

    def arm_seq(self):
        self.set_all(0)
        time.sleep(0.5)

        self.set_all(1000)
        time.sleep(0.5)

        self.set_all(500)
        time.sleep(0.5)

        self.set_all(1550)
  #      time.sleep(0.5) #FOR TESTING PURPOSES! REMOVE BEFORE USE!
  #      self.motor0.duty_cycle = self.microSec_to_duty(1600,pca.frequency)

    def set_all(self,PWM_setting):
        data = self.data_type(PWM_setting)
        print(data[0])
        print(int(pca.frequency))
        self.motor1.duty_cycle = self.microSec_to_duty(data[0],pca.frequency)
        self.motor2.duty_cycle = self.microSec_to_duty(data[1],pca.frequency)
        self.motor3.duty_cycle = self.microSec_to_duty(data[2],pca.frequency)
        self.motor4.duty_cycle = self.microSec_to_duty(data[3],pca.frequency)
        self.motor5.duty_cycle = self.microSec_to_duty(data[3],pca.frequency)
        self.motor6.duty_cycle = self.microSec_to_duty(data[4],pca.frequency)
        self.motor7.duty_cycle = self.microSec_to_duty(data[5],pca.frequency)
        self.motor8.duty_cycle = self.microSec_to_duty(data[6],pca.frequency)

    def clo_seq(self):
        self.set_all(0)

    def cut_motors(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if(data):
            pass
        else:
            self.clo_seq()

if __name__ == '__main__':
    try:
        loop = MainLoop()
        loop.arm_seq()
        while not rospy.is_shutdown():
            rospy.Subscriber("/command",Int32MultiArray,loop.callback)
            # rospy.Subscriber("volt_low", Bool, loop.cut_motors)
            rospy.spin()
    
    except KeyboardInterrupt:
        loop.clo_seq()
