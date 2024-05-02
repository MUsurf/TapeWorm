#!/usr/bin/env python3

# BEGIN IMPORT
import busio
import time
from board import SCL, SDA
import adafruit_pca9685 as PCA9685
# END IMPORT


# BEGIN SETUP
i2c = busio.I2C(SCL, SDA)
pca = PCA9685.PCA9685(i2c, address=0x40)
pca.frequency = 280  # Hz
# END SETUP


class MainLoop():
    def __init__(self, local_channels, num_motors):
        self.callback_count = 201
        self.motorNum = num_motors
        # ? This does the same as bellow but allows us to dynamically change motors with something like command line args
        self.motors = []
        for channel in local_channels:
            self.motors.append(pca.channels[channel])

    def microSec_to_duty(self, microSec):
        samp_time = (1/pca.frequency) * 1000 * 1000  # Convert to Micro Sec
        duty_cycle = int((65536 * microSec)/(samp_time))
        print(f"duty cycle: {hex(duty_cycle)}")
        print(f"duty cycle: {duty_cycle}")
        return duty_cycle

    def set_motor_speed(self, motor_id, speed):
        '''Set the speed of a single motor'''
        # Example speed conversion, needs to be adapted based on actual motor characteristics
        # Mapping 0...100 to appropriate PWM range
        pwm_value = self.microSec_to_duty(1000 + (speed * 10))
        self.motors[motor_id].duty_cycle = pwm_value

    def callback(self, message_rec):
        print("Data received is: " + str(message_rec.data))
        for index in range(len(message_rec)):
            x = self.microSec_to_duty(
                message_rec[index])

            self.motors[index].duty_cycle = x
            print(f"type of var callback {type(x)}")

    def arm_seq(self):
        self.set_all(0)
        time.sleep(2)

        self.set_all(40)
        time.sleep(2)

        self.set_all(50)
        time.sleep(2)

        self.set_all(10)
        time.sleep(2)

    def set_all(self, PWM_setting):
        for index in range(len(self.motors)):
            self.set_motor_speed(index, PWM_setting)

    def clo_seq(self):
        self.set_all(0)

    def cut_motors(self, data):
        if (data):
            pass
        else:
            self.clo_seq()


# Takes 0-100
# set_all()


if __name__ == '__main__':
    local_channels = [0]
    num_motors = 1
    # Setup
    try:
        loop = MainLoop(local_channels, num_motors)
        loop.arm_seq()
        time.sleep(3)
    except KeyboardInterrupt:
        loop.clo_seq()
    # Driver
    try:
        while True:
            for x in range(1):
                loop.set_all(15)
                time.sleep(1.5)
            time.sleep(1)
            for x in range(1):
                loop.set_all(40)
                time.sleep(1.5)
            time.sleep(1)
    except KeyboardInterrupt:
        loop.clo_seq()
