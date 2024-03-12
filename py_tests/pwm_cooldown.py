import pandas as pd
import numpy as np
from random import randint
from time import sleep
from platform import machine
if (machine() != 'x86_64'):
    from Jetson.GPIO import gpio_pin_data
    import Jetson.GPIO as GPIO
    has_GPIO = True
else:
    has_GPIO = False

# Verify that program runs correctly /! not what this does at all
print(f'{__file__} is running')

# Figure out how to kill any pwm pins on a failure

# !!! This is all working as is motor on pin 15 is having range issues due to lack of calibration


class JetsonBoard:
    def __init__(self, pins=[], frequency=24000, step_size=5):
        if (has_GPIO):
            GPIO.setmode(GPIO.BOARD)  # set pin numbering system
            GPIO.setup(pins, GPIO.OUT)
            self.pwm_pins = [GPIO.PWM(i, frequency) for i in pins]
        # Don't think this is nessescary and can probably be deleted
        self.pinObjs = pins

        self.pinStates = [0 for i in pins]
        self.step_size = step_size
        self.frequency = frequency
        print('setup\n')

    def pinStep(self, targets=[]):
        directions = self.__targetDistance(targets)
        for x in range(len(directions)):
            if (directions[x] == 0):
                continue
            self.pinStates[x] += directions[x] * self.step_size
            if (has_GPIO):
                self.pwm_pins[x].ChangeDutyCycle(self.pinStates[x])

            print(f'{self.pinStates[x]}', end=' ')
        print('\n', end='')

    # directions must be one of the following: -1, 1, 0
    def __targetDistance(self, targets=[]):
        directions = []
        for i in range(len(targets)):
            value = (targets[i] - self.pinStates[i])
            if (value == 0):
                directions.append(0)
            elif (value > 0):
                directions.append(1)
            elif (value < 0):
                directions.append(-1)
            else:
                raise ValueError
        return (directions)

    # """
    # According to Nikola, the duty cycle must start at 0, go up below 50, and back to 0 before cycling the power to the desired rates. This is the below block. start PWM of required Duty Cycle
    # """

    # Takes exactly three seconds between each step
    def arming(self):
        for pin in self.pwm_pins:
            # Must be a starting value of 10 this is lowest signal wich isn't brake
            pin.start(10)
        sleep(5)
        # for pin in self.pwm_pins:
        #     pin.ChangeDutyCycle(40)
        # for _ in range(30):
        #     sleep(.2)
        # for pin in self.pwm_pins:
        #     pin.ChangeDutyCycle(0)
        # for _ in range(30):
        #     sleep(.2)
        # print('Armed')

    def cleanup(self):
        print("cleaning")
        for pin in self.pwm_pins:
            pin.stop()
        GPIO.cleanup()


# pins = [33, 15]
pins = [33, 15]
board = JetsonBoard(pins, step_size=2)

"""
    Create funct that takes a value and quickly moves the motor to this value
    target time is 1 sec for transition
    max change every 100 miliseconds 10 change in duty cycle 
    10 changes in duty cycle ensures in a one second segment all values will reach where they need to be
"""
major_step = 5
minor_step = .2

if (has_GPIO):
    board.arming()

while True:
    # if (scaler < 0):
    # board.targetDistance([scaler for i in range(len(pins))])
    try:
        sleep(major_step)
        targets = [((int)(randint(10, 50) / 10) * 10)
                   for i in range(len(pins))]
        # targets = [60]
        for x in range(20):
            board.pinStep(targets)
            sleep(minor_step)
        print(targets)
    except KeyboardInterrupt:
        for x in range(10):
            board.pinStep([0 for i in range(len(pins))])
            sleep(minor_step)
        print('\nramped-down\n')
        break


if (has_GPIO):
    board.cleanup()
