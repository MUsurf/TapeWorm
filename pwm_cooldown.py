from random import randint
from time import sleep
from Jetson.GPIO import gpio_pin_data
import Jetson.GPIO as GPIO
print(f'{__file__} is running')  # Verify that program runs correctly


class PinObj:
    def __init__(self, pinNum):
        self.pinValue = pinNum
        # Must always be between 0-100
        self.value = 0


class JetsonBoard:
    def __init__(self, pins=[], frequency=24000, step_size=5):
        # GPIO.setmode(GPIO.BOARD) # set pin numbering system
        self.pinObjs = [PinObj(i) for i in pins]
        self.step_size = step_size

        for pin in self.pinObjs:
            # GPIO.setup(pin, GPIO.OUT)
            print('setup\n')
        # self.pwm_pins = [GPIO.PWM(i, frequency) for i in self.pinObjs]

    # directions must be -1, 1, 0
    def pinStep(self, targets=[]):
        directions = self.__targetDistance(targets)
        for x in range(len(directions)):
            if (directions[x] == 0):
                continue
            self.pinObjs[x].value = self.pinObjs[x].value + \
                (directions[x] * self.step_size)
            # self.pwm_pins[x].ChangeDutyCycle(self.pwm_pins[x].value)
            print(f'{self.pinObjs[x].value}', end=' ')
        print('\n', end='')

    def __targetDistance(self, targets=[]):
        directions = []
        for i in range(len(targets)):
            value = (targets[i] - self.pinObjs[i].value)
            if (value == 0):
                directions.append(0)
            elif (value > 0):
                directions.append(1)
            elif (value < 0):
                directions.append(-1)
            else:
                raise ValueError
        return (directions)

    """
    According to Nikola, the duty cycle must start at 0, go up below 50, and back to 0 before cycling the power to the desired rates. This is the below block. start PWM of required Duty Cycle
    """

    def arming(self):
        for pin in self.pwm_pins:
            pin.start(0)
        sleep(3)
        for pin in self.pwm_pins:
            pin.ChangeDutyCycle(40)
        sleep(3)
        for pin in self.pwm_pins:
            pin.ChangeDutyCycle(0)
        sleep(3)

    def cleanup(self):
        for pin in self.pwm_pins:
            pin.stop()
        GPIO.cleanup()


pins = [1, 5, 10, 15]
board = JetsonBoard(pins, step_size=10)

"""
    Create funct that takes a value and quickly moves the motor to this value
    target time is 1 sec for transition
    max change every 100 miliseconds 10 change in duty cycle 
    10 changes in duty cycle ensures in a one second segment all values will reach where they need to be
"""
major_step = 3
minor_step = .2

while True:
    # if (scaler < 0):
    # board.targetDistance([scaler for i in range(len(pins))])
    try:
        sleep(major_step)
        targets = [((int)(randint(0, 100) / 10) * 10)
                   for i in range(len(pins))]
        for x in range(10):
            board.pinStep(targets)
            sleep(minor_step)
        print(targets)
    except KeyboardInterrupt:
        for x in range(10):
            board.pinStep([0 for i in range(len(pins))])
            sleep(minor_step)
        print('\nramped-down\n')
        break

board.cleanup()
