from asyncore import ExitNow
from time import sleep
from Jetson.GPIO import gpio_pin_data
import Jetson.GPIO as GPIO


# ten is the new zero

pwmpin1 = 33
GPIO.setmode(GPIO.BOARD)  # set pin numbering system
GPIO.setup(pwmpin1, GPIO.OUT)  # set up pin pwnpin1 as a pwm output pin

jetson_pwm = GPIO.PWM(pwmpin1, 24000)  # create PWM instance with frequency


jetson_pwm.start(0)
