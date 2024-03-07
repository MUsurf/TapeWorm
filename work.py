from asyncore import ExitNow
from time import sleep
from Jetson.GPIO import gpio_pin_data
import Jetson.GPIO as GPIO
print("Bitches")  # Verify that program runs correctly

pwmpin1 = 15
GPIO.setmode(GPIO.BOARD)  # set pin numbering system
GPIO.setup(pwmpin1, GPIO.OUT)  # set up pin pwnpin1 as a pwm output pin

jetson_pwm = GPIO.PWM(pwmpin1, 20000)  # create PWM instance with frequency


jetson_pwm.start(0)  # start PWM of required Duty Cycle
sleep(3)
# jetson_pwm.ChangeDutyCycle(0)
# sleep(3)
jetson_pwm.ChangeDutyCycle(20)
sleep(3)
jetson_pwm.ChangeDutyCycle(0)
sleep(3)


print("kill me now")
for _ in range(0, 4):
    print('test')
    for _ in range(0, 101, 1):
        # provide duty cycle in the range 0-100
        jetson_pwm.ChangeDutyCycle(40)
        # throttle()
        # sleep(0.01)
        # stop()
        sleep(0.01)
    sleep(0.5)

    for _ in range(100, -1, -1):
        jetson_pwm.ChangeDutyCycle(0)
        sleep(0.01)
    sleep(0.5)
    print("cycle")
    # jetson_pwm.ChangeDutyCycle(0)


print('after')
jetson_pwm.ChangeDutyCycle(10)

sleep(10)


jetson_pwm.ChangeDutyCycle(0)
print("cleaning")
GPIO.cleanup(pwmpin1)
