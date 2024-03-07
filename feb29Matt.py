from time import sleep
from Jetson.GPIO import gpio_pin_data
import Jetson.GPIO as GPIO
print("Bitches")  # Verify that program runs correctly


pwmpin1 = 15  # PWM pin connected to one motor
pwmpin2 = 33  # PWM pin connected to another motor
GPIO.setmode(GPIO.BOARD)  # set pin numbering system
GPIO.setup(pwmpin1, GPIO.OUT)  # set up pin pwnpin1 as a pwm output pin

frequency = 24000
jetson_pwm = GPIO.PWM(pwmpin1, frequency)  # create PWM instance with frequency


def stop():
    jetson_pwm.ChangeDutyCycle(0)


# According to Nikola, the duty cycle must start at 0, go up below 50, and back to 0 before cycling the power to the desired rates. This is the below block.

timeToSleep = 3


jetson_pwm.start(0)  # start PWM of required Duty Cycle
sleep(timeToSleep)
jetson_pwm.ChangeDutyCycle(40)
sleep(timeToSleep)
jetson_pwm.ChangeDutyCycle(0)
sleep(timeToSleep)


try:
    print("testing")

    while True:
        for _ in range(0, 101, 1):
            # provide duty cycle in the range 0-100
            jetson_pwm.ChangeDutyCycle(60)
            sleep(0.01)
        sleep(0.5)

        for _ in range(100, -1, -1):
            jetson_pwm.ChangeDutyCycle(0)
            sleep(0.01)
        sleep(0.5)

        jetson_pwm.ChangeDutyCycle(0)
        sleep(3)
except KeyboardInterrupt:
    print("Stopping Jetson on GPIO ")
    # print("Stopping Jetson on GPIO " + pwmpin1)
    jetson_pwm.stop()  # Stop pwm setup
finally:
    GPIO.cleanup(pwmpin1)
    print("Stopped...")


"""
# Change Frequency
frequency = 20000
jetson_pwm = GPIO.PWM(pwmpin1, frequency)  # create PWM instance with frequency


jetson_pwm.start(0)  # start PWM of required Duty Cycle
sleep(3)
jetson_pwm.ChangeDutyCycle(40)
sleep(3)
jetson_pwm.ChangeDutyCycle(0)
sleep(3)


# while True:
for duty in range(0, 101, 1):
    jetson_pwm.ChangeDutyCycle(75)  # provide duty cycle in the range 0-100
    sleep(0.01)
sleep(0.5)

for duty in range(100, -1, -1):
    jetson_pwm.ChangeDutyCycle(5)
    sleep(0.01)
sleep(0.5)

jetson_pwm.ChangeDutyCycle(0)
sleep(0.01)

jetson_pwm(stop)
GPIO.cleanup(pwmpin1)
"""
