from asyncore import ExitNow
from time import sleep
from Jetson.GPIO import gpio_pin_data
import Jetson.GPIO as GPIO
print("Bitches")  # Verify that program runs correctly

"""
def clear_pwm_pins(pwm_pins):
    # Set the mode to BOARD
    GPIO.setmode(GPIO.BOARD)

    # Stop PWM on each pin and cleanup GPIO
    for pin in pwm_pins:
        pwm = GPIO.PWM(pin, 1000)  # Create PWM object with any frequency
        pwm.stop()  # Stop PWM
        GPIO.cleanup(pin)  # Cleanup GPIO for the pin


# List of PWM pins to clear
pwm_pins = [15, 32, 33]  # Example PWM pins

# Call function to clear PWM pins
clear_pwm_pins(pwm_pins)
"""
pwmpin1 = 15  # PWM pin connected to one motor
pwmpin2 = 33  # PWM pin connected to another motor
GPIO.setmode(GPIO.BOARD)  # set pin numbering system
GPIO.setup(pwmpin1, GPIO.OUT)  # set up pin pwnpin1 as a pwm output pin

jetson_pwm = GPIO.PWM(pwmpin1, 20000)  # create PWM instance with frequency


# def throttle():
#     jetson_pwm.ChangeDutyCycle(50)


# def stop():
#     jetson_pwm.ChangeDutyCycle(0)

# According to Nikola, the duty cycle must start at 0, go up below 50, and back to 0 before cycling the power to the desired rates. This is the below block.


jetson_pwm.start(0)  # start PWM of required Duty Cycle
sleep(3)
# jetson_pwm.ChangeDutyCycle(0)
# sleep(3)
jetson_pwm.ChangeDutyCycle(100)
sleep(3)
jetson_pwm.ChangeDutyCycle(0)
sleep(3)

print("kill me now")
for _ in range(0, 4):
    for _ in range(0, 101, 1):
        # provide duty cycle in the range 0-100
        jetson_pwm.ChangeDutyCycle(50)
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

jetson_pwm.ChangeDutyCycle(20)

sleep(10)


jetson_pwm.ChangeDutyCycle(0)
print("cleaning")
GPIO.cleanup(pwmpin1)

raise NameError

try:
    print("kill me now")
    while True:
        for _ in range(0, 101, 1):
            # provide duty cycle in the range 0-100
            jetson_pwm.ChangeDutyCycle(100)
            # throttle()
            # sleep(0.01)
            # stop()
            sleep(0.01)
        sleep(0.5)

        for _ in range(100, -1, -1):
            jetson_pwm.ChangeDutyCycle(20)
            sleep(0.01)
        sleep(0.5)
        print("cycle")
        jetson_pwm.ChangeDutyCycle(0)

finally:
    jetson_pwm.ChangeDutyCycle(0)
    print("cleaning")
    GPIO.cleanup(pwmpin1)
