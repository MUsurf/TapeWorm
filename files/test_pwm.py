import Jetson.GPIO as GPIO
from Jetson.GPIO import gpio_pin_data
from time import sleep

pwmpin1=25 # PWM pin connected to one motor
pwmpin2=33 # PWM pin connected to another motor
GPIO.setmode(GPIO.BOARD) # set pin numbering system
GPIO.setup(pwmpin1,GPIO.OUT)

jetson_pwm=GPIO(pwmpin1,16000) #create PWM instance with frequency
jetson_pwm.start(0) #start PWM of required Duty Cycle

while True:
	for duty in range(0,101,1):
		jetson_pwm.ChangeDutyCycle(50) #provide duty cycle in the range 0-100
		sleep(0.01)
	sleep(0.5)

	for duty in range(100,-1,-1):
		jetson_pwm.ChangeDutyCycle(50)
		sleep(0.01)
	sleep(0.5)
