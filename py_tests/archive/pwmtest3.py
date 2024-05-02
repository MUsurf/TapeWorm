import Jetson.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
channel = 15

GPIO.setup(channel, GPIO.OUT)

while True:
    GPIO.output(channel, GPIO.HIGH)

    time.sleep(1)
    GPIO.output(channel, GPIO.LOW)

    time.sleep(1)

GPIO.cleanup()
