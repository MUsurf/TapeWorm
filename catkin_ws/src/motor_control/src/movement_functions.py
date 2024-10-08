import time
import math
#import rospy
#from std_msgs.msg import Int32MultiArray
from typing import List

num_motors = 8

if_screwed = 0

def sideways(quadrant, speed):

    power = []

    if ((speed < -100) or (speed > 100) or (not(quadrant == 1 or quadrant == 2 or quadrant == 3 or quadrant == 4))):
        if_screwed = 1
    elif (quadrant == 1):
        power = [0, 0, 0, 0, speed, 0, 0 , -speed]
    elif (quadrant == 2):
        power = [0, 0, 0, 0, 0, speed, -speed, 0]
    elif (quadrant == 3):
        power = [0, 0, 0, 0, -speed, 0, 0 , speed]
    elif (quadrant == 4):
        power = [0, 0, 0, 0, 0, -speed, speed, 0]

    return power

def up(velocity):
    
    power = []

    if ((velocity < -100) or (velocity > 100)):
        if_screwed = 1
    else:
        power = [velocity, velocity, velocity, velocity, 0, 0, 0, 0]

    return power

def forward(velocity):

    power = []

    if ((velocity < -100) or (velocity > 100)):
        if_screwed = 1
    else:
        power = [0, 0, 0, 0, velocity, velocity, -velocity, -velocity]

    #movement_direction = movement_direction + power
    return power

def right(velocity, half):
    
    power = []

    if ((velocity < -100) or (velocity > 100)):
        if_screwed = 1
    elif (half == True):
        if (velocity >= 0):
            power = [0, 0, 0, 0, velocity, 0, velocity, 0]
        else:
            power = [0, 0, 0, 0, 0, -velocity, 0, -velocity]
    else:
        power = [0, 0, 0, 0, velocity, -velocity, velocity, -velocity]

    #movement_direction = movement_direction + power
    return power

if (if_screwed == 1):
    print("Mega Screwed")