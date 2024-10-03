#!/usr/bin/env python3

'''
ROS
---

node:
----
        - motor_commander

Publishes:
---------
        - motor_command

Subscribes:
----------

'''


"""

! This is just a test driver function not used in comp

"""



# BEGIN IMPORT
import rospy
from std_msgs.msg import Int32MultiArray
import time
# END IMPORT

class Limiter:
    def __init__(self, unitsPerSecond):
        self.unitsPerSecond = unitsPerSecond
        self.lastTime = time.time()
        self.lastValue = 0
    #Should be constantly called
    def calculate(self, value):
        currentTime = time.time()
        timeElapsed = currentTime - self.lastTime
        self.lastTime = currentTime
        v = value - self.lastValue
        direction = -1 if v < 0 else 1
        valueIncreaseWanted = abs(v) / self.unitsPerSecond
        if valueIncreaseWanted <= timeElapsed:
            self.lastValue = value
            return value
        else:
            self.lastValue = self.lastValue + direction * (timeElapsed * self.unitsPerSecond)
            
def getPowers(pow, rot, ver):
    #I dont remember the diagram that had the powers.
    #This assumes:
    # 1,2 positive moves up
    # 3,4 positive moves down
    # 5 positive forward and CW
    # 6 positive forward and CCW
    # 7 positive backwards and CCW
    # 8 positive backwards and CW
    
    """
    motor_to_directions = [
            [1, 1, -1, -1, 0, 0, 0, 0], # 'x-axis'
            [1, -1, 1, -1, 0, 0, 0, 0], # 'y-axis'
            [0, 0, 0, 0, 1, 1, 1, 1], # 'z-axis' 
            [0, 0, 0, 0, 1, 1, -1, -1], # 'pitch' 
            [1, -1, -1, 1, 0, 0, 0, 0], # 'yaw' 
            [0, 0, 0, 0, 1, -1, 1, -1], # 'roll'
            # Add depth control
        ]
    """
    
    #If rot != 0, then pow will not be accounted for
    #Clockwise is positive
    
    return [
        pow if rot == 0 else rot,
        -pow if rot == 0 else -rot,
        pow if rot == 0 else -rot,
        -pow if rot == 0 else rot,
        ver,
        ver,
        ver,
        ver
    ]

def getBarrelRoll(power):
    #I think I'm dumb but maybe you can run all 4? I know this will barrel roll buy maybe p, -p, -p, p will do the barrel roll?
    return [
        0,
        0,
        0,
        0,
        1,
        0,
        -1,
        0
    ]

from typing import List

num_motors = 8

high: List[int] = [20 for i in range(num_motors)]
low: List[int] = [30 for i in range(num_motors)]
list_thing = high

motor_powers = [0 for i in range(num_motors)]

hl_counter = 0

def commander():
    global hl_counter
    pub = rospy.Publisher('motor_command', Int32MultiArray, queue_size=10)
    rospy.init_node('motor_commander', anonymous=True)
    rate = rospy.Rate(.1) # 10hz
    unitsPerSec = 100
    
    limiters = [Limiter(unitsPerSec) for i in range(num_motors)]
    
    #! This is just a section to show the motors running when there is seperate input from ros this should not be used
    while not rospy.is_shutdown():
        wantedPower = 0
        wantedRot = 50
        wantedVer = 0
        #Swap that for the barrel roll function to get a barrel roll :)
        powers = getPowers(wantedPower, wantedRot, wantedVer)
        for i in range(num_motors):
            motor_powers[i] = limiters[i].calculate(powers[i])
        
        rospy.loginfo(motor_powers)
        pub.publish(data=motor_powers)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass