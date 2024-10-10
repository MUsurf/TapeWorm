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
# END IMPORT

from typing import List

num_motors = 8

if_screwed = 0

def sideways(quadrant, speed):

    power = []

    if ((speed < -50) or (speed > 50) or (not(quadrant == 1 or quadrant == 2 or quadrant == 3 or quadrant == 4))):
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

    if ((velocity < -50) or (velocity > 50)):
        if_screwed = 1
    else:
        power = [velocity, velocity, velocity, velocity, 0, 0, 0, 0]

    return power

def forward(velocity):

    power = []

    if ((velocity < -50) or (velocity > 50)):
        if_screwed = 1
    else:
        power = [0, 0, 0, 0, velocity, velocity, -velocity, -velocity]

    #movement_direction = movement_direction + power
    return power

def right(velocity, half):
    
    power = []

    if ((velocity < -50) or (velocity > 50)):
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

#high: List[int] = [20 for i in range(num_motors)]
#medium = [30 for i in range(num_motors)]
#low: List[int] = [30 for i in range(num_motors)]
#list_thing = [high, medium, low]

hl_counter = 0

def commander():
    global hl_counter
    chosen_list = movement_direction(0)
    pub = rospy.Publisher('motor_command', Int32MultiArray, queue_size=10)
    rospy.init_node('motor_commander', anonymous=True)
    rate = rospy.Rate(.1) # 10hz

    #! This is just a section to show the motors running when there is seperate input from ros this should not be used
    #while not rospy.is_shutdown():
    #    chosen_list = list_thing[hl_counter]
    #    hl_counter = (hl_counter + 1) % 3
    #    # hello_str = "hello world %s" % rospy.get_time()
    #    rospy.loginfo(list_thing)
    #    pub.publish(data=list_thing)
    #    rate.sleep()

    num_motors = 8

    num_of_xcalls = 3

    num_of_ycalls = 2

    listx1 = sideways(2, 45)
    listx2 = right(20, True)
    listx3 = forward(30)

    listy1 = up(49)
    listy2 = up(-24)

    allX = [0, 0, 0, 0, 0, 0, 0, 0]
    allY = [0, 0, 0, 0, 0, 0, 0, 0]

    movement_direction = [0, 0, 0, 0, 0, 0, 0, 0]

    for x in range(0, 8):
        allX[x] = (listx1[x] + listx2[x] + listx3[x])/num_of_xcalls

    for x in range(0, 8):
        allY[x] = (listy1[x] + listy2[x])/num_of_ycalls

    for x in range(0, 8):
        movement_direction[x] = allX[x] + allY[x]

    temp_movement_direction = [0, 0, 0, 0, 0, 0, 0, 0]

    for x in range(1, 51):
        for y in range(0, 8):
            temp_movement_direction[y] = movement_direction[y]*(x/50)
        rospy.loginfo(temp_movement_direction)
        pub.publish(list=temp_movement_direction)
        rate.sleep()

    for x in range(1, 51):
        for y in range(0, 8):
            temp_movement_direction[y] = movement_direction[y]*((50-x)/50)
        rospy.loginfo(temp_movement_direction)
        pub.publish(list=temp_movement_direction)
        rate.sleep()
    
    temp_movement_direction = [0, 0, 0, 0, 0, 0, 0, 0]

    rospy.loginfo(temp_movement_direction)
    pub.publish(list=temp_movement_direction)

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass