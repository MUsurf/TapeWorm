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
import numpy as np
# import time
# END IMPORT

from typing import List

num_motors = 8 

array_of_array = np.array(20 for i in range(num_motors))


# hl_counter = 0

def commander():
    # global hl_counter
    chosen_list = list_thing[0]
    pub = rospy.Publisher('motor_command', Int32MultiArray, queue_size=10)
    rospy.init_node('motor_commander', anonymous=True)
    rate = rospy.Rate(.1) # 10hz

    #! This is just a section to show the motors running when there is seperate input from ros this should not be used
    while not rospy.is_shutdown():
        # chosen_list = list_thing[hl_counter]
        # if (hl_counter == 0):
        #     list_thing = high
        # elif (hl_counter == 1):
        #     list_thing = medium
        # else:
        #     list_thing = low
        # hl_counter = (hl_counter + 1) % 3
        # # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(list_thing)
        scalerx = .3
        scalery = .3
        scalerx1 = where_turn(scalerx,1)
        scalery1 = where_fly(scalery,2)
        if(scalerx1 != scalerx or scalery1 != scalery):
            jelly_move(scalerx1,scalery1)
        # print(array_of_array)
        # time.sleep(1)
        pub.publish(data=array_of_array)
        rate.sleep()

def jelly_move(scalerx,scalery):
    global array_of_array
    array_of_array =  jelly_turn()*scalerx + jelly_fly()*scalery


 #positive = left negative = right
def jelly_turn():
    array = np.array([0,0,0,0,-100,100,-100,100])
    return array
#positive = up negative = down
def jelly_fly():
    array = np.array([-100,-100,-100,-100,0,0,0,0])
    return array

def where_turn(scalerx,dir):
    if(dir == 1):
        scalerx *= -1
    return scalerx

def where_fly(scalery,dir):
    if(dir == 1):
        scalery *= -1
    return scalery

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass