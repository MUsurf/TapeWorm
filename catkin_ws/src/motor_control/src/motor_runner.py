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

'''high: List[int] = [20 for i in range(num_motors)]
medium: List[int] = [30 for i in range(num_motors)]
low: List[int] = [40 for i in range(num_motors)]
list_thing = [high, medium, low]'''
counter = 10

hl_counter = 0

def commander():
    '''global hl_counter
    chosen_list = list_thing[0]'''
    pub = rospy.Publisher('motor_command', Int32MultiArray, queue_size=10)
    rospy.init_node('motor_commander', anonymous=True)
    rate = rospy.Rate(.3) # 10hz

    #! This is just a section to show the motors running when there is seperate input from ros this should not be used
    while not rospy.is_shutdown():
        #chosen_list = list_thing[hl_counter]
       # hl_counter = (hl_counter + 1) % 3
        # hello_str = "hello world %s" % rospy.get_time()
        while (counter < 100):
            arr = [counter for i in range(num_motors)]
            print(arr)
            counter = counter + 10
            rospy.loginfo(arr)
            pub.publish(data=arr)
            rate.sleep()
        while (counter >= 10):
            arr = [counter for i in range(num_motors)]
            print(arr)
            counter = counter - 10
            rospy.loginfo(arr)
            pub.publish(data=arr)
            rate.sleep()
        arr = [100 for i in range(num_motors)]
        print(arr)
        #rospy.loginfo(arr)
        #pub.publish(data=arr)
        #rate.sleep()




       

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass