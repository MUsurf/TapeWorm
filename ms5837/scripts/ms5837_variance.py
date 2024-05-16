#!/usr/bin/env python3

import rospy
import time
from ms5837.msg import ms5837_data
from nav_msgs.msg import Odometry

data = []
pub = rospy.Publisher('rov/ms5837_filtered', Odometry, queue_size=1)

def calculate(array):
    mean = 0
    #total = 0
    if(len(array)>50):
        array.pop(0)
    mean = sum(array)
    mean /= len(array)
    #for point in array:
    #    total += (point - mean) * (point - mean)
    #total /= len(array)
    print(mean)
    return mean


def callback(msg):
    data.append(msg.pose.pose.position.z)
    print(len(data))
    if(len(data) >= 50):
        msg.pose.pose.position.z = calculate(data)
        pub.publish(msg)



def main():
    rospy.init_node('depth_filter')
    time.sleep(20)
    while not rospy.is_shutdown():
        rospy.Subscriber('rov/depth_odom', Odometry, callback)
        rospy.spin()


if __name__ == '__main__':
    main()
