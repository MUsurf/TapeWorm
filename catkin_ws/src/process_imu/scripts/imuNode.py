#!/usr/bin/env python

'''
ROS 
node: imu_node
Publishes: 
Subscribes:
        - /IMUdata

Author: Luke Deffenbaugh
'''

import rospy
from sensor_msgs.msg import Imu


def imu_callback(data):
    '''
    # IMU Callback Function #
    This is the code that is called each time the subscriber receives data from the subscribed topic
    '''

    # Log the orientation
    rospy.loginfo("Orientation: x=%f, y=%f, z=%f, w=%f", 
                  data.orientation.x, 
                  data.orientation.y, 
                  data.orientation.z, 
                  data.orientation.w)
    # Log the Angular Velocity
    rospy.loginfo("Angular Velocity: x=%f, y=%f, z=%f", 
                  data.angular_velocity.x, 
                  data.angular_velocity.y, 
                  data.angular_velocity.z)
    # Log the linear acceleration
    rospy.loginfo("Linear Acceleration: x=%f, y=%f, z=%f", 
                  data.linear_acceleration.x, 
                  data.linear_acceleration.y, 
                  data.linear_acceleration.z)

    # TODO: Test that this works and learn to launch imu and process_imu nodes
        # - Format data as necessary and publish to a more usable topic

    # TODO: rename current imu to something more low level. This package (as far as the rest of our workspace is concerned) is imu

def imu_node():
    ''' 
    Create and handle ROS environment setup.
    '''

    # Setup Node
    rospy.init_node("imu_node", anonymous=False)
    # Setup Subscriber
    rospy.Subscriber("IMUdata", Imu, imu_callback)
    # Keep the node running
    rospy.spin()


############ Main #############
if __name__ == '__main__':
    try:
        # Run the node
        imu_subscriber()
    except rospy.ROSInterruptException:
        # Handle a ros interruption
        pass
    # Add special case handlers here
