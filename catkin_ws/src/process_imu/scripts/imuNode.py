#!/usr/bin/env python3

'''
ROS 
node: imu_node
Publishes: 
Subscribes:
        - /IMUdata

Maintainer: Luke Deffenbaugh
'''

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import re

# def quaternion_to_euler(x, y, z, w):
#     # Roll (x-axis rotation)
#     sinr_cosp = 2 * (w * x + y * z)
#     cosr_cosp = 1 - 2 * (x * x + y * y)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)

#     # Pitch (y-axis rotation)
#     sinp = 2 * (w * y - z * x)
#     if np.abs(sinp) >= 1:
#         pitch = np.sign(sinp) * (np.pi / 2)  # Use 90 degrees if out of range
#     else:
#         pitch = np.arcsin(sinp)

#     # Yaw (z-axis rotation)
#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

def quaternion_to_euler(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 


class imuData:
    def __init__(self, data):
        '''
        Hold the data in specific formats

        Orientation: Holds poor usage of quaternions
            0 : x
            1 : y
            2 : z

        Quaternion: Holds the raw Quaternion incase we need it
            0 : w
            1 : x
            2 : y
            3 : z

        Angular Velocity: < Need to determine what its unit is >
            0 : X
            1 : Y
            2 : Z

        Linear Acceleration: < Need to determine what its unit is >
            0 : X
            1 : Y     
            2 : Z   
        '''

        # Transform the data into Roll Pitch and Yaw
        # euler = quaternion_to_euler(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.x, self.y, self.z, self.w = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w

        # Store the data into tuples
        self.orientation = (-1 * self.x if self.w < 0 else self.x, -1 * self.y if self.w < 0 else self.y, -1 * self.z if self.w < 0 else self.z)
        self.angular_velocity = (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
        self.linear_acceleration = (data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)

    def __repr__(self):
        return (f"Orientation:[{self.orientation[0]},{self.orientation[1]},{self.orientation[2]}] "
                f"Quaternion: [{self.w},{self.x},{self.y},{self.z}] "
                f"Angular_Velocity:[{self.angular_velocity[0]},{self.angular_velocity[1]},{self.angular_velocity[2]}] "
                f"Linear_Acceleration:[{self.linear_acceleration[0]},{self.linear_acceleration[1]},{self.linear_acceleration[2]}]")




def imu_callback(data):
    '''
    # IMU Callback Function #
    This is the code that is called each time the subscriber receives data from the subscribed topic
    '''

    # Initialize Object
    imudata = imuData(data)

    # Publish a string of the __repr__ function output
    imu_string_publisher.publish(str(imudata))

def imu_node():
    ''' 
    Create and handle ROS environment setup.
    '''

    # Setup Node
    rospy.init_node("imu_node", anonymous=True)
    # Setup Subscriber
    rospy.Subscriber("IMUdata", Imu, imu_callback)

    # Create a publisher
    global imu_string_publisher
    imu_string_publisher = rospy.Publisher("ProcessedIMU", String, queue_size=10)


    # Keep the node running
    rospy.spin()


############ Main #############
if __name__ == '__main__':
    try:
        # Run the node
        imu_node()
    except rospy.ROSInterruptException:
        # Handle a ros interruption
        pass
    # Add special case handlers here
