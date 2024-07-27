#!/usr/bin/env python3

'''
ROS
---

node:
----
    - pid_controller

Publishes:
---------
    - pid_motor_command (10 Hz)

Subscribes:
----------
    - x_pid_target
    - y_pid_target
    - z_pid_target
    - fb_pid_target
    - lr_pid_target
    - depth_pid_target

'''

# Begin Imports
from pid_controller import PID_controller

import rospy
from std_msgs.msg import Int32MultiArray, Float32, Float32MultiArray
# End Imports





if __name__ == '__main__':
    # PID settings for each controller
    # error, int, prop, der, bias, setpoint
    pid_configs = {
        'x_pid':     [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0],
        'y_pid':     [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0],
        'z_pid':     [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0],
        'fb_pid':    [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0],
        'lr_pid':    [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0],
        'depth_pid': [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
    }

    # Initialize the ROS node
    rospy.init_node('pid_controller', anonymous=True)
    
    # Create PID controller instance
    pid_controller = PID_controller(1)

    # Initialize PID controllers
    pid_ids = {}
    for name, config in pid_configs.items():
        pid_id = pid_controller.init_pid(*config)
        pid_ids[name] = pid_id
    
    # Create subscribers for each PID target topic
    for name in pid_ids:
        rospy.Subscriber(f"{name}_target", Float32, pid_controller.ros_update_reading, (pid_ids[name], ))

    # Initialize publisher for motor command
    pid_controller.init_publisher('pid_motor_command')


    # Start the PID controller spin
    pid_controller.spin()