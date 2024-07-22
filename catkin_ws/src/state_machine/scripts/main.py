#!/usr/bin/env python

'''
ROS 
node: state_machine
Publishes:
    - TargetPID
Subscribes:


Maintainer: Luke Deffenbaugh
'''

from std_msgs.msg import String
import rospy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from StateMachine import State_Machine as StateMachine
from state_task import Task_Item

# declare it here so it can be initialized elsewhere
sm = None
        
######################### Add tasks here #########################

#task to level the sub
def level_1():
    """set x and y targets to 0 so that the PID will level the sub"""
    sm.x_target = 0
    sm.y_target = 0
# Put milestones together to create one task (tuple)
levelSub = (level_1, )


##################################################################
# Create the list of tasks to be passed to the state machine
task_list = {"LevelSub" : levelSub}


def fpub():
    """publish all current data immediately
    """
    pub.publish(sm.pidTargetTopic())


if __name__ == "__main__":
    # create a statemachine object using a tuple of task objects from the dictionary `task_list`
    sm = StateMachine((Task_Item(key, value) for key, value in task_list.items()))
    
    # Ros setup
    rospy.init_node('state_machine', anonymous = True)
    #Publishers
    pub = rospy.Publisher('TargetPID', String, queue_size=10)
    #Subscribers
    ##### Don't currently need any #####
    
    
    # This sets the proper PID controllers to 0 so that the sub will level
    sm.Call("LevelSub")
    fpub()


    # This is just to keep the node from dying... I suppose I don't need to call fpub() 
    rate = rospy.Rate(10)  # 10 Hz loop rate
    while not rospy.is_shutdown():
        fpub()
        rate.sleep()

