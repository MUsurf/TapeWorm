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
    sm.x_target = .1
    sm.y_target = .5
# Put milestones together to create one task (tuple)
levelSub = (level_1)


##################################################################
# Create the list of tasks to be passed to the state machine
task_list = {"LevelSub" : levelSub}

def fpub():
    pub.publish(sm.pidTargetTopic())

if __name__ == "__main__":
    # create a statemachine object using a tuple of task objects from the dictionary `task_list`
    sm = StateMachine((Task_Item(key, value) for key, value in task_list.items()))
    
    
    # Ros setup
    rospy.init_node('state_machine', anonymous = True)
    pub = rospy.Publisher('TargetPID', String, queue_size=10)
    # TODO: When a topic to subscribe to exists... add it here
    # TODO: publish the information about tasks
    
    # TODO: Specifically call on which tasks need to happen
