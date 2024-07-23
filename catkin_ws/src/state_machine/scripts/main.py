#!/usr/bin/env python

'''
ROS 
node: state_machine
Publishes:
    - TargetPID
Subscribes:
    - ProcessedIMU

Maintainer: Luke Deffenbaugh
'''

from std_msgs.msg import String
import rospy, sys, os, re
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))


from StateMachine import State_Machine as StateMachine
from state_task import Task_Item

# declare it here so it can be initialized elsewhere
sm = None

# Create a rate for entering loops
rate = rospy.rate(10)
        
######################### Add tasks here #########################

#task to level the sub
def level_1():
    """set x and y targets to 0 so that the PID will level the sub"""
    sm.x_target = 0
    sm.y_target = 0
# Put milestones together to create one task (tuple)
levelSub = (level_1, )

##################################################################

# Task to do a roll (broken into functions so that progress can be easily checked)
def roll_1():
    """Set Y (can change to x) to 90 degrees"""
    sm.y_target = 0.5
    sm.x_target = 0

    # allow a .05 margin of error
    while (IMU[1] < 0.45):
        rate.sleep()
    

def roll_2():
    """Set Y to 180 degrees"""
    sm.y_target = 1 # May cause an issue with that 1 and -1 are in the exact same place however it is unlikely to ever reach this
    sm.x_target = 0

    # allow a .05 margin of error (must be absolute value because it may cross 1 and then is negative)
    while (abs(IMU[1]) < 0.95):
        rate.sleep()

def roll_3():
    """ Set Y to 270 degrees """
    sm.y_target = -0.5
    sm.x_target = 0

    # allow a .05 margin of error
    while (IMU[1] < -0.55):
        rate.sleep()

def roll_4():
    """ Set back to level """
    sm.y_target = 0
    sm.x_target = 0
    
    # allow a .05 margin of error
    while (IMU[1] < -.05):
        rate.sleep()

rollSub = (roll_1, roll_2, roll_3, roll_4)
##################################################################
# Create the list of tasks to be passed to the state machine
task_list = {"LevelSub" : levelSub, "RollSub" : rollSub}


def fpub():
    """publish all current data immediately
    """
    pub.publish(sm.pidTargetTopic())

def callbackIMU(msg):
    """Simple callback function for IMU 
    I may change this to not use a global variable
    IMU : tuple x, y, z values for rotation

    Args:
        msg (str): msg.data is the message from the topic
    """
    global IMU

    # Search the topic data for the orientation x, y, and z
    pattern = r"Orientation:\[([-\d\.]+),([-\d\.]+),([-\d\.]+)\]"
    match = re.search(pattern, msg.data)
    
    if match:
        # Extract x, y, and z
        x = float(match.group(1))
        y = float(match.group(2))
        z = float(match.group(3))
        IMU = (x, y, z)







if __name__ == "__main__":
    # create a statemachine object using a tuple of task objects from the dictionary `task_list`
    sm = StateMachine((Task_Item(key, value) for key, value in task_list.items()))
    
    # Ros setup
    rospy.init_node('state_machine', anonymous = True)
    #Publishers
    pub = rospy.Publisher('TargetPID', String, queue_size=10)
    #Subscribers
    rospy.Subscriber('ProcessedIMU', String, callbackIMU)
    
    
    # This sets the proper PID controllers to 0 so that the sub will level
    sm.Call("LevelSub")

    # sleep for one minute
    sleep(60)

    # Just an example of how you might roll the sub
    sm.Call("RollSub")


    fpub()


    # This is just to keep the node from dying... I suppose I don't need to call fpub() 
    while not rospy.is_shutdown():
        fpub()
        rate.sleep()

