from typing import List
import numpy as np
import random as rd
import time

import rospy
from std_msgs.msg import Int32MultiArray


rate = rospy.Rate(100)


pub = rospy.Publisher('keyboard_input', Int32MultiArray, queue_size=10)
rospy.init_node('keyboard_input', anonymous=True)



#! Assuming all motors can only drive forward
#! When they can go backward change the allowed range of inputs not the code

class Translation_Layer():
    def __init__(self, max_, min_):
        self.num_motors = np.array([1.0 for _ in range(8)])
        # Clamp values between these values
        self.max = max_
        self.min = min_
    
    def forward_back(self, percent_drive):
        result = percent_drive * self.num_motors[0:4]
        print(f'forward_back\t{result}')
        self.num_motors[0:4] = result
    
    def ascend_descend(self, percent_drive):
        result = percent_drive * self.num_motors[4::]
        print(f'ascend_descend\t{result}')
        self.num_motors[4::] = result

    def yaw(self, percent_drive):
        result = percent_drive * self.num_motors[[1, 2]]
        print(f'yaw\t{result}')
        self.num_motors[[1, 2]] = result
    
    def roll(self, percent_drive):
        result = percent_drive * self.num_motors[[4, 6]]
        print(f'roll\t{result}')
        self.num_motors[[4, 6]] = result
    
    def pitch(self, percent_drive):
        result = percent_drive * self.num_motors[[4, 5]]
        print(f'pitch\t{result}')
        self.num_motors[[4, 5]] = result
    
    def publisher(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.num_motors)
            pub.publish(data=self.num_motors)
            rate.sleep()



if __name__ == '__main__':
    trans_layer = Translation_Layer(1, 0)
    while True:
        trans_layer.yaw(.70)
        trans_layer.roll(.80)

        print(trans_layer.num_motors)
        time.sleep(1)
