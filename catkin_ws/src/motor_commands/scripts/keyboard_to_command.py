import keyboard

from typing import List
import numpy as np

import rospy
from std_msgs.msg import Int32MultiArray






#! Assuming all motors can only drive forward
#! When they can go backward change the allowed range of inputs not the code

class Translation_Layer():
    def __init__(self, max_, min_):
        self.num_motors = np.array([1.0 for _ in range(8)])
        # Clamp values between these values
        self.max = max_
        self.min = min_
    
    def start_ros(self):
        print('start')
        self.rate = rospy.Rate(100)
        self.pub = rospy.Publisher('motor_commands', Int32MultiArray, queue_size=10)
        rospy.init_node('keyboards', anonymous=True)
    
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
            self.pub.publish(data=self.num_motors)
            self.rate.sleep()


trans_layer : Translation_Layer = Translation_Layer(1.0, 0.0)

if __name__ == '__main__':
    # keyboard.on_press_key('left arrow', lambda _: print('mkay'))
    keyboard.on_press_key('e', lambda _: trans_layer.yaw(1.0))
    keyboard.on_press_key('q', lambda _: trans_layer.yaw(-1.0))
    keyboard.on_press_key('a', lambda _: trans_layer.roll(1.0))
    keyboard.on_press_key('d', lambda _: trans_layer.roll(-1.0))
    keyboard.on_press_key('w', lambda _: trans_layer.pitch(1.0))
    keyboard.on_press_key('s', lambda _: trans_layer.pitch(-1.0))
    keyboard.on_press_key('space', lambda _: trans_layer.ascend_descend(1.0))
    keyboard.on_press_key('shift', lambda _: trans_layer.ascend_descend(-1.0))
    keyboard.on_press_key('up arrow', lambda _: trans_layer.forward_back(1.0))
    keyboard.on_press_key('down arrow', lambda _: trans_layer.forward_back(-1.0))

    trans_layer.start_ros()

    trans_layer.publisher()
    
    while True:
        continue