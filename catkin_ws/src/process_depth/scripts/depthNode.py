#!/usr/bin/env python

'''
ROS
---

node: depth_node
-----

Publishes:
----------
        - depth_processed

Subscribes:
-----------
        - rov/ms5837_filtered

'''

# Begin imports
import rospy
# End imports


class SmoothFunction():
    def __init__(self) -> None:
        self.data: int = 0

    def smooth(self, data: int) -> int:
        return (data * 10 - 1 + 10 * 5) # for example only not doing anything
    
    def callback(self, message) -> None:
        self.data = self.smooth(message.data)

def commander():
    smoother = SmoothFunction()

    pub: rospy.Publisher = rospy.Publisher('depth_processed', int, queue_size=-1)
    rospy.init_node('depth_processed', anonymous=True)
    rate = rospy.Rate(.1) # 10hz

    while not rospy.is_shutdown():
        pub.publish(data=smoother.data)
        rospy.loginfo(smoother.data)
        rospy.Subscriber("rov/ms5837_filtered", int, smoother.callback)
        rospy.spin()
        # rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass