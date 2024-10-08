import time
import math
#import rospy
#from std_msgs.msg import Int32MultiArray
from typing import List
from movement_functions import sideways, up, forward, right

num_motors = 8

num_of_xcalls = 3

num_of_ycalls = 2

listx1 = sideways(2, 54)
listx2 = right(20, True)
listx3 = forward(30)

listy1 = up(50)
listy2 = up(-24)

allX = [0, 0, 0, 0, 0, 0, 0, 0]
allY = [0, 0, 0, 0, 0, 0, 0, 0]

movement_direction = [0, 0, 0, 0, 0, 0, 0, 0]

for x in range(0, 8):
    allX[x] = (listx1[x] + listx2[x] + listx3[x])/num_of_xcalls

for x in range(0, 8):
    allY[x] = (listy1[x] + listy2[x])/num_of_ycalls

for x in range(0, 8):
    movement_direction[x] = allX[x] + allY[x]

print(movement_direction)

temp_movement_direction = [0, 0, 0, 0, 0, 0, 0, 0]

range_beginning = 50

#logrithmic
#for x in range(1, range_beginning):
#    for y in range(0, 8):
#        temp_movement_direction[y] = movement_direction[y]/(range_beginning-x)
#    print(temp_movement_direction)
#print("asdhjbashjdbas")

#linear
for x in range(1, range_beginning + 1):
    for y in range(0, 8):
        temp_movement_direction[y] = movement_direction[y]*(x/range_beginning)
    print(temp_movement_direction)
    time.sleep(0.1)
print("asdhjbashjdbas")
print(movement_direction)

#rate = rospy.Rate(.1) # 10hz

#for x in range(1, range_beginning + 1):
#    for y in range(0, 8):
#        temp_movement_direction[y] = movement_direction[y]*(x/range_beginning)
#    motor_powers = temp_movement_direction
    #rospy.loginfo(motor_powers)
    #pub.publish(data=motor_powers)
#    rate.sleep()