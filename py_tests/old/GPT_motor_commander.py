'''
This is entirely created through the SurfGPT AI

This is the text response it provided:

    Based on the code you've provided, it appears your setup includes a PCA9685 module for PWM control, likely connected to the Jetson Orin Nano via I2C. This setup controls 8 motors, with the PCA9685 handling the PWM signals to adjust motor speeds.

    To adapt your script to receive desired movement values (forward, back, left, right, up, down, pitch, yaw, roll) from a different ROS node and use these values to control the motors, we'll assume each direction maps to changes in motor speeds. The desired values will range from -100 to 100, indicating the direction and intensity of the movement.

    Let's create a script that subscribes to a topic /desired_movement publishing a Int32MultiArray message with 9 elements representing the desired values for forward, back, left, right, up, down, pitch, yaw, and roll, respectively. The script will calculate the necessary PWM settings for each motor based on these inputs.

    <CODE>

    Key Points to Adapt in the Real Implementation:

        Motor Speed Calculation: The set_motor_speed method simplifies the speed-to-PWM mapping. You'll need a nuanced approach, likely involving more complex math to handle directions and rotations accurately.
        Movement Logic: The current implementation naively assigns the forward value to all motors. You'll need to design logic that translates the desired movement directions (forward, back, left, right, up, down, pitch, yaw, roll) into coordinated motor speeds.
        Safety and Limits: Ensure that the speed values are safely within your motors' and the PCA9685's operational ranges to prevent damage.

    Remember to test any new code in a controlled environment to ensure safety and prevent hardware damage.
'''

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import busio
from board import SCL, SDA
import adafruit_pca9685 as PCA9685

# Initialize ROS node
rospy.init_node("motor_controller")

# Set up I2C and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685.PCA9685(i2c, address=0x40)
pca.frequency = 280  # Hz for the PCA9685 PWM controller

class MotorController:
    def __init__(self):
        # Initialize motor channels
        self.motors = [pca.channels[i] for i in range(8)]

    def microSec_to_duty(self, microSec):
        '''Convert microseconds to duty cycle value for PCA9685'''
        samp_time = (1/pca.frequency) * 1000 * 1000  # Convert to Microseconds
        return int((65536 * microSec) / samp_time)

    def set_motor_speed(self, motor_id, speed):
        '''Set the speed of a single motor'''
        # Example speed conversion, needs to be adapted based on actual motor characteristics
        pwm_value = self.microSec_to_duty(1500 + (speed * 10))  # Mapping -100...100 to appropriate PWM range
        self.motors[motor_id].duty_cycle = pwm_value

    def handle_movement_command(self, msg):
        '''Handle incoming movement command'''
        # Example: Simple mapping of the forward value to all motors for demonstration
        # You'll need to create a more complex logic here to handle all movements appropriately
        for i, value in enumerate(msg.data):
            self.set_motor_speed(i, value)  # This is a simplification

# Create motor controller instance
motor_controller = MotorController()

# Setup subscriber
def movement_command_callback(msg):
    '''Callback function for movement command messages'''
    rospy.loginfo(f"Received movement command: {msg.data}")
    motor_controller.handle_movement_command(msg)

movement_subscriber = rospy.Subscriber("/desired_movement", Int32MultiArray, movement_command_callback)

# ROS spin to keep the script for exiting
if __name__ == "__main__":
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down motor controller")
