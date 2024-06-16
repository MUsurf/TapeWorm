#!/usr/bin/env python
import rospy
import curses
from std_msgs.msg import String


class directionalOut():
    def __init__(self, x=0, y=0, z=0, pitch=0, yaw=0, roll=0):
        """
        Initialize directional outputs to zero
        All axis are relative to the sub and not the environment (for now)
        :param x: Movement along the X-axis (forward/backward)
        :param y: Movement along the Y-axis (left/right)
        :param z: Movement along the Z-axis (up/down)
        :param p: Rotational movement around the Y-axis (Pitch)
        :param y: Rotational movement around the Z-axis (Yaw)
        :param r: Rotational movement around the X-axis (Roll)
        """
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

    def updateMovement(self, vector):
        """
        Update the translational vector pamameters based on a 3D vector
        :param vector: A tuple or list of three elements (x, y, z)
        """

        self.x = vector[0]
        self.y = vector[1]
        self.z = vector[2]

    def updateRotation(self, vector):
        """
        Update the rotational vector parameters based on a 3D vector
        :param vector: a tuple or list of three elements (pitch, yaw, roll)
        """

        self.pitch = vector[0]
        self.yaw = vector[1]
        self.roll = vector[2]

    def __str__(self):
        """
        Simply formats all six parameters into an easy to parse string. Used to publish to a topic.
        """

        return f"{self.x} {self.y} {self.z} {self.pitch} {self.yaw} {self.roll}"


def read_key_input(screen):
    # Set the curses environment
    curses.noecho()
    curses.cbreak()
    screen.keypad(True)

    # Define the keymap mapped to WASD
    key_map = {
        ord('w'): (1, 0, 0),   # Increase X
        ord('s'): (-1, 0, 0),  # Decrease X
        ord('a'): (0, 1, 0),   # Increase Y
        ord('d'): (0, -1, 0),  # Decrease Y
        ord('q'): (0, 0, 1),   # Increase Z
        ord('e'): (0, 0, -1)   # Decrease Z
    }

    while True:
        char = screen.getch()
        if char == ord(' '):
            break
        if char in key_map:
            return key_map[char]
    return None


def main():
    # Create a publisher object that we can use to push messages of type "String" on the 'motor_commands' topic
    # This "String" can be changed to any object we want
    pub = rospy.Publisher('motor_commands', String, queue_size=10)

    rospy.loginfo("keyboard input has started")

    # This line Initializes the ROS node named "motor_controller". the anonymous=True parameter ensures that you can run multiples of the same node
    rospy.init_node('motor_controller', anonymous=True)

    # This sets the rate of teh object to 10Hz allowing you to use 'rate.sleep()' in the loop
    rate = rospy.Rate(10)

    # Define the "vector" object
    directions = directionalOut()

    # the main running loop of the ros node
    def mainloop(screen):
        while not rospy.is_shutdown():
            movement = read_key_input(screen)
            if movement is None:
                break

            # Update movement based on keyboard Input
            directions.updateMovement(movement)
            rate.sleep()

    # runs the mainloop in a curses wrapper
    curses.wrapper(mainloop)


# Start the main thread and handle the interrupt
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
