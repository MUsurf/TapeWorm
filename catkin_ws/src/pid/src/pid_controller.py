#!/usr/bin/env python3

'''
ROS 
node: pid_controller
Publishes:
Subscribes:


Maintainer: Henry Bloch
'''

# Begin Imports
import time
import rospy
from pid_driver import Pid_object

# End Imports

class PID_controller():
    def __init__(self, ttw: int) -> None:
        """This does not setup any pid controllers that is to be done at a later step

        Parameters
        ----------
        ttw : int
            time to wait until next step
        """        

        self.pid_controllers: list[Pid_object] = []

        self.current_values: list[float] = []

        self.time_counter = time.time()
        self.wait_time: int = ttw

    def init_pid(self, error: float, integral: float, prop: float, derivative: float, bias: float, set_point: float, current_value: float) -> int:
        """This starts a pid controller with provided settings

        This starts a pid instance with the settings provided and returns the id for future refrence

        Parameters
        ----------
        error : float
            The amount of distance between 'set_point' and 'current_value'
        integral : float
            The integral component
        prop : float
            The proportional component
        derivative : float
            The derivitive component
        bias : float
            The bias drive
        set_point : float
            The target value of the PID
        current_value : float
            The value to start the system

        Returns
        -------
        int
            Id of pid added
        """        

        self.pid_controllers.append(Pid_object(error, integral, prop, derivative, bias, set_point, current_value))
        pid_id = len(self.pid_controllers)
        return pid_id

    def __pid_step(self, pid_object: Pid_object, current_value: float) -> None:
        pid_object.Update(current_value)

    def step_all(self, current_values: "list[float]") -> None:
        """Update all PID objects

        WARNING probably should not be used out of class

        Parameters
        ----------
        current_values : list[float]
            values for the PIDs
        """
        assert (len(current_values) == len(self.pid_controllers))

        for counter in range(len(current_values)):
            self.__pid_step(self.pid_controllers[counter], current_values[counter])

    def step_one(self, current_value: float, pid_id: int) -> None:
        """Update one PID object

        WARNING probably should not be used out of class

        Parameters
        ----------
        current_value : float
            value for PID being updated
        pid_id : int
            pid to update
        """
        self.__pid_step(self.pid_controllers[pid_id], current_value)

    def ros_update_reading(self, data, args) -> None:
        """Function to give ros for callback

        Parameters
        ----------
        reading : float
            value for next run 
        index : int
            _description_
        """
        self.current_values[args[0]] = data.data
    
    def spin(self) -> None:
        """ros runner function"""

        while not rospy.is_shutdown():
            current_time = time.time()
            if (current_time > (self.wait_time + self.time_counter)):
                self.time_counter = current_time
                self.step_all(self.current_values)
                rospy.loginfo(self.current_values)



if __name__ == '__main__':
    motor_pid = [
        # error
        0.0,
        # integral
        0.0,
        # prop
        0.0,
        # derivative
        0.0,
        # bias
        0.0,
        # set_point
        0.0,
        # current_value
        0.0
    ]

    # Start up stuff
    pid_controller = PID_controller(1)

    # Start a new pid controller for each thing needing to be controlled
    motor_pid = pid_controller.init_pid(motor_pid[0], motor_pid[1], motor_pid[2], motor_pid[3], motor_pid[4], motor_pid[5], motor_pid[6])


    # ros stuff
    rospy.init_node('pid_controller', anonymous=True)

    # Add a subscriber for each pid
    rospy.Subscriber("example", 'example', pid_controller.ros_update_reading, (motor_pid))
