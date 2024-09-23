# Begin typing imports
from typing import List
# End typing imports

# Begin imports
from motor_commander import MotorCommand
import time
# End imports

class MotorInterface():
    """Handles direct control of motors
    
        Should be given an array of ints 
    """

    def __init__(self, channels: List[int], numMotors: int, offset: int, max_val: int, minor_time: float, step_size: int, steps_used=10) -> None:
        # info Number of motors
        self.numMotors: int = numMotors
        # info This is the amount of time between steps
        self.minor_time: float = minor_time
        # info This is how to set the min value
        self.offset: int = offset
        # info This is needed as this interface will take percent and scale to output used
        self.max_val: int = max_val
        # info max steps to go from one extreme to the other
        self.max_steps_needed: int = int(self.max_val / step_size)
        # info This is the amount of steps used assuming motors don't need to reach value
        self.steps_used: int = steps_used
        # info This is the instance of motorcommand that will be used
        self.motor_commander = MotorCommand(
            channels, self.numMotors, step_size, self.minor_time)
        # info This is the latest command recieved from ros if ros fails to deliver a new value before next execution then the same values are used
        self.last_directions: List[int] = []

    def arm_seq(self) -> None:
        """Current method of arming all motors may change with calibration

        Notes
        -----
            Pin target values are hardcoded they should not need to be changed often.
        """

        target_speeds: List[List[int]] = [
            [0 for _ in range(self.numMotors)],
            [20 for _ in range(self.numMotors)],
            [30 for _ in range(self.numMotors)],
            [10 for _ in range(self.numMotors)]
        ]

        for targets in target_speeds:
            for _ in range(self.max_steps_needed):
                self.motor_commander.pinStep(targets)
                time.sleep(self.minor_time)

    def clo_seq(self) -> None:
        """Cleans up motors and is responsible for bringing them all back to zero"""

        num_runs: int = self.max_steps_needed + 1
        targets: List[int] = [0 for _ in range(self.numMotors)]
        for _ in range(num_runs):
            self.motor_commander.pinStep(targets)
            time.sleep(self.minor_time)

    def __percent_to_duty(self, percent: int) -> int:
        range: int = abs(self.max_val - self.offset)
        duty: int = int(((percent / 100) * range) + self.offset)
        return duty

    def calling_function(self, directions) -> None:
        """Used to step motors to each target given"""

        duty_directions: List[int] = self.direction_to_motor(directions)
        for _ in range(self.steps_used):
            self.motor_commander.pinStep(duty_directions)
            time.sleep(self.minor_time)

    def direction_to_motor(self, directions) -> List[int]:
        """This function will have some of the direction to motor commands

        Notes
        -----
            directions will be in the form of a list of floats
                ['x': -1-1, 'y':-1-1, 'z':-1-1, 'pitch':-1-1, 'yaw':-1-1, 'roll':-1-1]

            This function is not implemented yet and only contains the translation from percent drive of commands to duty cycle
        """
        # * if you wish to go to the negative end of this axis the mangintude must also be supplied as a negative
        # info For multiple instructions to be followed at once the results post array must be added together
        # info This assumes that all motors are number 1-4 5-8 left to right and horizontal then vertical
        # ~ This could be used to balance out an under preforming motor
        motor_to_directions = [
            [1, 1, -1, -1, 0, 0, 0, 0], # 'x-axis'
            [1, -1, 1, -1, 0, 0, 0, 0], # 'y-axis'
            [0, 0, 0, 0, 1, 1, 1, 1], # 'z-axis' 
            [0, 0, 0, 0, 1, 1, -1, -1], # 'pitch' 
            [1, -1, -1, 1, 0, 0, 0, 0], # 'yaw' 
            [0, 0, 0, 0, 1, -1, 1, -1], # 'roll'
            # Add depth control
        ]

        drive_in_duty = [0 for _ in range(self.numMotors)]

        # ! Not working need to diagnose later
        # for index in range(len(directions)):
        #     for second_index in  range(len(motor_to_directions[0])):
        #         drive_in_duty[second_index] += directions[index] * motor_to_directions[index][second_index]
        # ~ Temp fix for above
        drive_in_duty = directions

        drive_to_duty = [self.__percent_to_duty(duty) for duty in drive_in_duty]

        # for p_direction in directions:
        #     drive_in_duty.append(self.__percent_to_duty(p_direction))
        return (drive_to_duty)
    

    def callback(self, message_rec):
        """Function to subscribe to driver with ros
        
        This is set up in a way to allow ros and the motor controls to function on a async basis. This will make sure nothing locks up
        and that pid gets very low latency feedback (not really but kinda).
        """

        print("Data received is: " + str(message_rec.data))
        self.last_directions = message_rec.data
        self.calling_function(self.last_directions)