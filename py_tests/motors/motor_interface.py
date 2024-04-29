from motor_command import MotorCommand
from typing import List
import time


class MotorInterface():
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

    def arm_seq(self) -> None:
        """Current method of arming all motors may change with calibration

        Notes
        -----
            Pin target values are hardcoded they should not need to be changed often.
        """

        target_speeds: List[List[int]] = [
            [0 for _ in range(self.numMotors)],
            [40 for _ in range(self.numMotors)],
            [50 for _ in range(self.numMotors)],
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
            This function is not implemented yet and only contains the translation from percent drive of commands to duty cycle
        """

        drive_in_duty: list[int] = []
        for p_direction in directions:
            drive_in_duty.append(self.__percent_to_duty(p_direction))
        return (drive_in_duty)


try:
    local_channels: List[int] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    num_motors: int = len(local_channels)
    motor_caller = MotorInterface(local_channels, num_motors, 0, 100, .1, 5)

    high: List[int] = [50 for i in range(num_motors)]
    low: List[int] = [70 for i in range(num_motors)]

    motor_caller.arm_seq()

    while True:
        motor_caller.calling_function(low)
        time.sleep(5)
        motor_caller.calling_function(high)
        time.sleep(5)
except KeyboardInterrupt:
    motor_caller.clo_seq()
