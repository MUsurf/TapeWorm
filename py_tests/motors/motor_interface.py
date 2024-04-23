from motor_command import MotorCommand
from typing import List
import time


class MotorInterface():
    def __init__(self, channels: List[int], numMotors: int, offset: int, max_val: int, minor_time: float, step_size: int) -> None:
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
            for _ in range(10):
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

    # Percent drive
    # [right, left, forward, back, up, down]
    def direction_to_motor(self, directions):
        # up words facing 1-4
        # forward-left
        # Forward
        print("hell")

        # Setting up MotorCommand move this to motor interface for higher level access
local_channels: List[int] = [0, 1, 2, 3]
num_motors = 4

try:
    motor_commander = MotorCommand(local_channels, num_motors, step_size=1)

    motor_commander.arm_seq()
    print("done arming")
    time.sleep(1)

    # Driver
    # ! These are place holders for arbit values to hold
    high: List[int] = [25 for i in range(num_motors)]
    low: List[int] = [35 for i in range(num_motors)]

    value_running: List[int] = [0 for _ in range(num_motors)]
    value_to_set = 0
    while True:
        print(f"current Value {value_running}")
        for x in range(10):
            motor_commander.pinStep(value_running)
            time.sleep(.1)
        value_to_set: int = value_to_set + 1
        for index in range(num_motors):
            value_running[index] = value_to_set
        time.sleep(10)
except KeyboardInterrupt:
    motor_commander.clo_seq()
