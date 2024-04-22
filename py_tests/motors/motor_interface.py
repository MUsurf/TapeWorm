from motor_command import MotorCommand
from typing import List
import time

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
