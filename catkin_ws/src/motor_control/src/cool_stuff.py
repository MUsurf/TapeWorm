class MotorHandler:
    def __init__(self, num_motors, step_value):
        self.previous_power = [0 for i in range(num_motors)]
        self.step_value = step_value
    def run_motors(self, powers):
        actual_powers = [0 for i in range(self.num_motors)]
        for i in range(self.num_motors):
            actual_powers[i] = 
        