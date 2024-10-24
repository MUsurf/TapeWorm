
class MotorHandler:
    def __init__(self, num_motors, step_value):
        self.previous_powers = [0 for i in range(num_motors)]
        self.step_value = step_value
        self.num_motors = num_motors
    def run_motors(self, powers):
        actual_powers = [0 for i in range(self.num_motors)]
        for i in range(self.num_motors):
            direction = 1 if powers[i] > self.previous_powers[i] else -1
            actual_powers[i] = powers[i] if abs(powers[i]-self.previous_powers[i]) <= self.step_value else self.previous_powers[i] + self.step_value * direction
        #Pretend this does stuff :)
        self.previous_powers = actual_powers
        
        return actual_powers
        
e = MotorHandler(2, 2)
print(e.run_motors([8,8]))
print(e.run_motors([8,8]))
print(e.run_motors([8,8]))
print(e.run_motors([8,8]))
print(e.run_motors([8,8]))
print(e.run_motors([8,8]))
print(e.run_motors([8,8]))
print(e.run_motors([8,8]))

        