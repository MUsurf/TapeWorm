
class MotorHandler:
    def __init__(self, num_motors, step_value):
        self.previous_powers = [0 for i in range(num_motors)]
        self.step_value = step_value
        self.num_motors = num_motors
    def run_motors(self, powers):
        actual_powers = [0 for i in range(self.num_motors)]
        for i in range(self.num_motors):
            direction = 1 if powers[i] > self.previous_powers[i] else -1
            should_step = not abs(powers[i]-self.previous_powers[i]) <= self.step_value
            actual_powers[i] = powers[i] if should_step else self.previous_powers[i] + self.step_value * direction
        
        #Pretend this does stuff :)
        
        self.previous_powers = actual_powers
        
        return actual_powers

    #Assumes 8 motor setup
    def run_motors_by_direction(self, power, direction, up_down_power):
        #Direction is true if forward backwards else left right
        
        #Positive is CW, Negative is CCW
        if self.num_motors == 8:
            if direction:
                self.run_motors(power, power, -power, -power, up_down_power, up_down_power, up_down_power, up_down_power)
            else: 
                self.run_motors(power, -power, power, -power, up_down_power, up_down_power, up_down_power, up_down_power)
        else:
            print("Not 8 motor drive")
        