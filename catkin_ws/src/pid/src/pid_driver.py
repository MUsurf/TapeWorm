"""
This assumes that each step is done in a constant time. This effects many things but may be clearly seen when going through tunning
"""

# Begin imports
import time

# End imports

#! Implement time delta resolving on the pid driver side
class Pid_object():
    def __init__(self, error: float, integral: float, prop: float, derivative: float, bias: float, set_point: float, current_value: float):
        self.error_prior: float = error
        self.integral_prior: float = integral
        self.coef_Prop: float = prop
        self.coef_Inte: float = integral
        self.coef_Deriv: float = derivative
        self.bias: float = bias
        self.set_point: float = set_point 
        self.current_value: float = current_value

        self.prev_time: float = time.time()
    
    def __pid_Step(self, time_delta: float) -> float:
        error : float = self.set_point - self.current_value
        integral : float = self.integral_prior + (error * time_delta)
        derivative : float = (error - self.error_prior) / time_delta
        output : float = self.coef_Prop * error + self.coef_Inte * integral + self.coef_Deriv * derivative + self.bias
        self.error_prior = error
        self.integral_prior = integral
        
        return output
    
    def Update(self, current_value: float) -> float :
        """Method to be called for each pid step"""

        # Resolve time delta between calls
        execution_time = time.time()
        time_delta = abs(execution_time - self.prev_time)

        self.current_value += current_value
        self.current_value = self.__pid_Step(time_delta)

        self.prev_time = execution_time
        return self.current_value
    
    def Update_setpoint(self, set_point: float) -> None:
        """Method to be called when updating target value

        Parameters
        ----------
        set_point : float
            target value for pid
        """

        self.set_point = set_point