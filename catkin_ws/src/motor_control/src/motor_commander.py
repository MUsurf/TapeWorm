
# Begin typing imports
from typing import List
# End typing imports

# Begin imports
import busio
from board import SCL, SDA
import adafruit_pca9685 as PCA9685
# End imports

# BEGIN SETUP
i2c = busio.I2C(SCL, SDA)
pca = PCA9685.PCA9685(i2c, address=0x40)
pca.frequency = 280  # Hz
# END SETUP


class MotorCommand():
    def __init__(self, 
        local_channels: List[int], 
        num_motors: int, 
        step_size: int=5, 
        minor_time: float=.1) -> None:
        """_summary_

        _extended_summary_

        Parameters
        ----------
        local_channels : List[int]
            List of channels to be using from i2c splitter
        num_motors : int
            Number of motors equal to len of 'local_channels'
        step_size : int, optional
            amount to move motors by out of 100, by default 5
        minor_time : float, optional
            time between steps for same target, by default .1
        """

        # info Number of motors being managed
        self.motorNum: int = num_motors

        # info list can only contain -1, 0, 1
        self.motor_direction: List[int] = [1 for _ in range(num_motors)]

        # info How much to move the motors at each minor step
        self.step_size: int = step_size

        # info Set how much time to wait on arming
        self.minor_step: float = minor_time

        # info Needed to save pin states to let outside program manage interupts when driving motors
        # info As this lets us step between power levels using duty cycle 0-100
        self.pinStates: List[int] = [0 for _ in range(num_motors)]

        # info This creates an array of channels to change
        # info This is done so number of motors can be changed on the fly
        self.motors: List[PCA9685.PWMChannel] = [
            pca.channels[channel] for channel in local_channels]

    def __microSec_to_duty(self, microSec: int) -> int:
        """Convert Microsecond pulses to duty cycle

        Convert Microsecond length pulses that have been aligned with the operating requirments of the interface to duty cycle of the current PWM frequency

        Parameters
        ----------
            microSec : int
                Must be int from 0-100 'microSec'

        Returns
        -------
            int
                int from 65536-0

        Notes
        -----
        'microsec' range comes from pca chips desired control frequency
        
        """

        samp_time: float = (1/pca.frequency) * 1000 * \
            1000  # Convert to Micro Sec
        duty_cycle = int((65536 * microSec)/(samp_time))
        return duty_cycle

    def set_motor_speed(self, motor_idex: int, speed: int) -> None:
        '''Set the speed of a single motor'''

        pwm_value: int = self.__microSec_to_duty(1000 + (speed * 10))
        self.motors[motor_idex].duty_cycle = pwm_value

    def pinStep(self, targets: List[int]) -> None:
        """Move pin towards target supplied

        Generates intermediate values and then steps pin from current state toward target.

        Parameters
        ----------
            targets : List[int]
                list of targets for motors (order matters).

        Notes
        -----
            Should be used with an outside function to handle interupts
        """

        directions: List[int] = self.__targetDistance(targets)
        for index in range(len(directions)):
            if (directions[index] == 0):
                continue
            self.pinStates[index] += directions[index] * self.step_size
            print(self.pinStates[index])
        # ? Sets every pin even if it is already opperating at that speed
        # ? Don't think this is an area that needs to be improved but is an easy target
        self.__set_motors(self.pinStates)

    def __targetDistance(self, targets: List[int]) -> List[int]:
        """Figures out wich direction to step pins

        Notes
        -----
            Reworked

        """

        values: List[int] = [target - pinState for target, pinState in zip(targets, self.pinStates)]
        conversions: List[int] = [int(value / abs(value)) if value != 0 else 0 for value in values] # int cast should only be nessisary for linter
        return (conversions)

    def __set_motors(self, speeds: List[int]) -> None:
        """Sets pins to values given by speed position"""

        for index in range(self.motorNum):
            self.set_motor_speed(index, speeds[index])