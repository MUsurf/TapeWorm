# This code was translated to account for the depreciation of 'SMBUS'
# This file is now using busio for i2c
# As of 6/7/24 1:15 PM this file is untested

import rospy
import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice
from time import sleep

# Models
MODEL_02BA = 0
MODEL_30BA = 1

# Oversampling options
OSR_256  = 0
OSR_512  = 1
OSR_1024 = 2
OSR_2048 = 3
OSR_4096 = 4
OSR_8192 = 5

# kg/m^3 convenience
DENSITY_FRESHWATER = 997
DENSITY_SALTWATER = 1029

# Conversion factors (from native unit, mbar)
UNITS_Pa     = 100.0
UNITS_hPa    = 1.0
UNITS_kPa    = 0.1
UNITS_mbar   = 1.0
UNITS_bar    = 0.001
UNITS_atm    = 0.000986923
UNITS_Torr   = 0.750062
UNITS_psi    = 0.014503773773022

# Valid units
UNITS_Centigrade = 1
UNITS_Farenheit  = 2
UNITS_Kelvin     = 3

class MS5837(object):
    # Registers
    _MS5837_ADDR             = 0x76  
    _MS5837_RESET            = 0x1E
    _MS5837_ADC_READ         = 0x00
    _MS5837_PROM_READ        = 0xA0
    _MS5837_CONVERT_D1_256   = 0x40
    _MS5837_CONVERT_D2_256   = 0x50
    # TO change bus, change line 87 in ms5837_ros.py. DO not change this file for bus changes!
    def __init__(self, model=MODEL_30BA, bus=1):
        self._model = model
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.device = I2CDevice(self.i2c, self._MS5837_ADDR)
        except Exception as e:
            rospy.logerr(f"Bus {bus} is not available.")
            rospy.logerr(f"Available busses are listed as /dev/i2c*")
            self.device = None
        
        self._fluidDensity = DENSITY_FRESHWATER
        self._pressure = 0
        self._temperature = 0
        self._D1 = 0
        self._D2 = 0
        
    def init(self):
        if self.device is None:
            "No bus!"
            return False
        
        with self.device as i2c:
            i2c.write(bytes([self._MS5837_RESET]))
        
        # Wait for reset to complete
        sleep(0.1)
        
        self._C = []
        
        # Read calibration values and CRC
        for i in range(7):
            with self.device as i2c:
                i2c.write(bytes([self._MS5837_PROM_READ + 2*i]))
                result = bytearray(2)
                i2c.readinto(result)
                c =  ((result[0] << 8) | result[1]) # Little-endian for word transfers
                self._C.append(c)
                        
        crc = (self._C[0] & 0xF000) >> 12
        if crc != self._crc4(self._C):
            rospy.loginfo("PROM read error, CRC failed!")
            return False
        
        return True
        
    def read(self, oversampling=OSR_8192):
        if self.device is None:
            rospy.logerr("No bus!")
            return False
        
        if oversampling < OSR_256 or oversampling > OSR_8192:
            rospy.logerr("Invalid oversampling option!")
            return False
        
        # Request D1 conversion (temperature)
        with self.device as i2c:
            i2c.write(bytes([self._MS5837_CONVERT_D1_256 + 2*oversampling]))
        
        # Maximum conversion time increases linearly with oversampling
        sleep(2.5e-6 * 2**(8+oversampling))
        
        d = bytearray(3)
        with self.device as i2c:
            i2c.write(bytes([self._MS5837_ADC_READ]))
            i2c.readinto(d)
        self._D1 = d[0] << 16 | d[1] << 8 | d[2]

        # Request D2 conversion (pressure)
        with self.device as i2c:
            i2c.write(bytes([self._MS5837_CONVERT_D2_256 + 2*oversampling]))
        
        # As above
        sleep(2.5e-6 * 2**(8+oversampling))
 
        with self.device as i2c:
            i2c.write(bytes([self._MS5837_ADC_READ]))
            i2c.readinto(d)
        self._D2 = d[0] << 16 | d[1] << 8 | d[2]

        # Calculate compensated pressure and temperature
        self._calculate()
        
        return True
    
    def setFluidDensity(self, density):
        self._fluidDensity = density
        
    # Pressure in requested units
    def pressure(self, conversion=UNITS_mbar):
        return self._pressure * conversion
        
    # Temperature in requested units
    def temperature(self, conversion=UNITS_Centigrade):
        degC = self._temperature / 100.0
        if conversion == UNITS_Farenheit:
            return (9.0/5.0)*degC + 32
        elif conversion == UNITS_Kelvin:
            return degC + 273
        return degC
        
    # Depth relative to MSL pressure in given fluid density
    def depth(self):
        return (self.pressure(UNITS_Pa)-101300)/(self._fluidDensity*9.80665)
    
    # Altitude relative to MSL pressure
    def altitude(self):
        return 0  
 
    # Cribbed from datasheet
    def _calculate(self):
        OFFi = 0
        SENSi = 0
        Ti = 0

        dT = self._D2-self._C[5]*256
        if self._model == MODEL_02BA:
            SENS = self._C[1]*65536+(self._C[3]*dT)/128
            OFF = self._C[2]*131072+(self._C[4]*dT)/64
            self._pressure = (self._D1*SENS/(2097152)-OFF)/(32768)
        else:
            SENS = self._C[1]*32768+(self._C[3]*dT)/256
            OFF = self._C[2]*65536+(self._C[4]*dT)/128
            self._pressure = (self._D1*SENS/(2097152)-OFF)/(8192)
        
        self._temperature = 2000+dT*self._C[6]/8388608

        # Second order compensation
        if self._model == MODEL_02BA:
            if (self._temperature/100) < 20: # Low temp
                Ti = (11*dT*dT)/(34359738368)
                OFFi = (31*(self._temperature-2000)*(self._temperature-2000))/8
                SENSi = (63*(self._temperature-2000)*(self._temperature-2000))/32
                
        else:
            if (self._temperature/100) < 20: # Low temp
                Ti = (3*dT*dT)/(8589934592)
                OFFi = (3*(self._temperature-2000)*(self._temperature-2000))/2
                SENSi = (5*(self._temperature-2000)*(self._temperature-2000))/8
                if (self._temperature/100) < -15: # Very low temp
                    OFFi = OFFi+7*(self._temperature+1500)*(self._temperature+1500)
                    SENSi = SENSi+4*(self._temperature+1500)*(self._temperature+1500)
            elif (self._temperature/100) >= 20: # High temp
                Ti = 2*(dT*dT)/(137438953472)
                OFFi = (1*(self._temperature-2000)*(self._temperature-2000))/16
                SENSi = 0
        
        OFF2 = OFF-OFFi
        SENS2 = SENS-SENSi
        
        if self._model == MODEL_02BA:
            self._temperature = (self._temperature-Ti)
            self._pressure = (((self._D1*SENS2)/2097152-OFF2)/32768)/100.0
        else:
            self._temperature = (self._temperature-Ti)
            self._pressure = (((self._D1*SENS2)/2097152-OFF2)/8192)/10.0   
        
    # Cribbed from datasheet
    def _crc4(self, n_prom):
        n_rem = 0
        
        n_prom[0] = ((n_prom[0]) & 0x0FFF)
        n_prom.append(0)
    
        for i in range(16):
            if i%2 == 1:
                n_rem ^= ((n_prom[i>>1]) & 0x00FF)
            else:
                n_rem ^= (n_prom[i>>1] >> 8)
                
            for n_bit in range(8,0,-1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = (n_rem << 1)

        n_rem = ((n_rem >> 12) & 0x000F)
        
        self.n_prom = n_prom
        self.n_rem = n_rem
    
        return n_rem ^ 0x00
    
class MS5837_30BA(MS5837):
    def __init__(self, bus=1):
        MS5837.__init__(self, MODEL_30BA, bus)
        
class MS5837_02BA(MS5837):
    def __init__(self, bus=1):
        MS5837.__init__(self, MODEL_02BA, bus)
