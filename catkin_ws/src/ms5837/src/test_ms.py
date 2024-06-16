import smbus
import time

_MS5837_ADDR = 0x76
_MS5837_RESET = 0x1E

for bus in range(0,11):
    try:
        _bus = smbus.SMBus(bus)
        time.sleep(0.2)
        _bus.write_byte(_MS5837_ADDR, _MS5837_RESET)
    except:
        print("Bus %d not available." % bus)