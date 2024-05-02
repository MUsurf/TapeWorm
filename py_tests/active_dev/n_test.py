import board
import busio

# Within the busio module you’ll use the busio.I2C class to create an interface to access the I2C bus
# Uknown why this doesn't work with SCL
i2c = busio.I2C(board.I2C0_SCL, board.I2C0_SDA)


# However before you make calls against the I2C bus you first need to take control,
# or ‘lock’, it to ensure your code has exclusive access to I2C.

# To lock the I2C bus you want to use a special loop syntax that waits for
# the busio.I2C.try_lockfunction to succeed:

while not i2c.try_lock():
    pass

# returns the adresses of all i2c devives in chain
address_returns = i2c.scan()

i2c.unlock()

for add in address_returns:
    print(hex(add))
