import filterpy
import pandas as pd
import time


def integrate_acceleration(acc_data, time_intervals):
    # Initialize variables
    velocity = [0.0, 0.0, 0.0]  # Initial velocity in x, y, z
    position = [0.0, 0.0, 0.0]  # Initial position in x, y, z

    # Integrate acceleration to obtain velocity
    for acc, dt in zip(acc_data, time_intervals):
        velocity[0] += acc[0] * dt
        velocity[1] += acc[1] * dt
        velocity[2] += acc[2] * dt

    # Integrate velocity to obtain position
    for vel, dt in zip(velocity, time_intervals):
        position[0] += vel[0] * dt
        position[1] += vel[1] * dt
        position[2] += vel[2] * dt

    return position


# Example usage
acceleration_data = [(1.0, 0.5, 0.2), (1.2, 0.6, 0.3),
                     (1.5, 0.7, 0.4)]  # Acceleration data in x, y, z
# Time intervals for each acceleration measurement
time_intervals = [1.0, 1.0, 1.0]

position = integrate_acceleration(acceleration_data, time_intervals)
print("Final position:", position)
