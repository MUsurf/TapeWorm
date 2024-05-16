import numpy as np
import random
import matplotlib.pyplot as plt

class KalmanFilter:

    def __init__(self, max_diff=100, process_noise_matrix=np.array([[0.001, 0.00000001, 0.00000001], [0.00000001, 0.0005, 0.00000001], [0.00000001, 0.00000001, 0.0001]])):
        # initialize the filter with random values
        self.KG = np.ones((3, 3))  # Kalman gain
        self.est = np.zeros((3, 3))  # last estimate (x,v,a)
        self.est_error = np.ones((3, 3))  # Error of the filter estimate
        self.process_noise_matrix = process_noise_matrix

        self.last_time = 0  # for calculating dt

        # for sanity check on sensor value
        self.last_x_m = None
        self.max_diff = max_diff  # the maximum error between two sensor readings for value to be thrown out

    def update(self, x_m, v_m, a_m, x_e, v_e, a_e):
        # pass in measure position, velocity, and acceleration along with error for each
        # check to make sure sensor value has not had catastrophic problem
        if self.last_x_m is None:
            self.last_x_m = x_m
        if abs(x_m - self.last_x_m) > self.max_diff:
            print("Sensor value error: change in measurement too large for one time step")
        else:

            # if value is all good then continue to run time step step
            current_time = self.last_time + 1
            dt = current_time - self.last_time
            self.last_time = current_time

            # create our new estimate based on the model
            self.est = np.matmul(np.array([[1, dt, 1 / 2.0 * dt ** 2], [0, 1, dt], [0, 0, 1]]), self.est) * [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            self.est_error = self.est_error * [[1, 0, 0], [0, 1, 0], [0, 0, 1]] + self.process_noise_matrix  # prevent error from going to zero

            # update estimate with new sensor values
            self.KG = self.est_error / (self.est_error + np.array([[x_e, 0, 0], [0, v_e, 0], [0, 0, a_e]]))  * [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            self.est = self.est + np.matmul(self.KG, np.array([[x_m, 0, 0], [0, v_m, 0], [0, 0, a_m]]) - self.est)
            self.est_error = np.matmul((np.identity(3) - self.KG), self.est_error)

            print(self.est)
            print(self.est_error)


        self.last_x_m = x_m
        return self.est, self.est_error


####  Begin testing the kalman filter with a plant sim with know state

class PlantSim:
    def __init__(self):
        self.x = 0  # not constant
        self.v = 0.1  # not constant
        self.a = 0.01  # constant

    def update(self, dt=1.0):
        self.x = self.x + self.v * dt + 1 / 2 * self.a * dt ** 2
        self.v = self.v + self.a * dt
        self.a = random.gauss(self.a, 0.01)
        return [self.x, self.v, self.a]


filter = KalmanFilter(max_diff=1000)
sim = PlantSim()
error = 0.01

input = ''
filter_values = []
measured_values = []
measured_acceleration = []
filtered_acceleration = []
measured_velocity = []
filtered_velocity = []

for i in range(200):
    measured_state = sim.update()
    print(measured_state)
    filter.update(measured_state[0], measured_state[1], measured_state[2], error, error, error)
    filter_values.append(filter.est[0, 0])
    filtered_acceleration.append(filter.est[2, 2]*1000)
    filtered_velocity.append(filter.est[1, 1]*100)
    measured_values.append(measured_state[0])
    measured_acceleration.append(measured_state[2]*1000)
    measured_velocity.append(measured_state[1]*100)

    print("")

plt.plot(filter_values, color='blue')
plt.plot(measured_values, color='orange')
plt.plot(measured_acceleration, color='red')
plt.plot(filtered_acceleration, color='green')
plt.plot(measured_velocity, color='skyblue')
plt.plot(filtered_velocity, color='lightgreen')
plt.show()
