#!/usr/bin/env python3

import ms5837_driver
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from ms5837.msg import ms5837_data
from nav_msgs.msg import Odometry
import numpy as np


# Choose seawater or freshwater depth calibration using ros param
# freshwater = 997 kg/m^3
# seawater = 1029 kg/m^3

# def calculate(array):
#     mean = 0
#     #total = 0
#     if(len(array)>50):
#         array.pop(0)
#     mean = sum(array)
#     mean /= len(array)
#     #for point in array:
    #    total += (point - mean) * (point - mean)
    #total /= len(array)
    #print(mean)
    #return mean

class KalmanFilter:

    def __init__(self, max_diff=100, process_noise_matrix=np.array([[0.001, 0.00000001, 0.00000001], [0.00000001, 0.0005, 0.00000001], [0.00000001, 0.00000001, 0.0001]])):
        # initialize the filter with random values
        self.KG = np.ones((3, 3))  # Kalman gain
        self.est = np.zeros((3, 3))  # last estimate (x,v,a)
        self.est_error = np.ones((3, 3))  # Error of the filter estimate
        self.process_noise_matrix = process_noise_matrix

        self.last_time = time.time()  # for calculating dt

        # for sanity check on sensor value
        self.last_x_m = None
        self.max_diff = max_diff  # the maximum error between two sensor readings for value to be thrown out

    def update(self, x_m, v_m, a_m, x_e, v_e, a_e):
        # pass in measure position, velocity, and acceleration along with error for each
        # check to make sure sensor value has not had catastrophic problem
        if self.last_x_m is None:
            self.last_x_m = x_m
        if abs(x_m - self.last_x_m) > self.max_diff:
            rospy.logerr("Sensor value error: change in measurement too large for one time step")
        else:

            # if value is all good then continue to run time step step
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            # create our new estimate based on the model
            self.est = np.matmul(np.array([[1, dt, 1 / 2.0 * dt ** 2], [0, 1, dt], [0, 0, 1]]), self.est) * [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            rospy.logdebug(self.est)
            self.est_error = self.est_error * [[1, 0, 0], [0, 1, 0], [0, 0, 1]] + self.process_noise_matrix  # prevent error from going to zero
            rospy.logdebug(self.est_error)

            # update estimate with new sensor values
            self.KG = self.est_error / (self.est_error + np.array([[x_e, 0, 0], [0, v_e, 0], [0, 0, a_e]])) * [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            self.est = self.est + np.matmul(self.KG, np.array([[x_m, 0, 0], [0, v_m, 0], [0, 0, a_m]]) - self.est)
            self.est_error = np.matmul((np.identity(3) - self.KG), self.est_error)

        self.last_x_m = x_m
        return self.est, self.est_error


if __name__ == '__main__':
    try:
        # set up ros stuff
        data = []
        rospy.init_node('ms5837_node')
        fluid_density = rospy.get_param('~fluid_density', '1000')
        publish_odom = rospy.get_param('~publish_odom', True)
        publish_pose = rospy.get_param('~publish_pose', False)
        use_kalman_filter = rospy.get_param('~use_kalman_filter', False)
        depth_variance = rospy.get_param('~depth_variance', 0.001)
        tf_frame = rospy.get_param("~tf_frame", "depth_sensor_link")

        pub = rospy.Publisher('rov/ms5837', ms5837_data, queue_size=1)
        rate = rospy.Rate(20)  # 100Hz data read
        sensor = ms5837_driver.MS5837_02BA(bus=10)  # Default I2C bus is 1 (Raspberry Pi 3)
        # sensor = ms5837.MS5837_02BA()

        sensor.setFluidDensity(int(fluid_density))
        time.sleep(1)
        # sensor.init must run immediately after installation of ms5837 object
        sensor.init()

        odom_pub = None
        pose_pub = None
        filter = None
        if publish_odom:
            odom_pub = rospy.Publisher("/rov/depth_odom", Odometry, queue_size=1)
        if publish_pose:
            pose_pub = rospy.Publisher("/rov/depth_pose", PoseWithCovarianceStamped, queue_size=1)
        if use_kalman_filter:
            filter = KalmanFilter(100)

        last_depth_m = 0  # the last sensor value for computing the velocity
        last_velocity_m = 0  # last velocity for computing acceleration
        last_time = time.time()  # time of last read for computing velocity

        while not rospy.is_shutdown():
            msg = ms5837_data()

            sensor.read(oversampling=0)  # maximum read rate of ~90Hz

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # measured values for depth, velocity, acceleration
            velocity_m = (sensor.depth() - last_depth_m) / dt
            last_depth_m = sensor.depth()
            acceleration_m = (velocity_m - last_velocity_m) / dt
            last_velocity_m = velocity_m

            if use_kalman_filter:
                state, variance = filter.update(sensor.depth(), velocity_m, acceleration_m,
                                                depth_variance, depth_variance, depth_variance)
                depth = state[0, 0]
                velocity = state[1, 1]
                variance = [variance[0, 0], variance[1, 1]]  # position and velocity variance
            else:
                depth = sensor.depth()

                velocity = velocity_m
                variance = [depth_variance, depth_variance]

            msg.tempC = sensor.temperature(ms5837_driver.UNITS_Centigrade)
            msg.tempF = sensor.temperature(ms5837_driver.UNITS_Farenheit)
            msg.depth = sensor.depth()
            # msg.altitudeM = sensor.altitude() # causes error in driver

            # update message headers
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'depth_data'

            pub.publish(msg)

            if publish_odom:
                msg = Odometry()
                msg.header.frame_id = tf_frame
                msg.header.stamp = rospy.Time.now()
                msg.pose.pose.position.z = float(depth)
                data.append(depth)
                # if(len(data) >= 50):
                #     msg.pose.pose.position.z = float(calculate(data))
                time_now = time.time()
                msg.twist.twist.linear.z = float(velocity)
                last_time = time_now
                msg.pose.covariance[14] = variance[0]
                msg.twist.covariance[14] = variance[1]
                odom_pub.publish(msg)

            if publish_pose:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = tf_frame
                msg.header.stamp = rospy.Time.now()
                msg.pose.pose.position.z = 1#sensor.depth() #(float(depth) - 193)
                msg.pose.covariance[14] = variance[0]
                pose_pub.publish(msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
