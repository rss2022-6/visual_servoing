#!/usr/bin/env python

import rospy
import numpy as np
from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped


class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
                         self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic")  # set in launch file; different for simulator vs racecar

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
                                         AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
                                         ParkingError, queue_size=10)

        # init previous values and relative position vals
        self.relative_x, self.relative_y = 0, 0
        self.prev_dist, self.prev_angle, self.prev_time, self.prev_speed = 0, 0, 0, 0
        self.prev_cmd = AckermannDriveStamped()
        self.int_ang_e = 0  # init integral

        # initialize parameters to the values below
        self.param_list = ['kps', 'kds', 'kpa', 'kda', 'kia', 'angle_coeff']
        init_vals = [3, 0.5, 1, 0.08, 0.001, 15]
        tuners = []
        for i in range(len(self.param_list)):
            tuner = rospy.set_param(self.param_list[i], init_vals[i])
            tuners.append(tuner)
        self.kps, self.kds, self.kpa, self.kda, self.kia, self.angle_coeff = tuners

        # initialize parking parameters
        self.parking_distance = 1
        if rospy.has_param('park_dist'):
            self.parking_distance = rospy.get_param('park_dist')
        else:
            self.parking_distance = rospy.set_param('park_dist', 1)

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        now = rospy.Time.now().nsecs

        # create the tuning parameters
        tuners = []
        for i in range(len(self.param_list)):
            tuner = rospy.get_param(self.param_list[i])
            tuners.append(tuner)
        self.kps, self.kds, self.kpa, self.kda, self.kia, self.angle_coeff = tuners
        # self.parking_distance = rospy.get_param('park_dist')

        # calculate distance and angle errors and derivatives
        dist_e = np.linalg.norm(np.array([self.relative_x, self.relative_y])) - self.parking_distance
        angle_e = np.arctan2(self.relative_y, self.relative_x)  # radians -pi to pi of the angle of the car dir vs cone
        self.int_ang_e += angle_e # integral term
        self.int_ang_e = max(min(self.int_ang_e, 0.34 / self.kia), -0.34 / self.kia)
        dist_e_dot = (dist_e - self.prev_dist) / ((now - self.prev_time) * (10 ** -9))  # approx dist derivative
        angle_e_dot = (angle_e - self.prev_angle) / ((now - self.prev_time) * (10 ** -9))  # approx angle derivative

        # if the angle error is not zero, allows the car to turn until correct
        self.dist_multiplier = abs(angle_e) * self.angle_coeff
        # calculate the speed
        speed_calc = (self.kps * dist_e + self.kds * dist_e_dot)
        if not np.isclose(angle_e, 0, atol=0.04) and np.isclose(dist_e, 0, atol=0.05):
            speed_calc = 0.5*np.sign(speed_calc)
        speed = max(min(speed_calc, 1), -1)  # cap the speed at 1
        # steering angle depends on the direction of speed and PID loop
        steering_angle = np.sign(speed) * (self.kpa * angle_e + self.kda * angle_e_dot + self.kia * self.int_ang_e)
        steering_angle = max(min(steering_angle, 0.34), -0.34)
        # resets the integral error if it is close to the right direction or pointing the right way
        if np.isclose(dist_e, 0, atol=0.05) or np.isclose(angle_e, 0, atol=0.04):
            self.int_ang_e = 0

        # sets the speed and steering angle to 0 if it is near the goal parking spot
        if np.isclose(dist_e, 0, atol=0.05) and np.isclose(angle_e, 0, atol=0.09):
            rospy.loginfo('==================ZERO===================')
            speed, steering_angle = 0, 0

        cmd = drive_cmd_maker(speed=speed, steering_angle=steering_angle)
        if self.prev_speed == -1*speed:
            rospy.loginfo('%%%%%%%%%%%%%%%%%%%%% PREV CMD %%%%%%%%%%%%%%%%%%%%%%%%%')
            cmd = self.prev_cmd
        self.drive_pub.publish(cmd)
        # publish x, y, and distance error
        self.error_publisher(dist_e * np.cos(angle_e), dist_e * np.sin(angle_e), dist_e)

        # set all of the previous values to these values
        self.prev_dist = dist_e
        self.prev_angle = angle_e
        self.prev_cmd = cmd
        self.prev_time = now
        self.prev_speed = speed
        self.int_ang_e += angle_e

        # log info for debugging
        rospy.loginfo(
            'tuners: ' + str(
                (self.kps, self.kds, self.kpa, self.kda, self.kia, self.angle_coeff, self.dist_multiplier)))
        rospy.loginfo('dist_e: ' + str((dist_e, self.kps * dist_e)))
        rospy.loginfo('dist_e_dot: ' + str((dist_e_dot, self.kds * dist_e_dot)))
        rospy.loginfo('angle_e: ' + str((angle_e, self.kpa * angle_e)))
        rospy.loginfo('integral_angle_e: ' + str((self.int_ang_e, self.kia * self.int_ang_e)))
        rospy.loginfo('angle_e_dot: ' + str((angle_e_dot, self.kda * angle_e_dot)))
        rospy.loginfo('speed: ' + str((speed, speed_calc)))
        rospy.loginfo('steering_angle: ' + str(steering_angle))
        rospy.loginfo('----------------------------------------------------------\n')

    def error_publisher(self, x_e, y_e, dist_e):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        error_msg.x_error = x_e
        error_msg.y_error = y_e
        error_msg.distance_error = dist_e

        self.error_pub.publish(error_msg)


def drive_cmd_maker(speed=0.0, acceleration=0.0, jerk=0.0, steering_angle=0.0, steering_angle_v=0.0):
    """
    :param steering_angle: desired virtual angle (radians)
    :param steering_angle_v: desired rate of change (radians/s), 0 is max
    :param speed: desired forward speed (m/s)
    :param acceleration: desired acceleration (m/s^2)
    :param jerk: desired jerk (m/s^3)

    :return: AckermannDriveStamped type to publish
    """
    drv_cmd = AckermannDriveStamped()
    drv_cmd.header.stamp = rospy.get_rostime()
    drv_cmd.header.frame_id = "base_link"

    drv_cmd.drive.steering_angle = float(steering_angle)
    drv_cmd.drive.steering_angle_velocity = float(steering_angle_v)
    drv_cmd.drive.speed = speed
    drv_cmd.drive.acceleration = acceleration
    drv_cmd.drive.jerk = jerk

    return drv_cmd


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
