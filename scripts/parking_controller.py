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

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = rospy.default_param('park_dist', 0.75) # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.prev_dist = 0
        self.prev_angle = 0
        self.prev_time = 0

        self.kps = 1
        self.kds = 1
        self.kpa = 1
        self.kda = 1


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        now = rospy.Time.now().nsecs

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################
        dist_e = np.linalg.norm(np.array([self.relative_x, self.relative_y]))
        angle_e = np.arctan2(self.relative_y,self.relative_x) # radians -pi to pi of the angle of the car dir vs cone
        dist_e_dot = (dist_e - self.prev_dist)/((now - self.prev_time)*(10**-9)) # approx dist derivative
        angle_e_dot = (angle_e - self.prev_angle) / ((now - self.prev_time) * (10 ** -9))  # approx angle derivative

        speed = self.kps*dist_e + self.kds*angle_e_dot
        steering_angle = np.sign(speed)*(self.kpa*dist_e+self.kda*angle_e_dot)


        self.drive_pub.publish(drive_cmd_maker(speed=speed, steering_angle=steering_angle))

        self.error_publisher()
        self.prev_dist = dist_e
        self.prev_angle = angle_e
        self.prev_time = now

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
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
