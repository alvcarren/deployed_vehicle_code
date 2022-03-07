#!/usr/bin/env python

from math import tan, atan, cos, sin, pi, atan2, fmod
import numpy as np
import matplotlib.patches as patches
import rospy
from numpy.random import randn,rand
import rosbag
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Float32
from sensor_fusion.msg import sensorReading, control
import tf
import time
from numpy import linalg as LA
import datetime
import os
import sys
import scipy.io as sio
import sys

sys.path.append(('/').join(sys.path[0].split('/')[:-2])+'/observer/src/')


class random_input_generator:
    def __init__(self):
        self.sensor = sensorReading()
        self.time0 = rospy.get_rostime().to_sec()
        self.sensor.timestamp_ms = self.time0
        self.sensor.X = -10
        self.sensor.Y = -10
        self.sensor.roll = -10
        self.sensor.yaw = -10
        self.sensor.pitch = -10
        self.sensor.vx = -10
        self.sensor.vy = -10
        self.sensor.yaw_rate = -10
        self.sensor.ax = -10
        self.sensor.ay = -10
        self.sensor.s = -10
        self.sensor.x = -10
        self.sensor.y = -10
        self.accel = 1;
        self.steering = 0.2;
        self.mu1 = 0.7
        self.mu2 = 0
        self.sigma1 = 0.1
        self.sigma2 = 0.1
        self.accel_publisher  = rospy.Publisher('control/accel', Float32, queue_size=1)
        self.steering_publisher = rospy.Publisher('control/steering',Float32,queue_size=1)
        self. sensor = rospy.Subscriber('est_state_info', sensorReading,self.update_sensors_info, queue_size=1)


    def update_sensors_info(self,data):
        self.sensor.timestamp_ms = rospy.get_rostime().to_sec() -self.time0
        self.sensor.X = data.X
        self.sensor.Y = data.Y
        self.sensor.roll = data.roll
        self.sensor.yaw = data.yaw
        self.sensor.pitch = data.pitch
        self.sensor.vx = data.vx
        self.sensor.vy = data.vy
        self.sensor.yaw_rate = data.yaw_rate
        self.sensor.ax = data.ax
        self.sensor.ay = data.ay
        self.sensor.s = data.s
        self.sensor.x = data.x
        self.sensor.y = data.y


    def publish_control(self):
        self.accel = self.sigma1 * np.random.randn() + self.mu1 + (self.mu1/5)*(sin(rospy.get_rostime().to_sec()*2*pi/2.5)+sin(rospy.get_rostime().to_sec()*2*pi/7.1)+sin(rospy.get_rostime().to_sec()*2*pi/12)+sin(rospy.get_rostime().to_sec()*2*pi/0.35)+sin(rospy.get_rostime().to_sec()*2*pi/7.38))
        self.steering = self.sigma2 * np.random.randn() + (0.35/5)*(sin(rospy.get_rostime().to_sec()*2*pi/3)+sin(rospy.get_rostime().to_sec()*2*pi/5)+sin(rospy.get_rostime().to_sec()*2*pi/4)+sin(rospy.get_rostime().to_sec()*2*pi/17)+sin(rospy.get_rostime().to_sec()*2*pi/7.38))
        self.accel = max(0,min(self.accel,1))
        self.steering = max(-0.35,min(self.steering,0.35))
        self.accel_publisher.publish(self.accel)
        self.steering_publisher.publish(self.steering)


def main():
    print("NODE FOR PUBLISHING RANDOM ACCELS/STEERINGS")
    rospy.init_node('random_input_generation', anonymous=True)

    loop_rate   = rospy.get_param("switching_lqr_observer/publish_frequency")
    rate        = rospy.Rate(loop_rate)
    random_pub = random_input_generator()
    time.sleep(3)


    while not(rospy.is_shutdown()):
        random_pub.publish_control()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
