#!/usr/bin/env python

from math import tan, atan, cos, sin, pi, atan2, fmod
import numpy as np
import matplotlib.pyplot as plt
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
from observer.msg import relevantData

sys.path.append(('/').join(sys.path[0].split('/')[:-2])+'/observer/src/')


class InfoUnifier:
    def __init__(self):
        self.relevant_msg = relevantData()
        self.time0 = rospy.get_rostime().to_sec()
        self.relevant_msg.timestamp_ms = self.time0
        self.relevant_msg.X = -10
        self.relevant_msg.Y = -10
        self.relevant_msg.roll = -10
        self.relevant_msg.yaw = -10
        self.relevant_msg.pitch = -10
        self.relevant_msg.vx = -10
        self.relevant_msg.vy = -10
        self.relevant_msg.yaw_rate = -10
        self.relevant_msg.ax = -10
        self.relevant_msg.ay = -10
        self.relevant_msg.s = -10
        self.relevant_msg.x = -10
        self.relevant_msg.y = -10
        self.relevant_msg.steering = -10
        self.relevant_msg.accel = -10
        self.info_publisher  = rospy.Publisher('relevant_data', relevantData, queue_size=1)
        rospy.Subscriber('control/accel', Float32, self.update_accel, queue_size=1)
        rospy.Subscriber('control/steering', Float32, self.update_steering, queue_size=1)
        rospy.Subscriber('est_state_info', sensorReading,self.update_sensors_info, queue_size=1)

    def update_accel(self,data):
        self.relevant_msg.timestamp_ms = rospy.get_rostime().to_sec() -self.time0
        self.relevant_msg.accel = data.data

    def update_steering(self,data):
        self.relevant_msg.timestamp_ms = rospy.get_rostime().to_sec() -self.time0
        self.relevant_msg.steering = data.data

    def update_sensors_info(self,data):
        self.relevant_msg.timestamp_ms = rospy.get_rostime().to_sec() -self.time0
        self.relevant_msg.X = data.X
        self.relevant_msg.Y = data.Y
        self.relevant_msg.roll = data.roll
        self.relevant_msg.yaw = data.yaw
        self.relevant_msg.pitch = data.pitch
        self.relevant_msg.vx = data.vx
        self.relevant_msg.vy = data.vy
        self.relevant_msg.yaw_rate = data.yaw_rate
        self.relevant_msg.ax = data.ax
        self.relevant_msg.ay = data.ay
        self.relevant_msg.s = data.s
        self.relevant_msg.x = data.x
        self.relevant_msg.y = data.y

def main():
    print("NODE FOR RECORD MAIN INFORMATION")
    rospy.init_node('record_relevant_data', anonymous=True)

    loop_rate   = rospy.get_param("switching_lqr_observer/publish_frequency")
    rate        = rospy.Rate(loop_rate)
    info_unifier = InfoUnifier()
    time.sleep(3)


    while not(rospy.is_shutdown()):
        info_unifier.info_publisher.publish(info_unifier.relevant_msg)
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
