#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' This code is to compare the performance of the estimation technique. The
estimated states, measured states and the open loop simulation states are
plotted to compare the performance. This code is bit slow due to online
plotting of matplotlib. Use other plotter for real time debugging such as
plotjuggler. This plot is created for documentation which has nice appearance.
 '''

import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from sensor_fusion.msg import sensorReading, control
from PIL import Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Pose
from math import pi, sin, cos, tan, atan
import datetime
import os
import matplotlib.animation as animation

#### plotter for visual sensor vs visual-inertial sensor ##########
def plot_cam_pose(x_lim,y_lim):

    xdata = []; ydata = []
    fig = plt.figure(figsize=(10,8))
    plt.ion()
    plt.xlim([-1*x_lim,x_lim])
    plt.ylim([-1*y_lim,y_lim])

    axtr = plt.axes()

    line_pure,        = axtr.plot(xdata, ydata, '-g', label = 'Localization using visual only', linewidth =2 )
    line_fused,       = axtr.plot(xdata, ydata, '-b', label = 'Localization using visual-inertial', linewidth =2)  

    plt.legend()
    plt.grid()
    

    return fig, plt, line_pure, line_fused


### wrap the angle between [-pi,pi] ###
def wrap(angle):
    if angle < -np.pi:
        w_angle = 2 * np.pi + angle
    elif angle > np.pi:
        w_angle = angle - 2 * np.pi
    else:
        w_angle = angle

    return w_angle


#################################### SUbsribe to fisheye camera ##################################### 
class fiseye_cam():


    def __init__(self):
        '''
        1. /pure_cam_pose topic estimate the camera position in world frame using only camera information 
        2. /fused_cam_pose topic estimate the camera position in world frame using IMU and camera information 
        '''

        rospy.Subscriber('pure_cam_pose', Pose, self.pure_cam_pose_callback, queue_size=1)
        rospy.Subscriber('fused_cam_pose', Pose, self.fused_cam_pose_callback, queue_size=1)

        #### Homogeneous transformation for reference change####
        # self.x_tf     = rospy.get_param("switching_lqr_observer/x_tf")
        # self.y_tf     = rospy.get_param("switching_lqr_observer/y_tf")
        # theta_tf = rospy.get_param("switching_lqr_observer/theta_tf")*pi/180
        # self.R_tf = np.array([[cos(theta_tf), -sin(theta_tf)],
        #                  [sin(theta_tf),  cos(theta_tf)]])
        # self.yaw_tf   = rospy.get_param("switching_lqr_observer/yaw_tf")*pi/180

        self.x_tf     = 0.
        self.y_tf     = 0.
        theta_tf = 0.*pi/180
        self.R_tf = np.array([[cos(theta_tf), -sin(theta_tf)],
                         [sin(theta_tf),  cos(theta_tf)]])
        self.yaw_tf   = 0.*pi/180


        self.pure_x   = 0.0
        self.pure_y   = 0.0
        self.pure_yaw = 0.0
        

        self.fused_x             = 0.0
        self.fused_y             = 0.0
        self.fused_yaw           = 0.0


    def pure_cam_pose_callback(self, data):

        self.pure_x   = data.position.x
        self.pure_y   = data.position.y
        self.pure_yaw = data.orientation.z


    def fused_cam_pose_callback(self, data):

        [self.fused_x, self.fused_y] = np.dot(self.R_tf, np.array([data.position.x,data.position.y]).T)
        self.fused_x = self.fused_x - self.x_tf
        self.fused_y = self.fused_y - self.y_tf
        self.fused_yaw = wrap(data.orientation.z + self.yaw_tf)





def main():


    rospy.init_node('fisheyecam_plotter', anonymous=True)
    loop_rate       = 2000
    rate            = rospy.Rate(loop_rate)

    cam_pose  = fiseye_cam()

    cam_pose_pure_x         = 0
    cam_pose_pure_y         = 0

    cam_pose_fused_x        = 0
    cam_pose_fused_y        = 0

    cam_pose_pure_x_hist    = []
    cam_pose_pure_y_hist    = []

    cam_pose_fused_x_hist   = []
    cam_pose_fused_y_hist   = []

    record_on = True
    visualization  = True

    if visualization == True:
        x_lim = 3
        y_lim = 6
        fig, plt, line_pure, line_fused = plot_cam_pose(x_lim,y_lim)

    counter = 0
    while not (rospy.is_shutdown()):


        ########################################### unpack messages ############################################

        cam_pure_x, cam_pure_y = cam_pose.pure_x, cam_pose.pure_y
        cam_fused_x, cam_fused_y = cam_pose.fused_x, cam_pose.fused_y

        ########################################################################################################


        cam_pose_pure_x_hist.append(cam_pure_x)
        cam_pose_pure_y_hist.append(cam_pure_y)
        cam_pose_fused_x_hist.append(cam_fused_x)
        cam_pose_fused_y_hist.append(cam_fused_y)

        if visualization == True:
            line_pure.set_data(cam_pose_pure_x_hist ,cam_pose_pure_y_hist)
            line_fused.set_data(cam_pose_fused_x_hist ,cam_pose_fused_y_hist)

            fig.canvas.draw()

        ##########################################################################################################

            plt.show()

            plt.pause(1.0/3000)

        counter +=1
        rate.sleep()

    print "Saving GIF images"

    if record_on == True:
        path = ('/').join(__file__.split('/')[:-2]) + '/data/camera/' 
            
        now = datetime.datetime.now()
        # path = path + now.strftime("d%d_m%m_y%Y/")
        path = path + now.strftime("d%d_m%m_y%Y_hr%H_min%M_sec%S")

        if not os.path.exists(path):
            os.makedirs(path)

        pure_path = path + '/pure_cam_pose'
        fused_path = path + '/fused_cam_pose'  

        pure_data = {'X':cam_pose_pure_x_hist, 'Y': cam_pose_pure_y_hist}
        fused_data = {'X':cam_pose_fused_x_hist, 'Y': cam_pose_fused_y_hist}
        

        np.save(pure_path, pure_data)
        np.save(fused_path, fused_data)

    # from collections import deque
    # import itertools
    # fig, plt, line_pure, line_fused = plot_cam_pose(x_lim,y_lim)
    # # history_len = 50  # how many trajectory points to display
    # # history_x, history_y = deque(maxlen=history_len), deque(maxlen=history_len)

    # ani_x = []
    # ani_y = []

    # def data_gen():
    #     for cnt in itertools.count():
    #         yield cnt

    # def animate(i):
    #     ani_x.append(cam_pose_pure_x_hist[:i]) 
    #     ani_y.append(cam_pose_pure_y_hist[:i])
    #     line_pure.set_data(ani_x , ani_y)
    #     # line_fused.set_data(cam_pose_fused_x_hist[i] ,cam_pose_fused_y_hist[i])

    #     return line_pure, #, line_fused

    # ani = animation.FuncAnimation(fig, animate, len(cam_pose_pure_y_hist), interval=10, blit=True)
    # plt.show()



    # Save into a GIF file that loops forever
    # image_dy_his[0].save("/home/auto/Desktop/autonomus_vehicle_project/thesis/TFM_Shivam/raw_doc/estimator/images/vehilce_motion.gif", format='GIF', append_images=image_dy_his[1:], save_all=True, duration=300, loop=0)
    # from moviepy.editor import ImageSequenceClip
    # clip = ImageSequenceClip(list(np.array(image_veh_his)), fps=20)
    # clip.write_gif('test.gif', fps=20)

    print "Saved"
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
