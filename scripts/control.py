#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EventInfo
from std_msgs.msg import Int16, Bool
import numpy as np
from tf.transformations import euler_from_quaternion
from kurrier.msg import mission, obstacle   # 사용자 정의 메시지 임포트 
from morai_msgs.srv import MoraiEventCmdSrv
from enum import Enum
from std_msgs.msg import Int32MultiArray

class pure_pursuit:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_odom = False
        self.is_yolo = False

        self.traffic_color = ""
        self.obstacle = obstacle()
        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 3
        self.lfd = 5

        self.is_waiting_time = False
        self.count = 0
        self.mission_info = mission()


        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:

                # path의 절반 지점 찾기
                mid_index = len(self.path.poses) // 2
                mid_pose = self.path.poses[mid_index].pose.position

                # local_path_point 설정
                local_path_point = [mid_pose.x, mid_pose.y]

                theta = atan2(local_path_point[1], local_path_point[0])
                
                default_vel = 10

                # if self.is_look_forward_point:
                if self.mission_info.mission_num == 3:
                    # steering
                    self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                    normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                    self.ctrl_cmd_msg.velocity = default_vel * (1 - 0.6 * normalized_steer)
                else:
                    self.ctrl_cmd_msg.steering = 0
                    self.ctrl_cmd_msg.velocity = default_vel
                # else:
                #     rospy.loginfo_once("No forward point found")
                #     self.ctrl_cmd_msg.steering = 0.0
                #     self.ctrl_cmd_msg.velocity = 0.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            self.is_path = self.is_odom = False
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def mission_callback(self, msg):
        self.mission_info = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
