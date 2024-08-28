#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from kurrier.msg import mission  # 사용자 정의 메시지 임포트
from tf.transformations import euler_from_quaternion
from math import sin, cos
from std_msgs.msg import Bool

class TFNode:
    def __init__(self):
        rospy.init_node('tf_node', anonymous=True)
        rospy.Subscriber("/gps/fix", GPSMessage, self.navsat_callback)
        rospy.Subscriber("/imu_raw", Imu, self.imu_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)
        rospy.Subscriber("/liorf/mapping/odometry", Odometry, self.liorf_callback)
        rospy.Subscriber("/is_stop", Bool, self.stop_callback)

        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        self.odom_start_pub = rospy.Publisher('/odom_slam_start',Odometry, queue_size=1)

        # 초기화
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False
        self.is_stopped = False
        self.is_1st_slam_started = False
        self.is_2nd_slam_started = False
        self.is_slam_odom = False
        self.initial_yaw = None
        self.start_position = Odometry()   #slam 초기 위치

        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'
        self.odom_slam_msg=Odometry()
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not (self.is_1st_slam_started or self.is_2nd_slam_started):
                if  self.is_imu==True and self.is_gps == True:
                    self.convertLL2UTM()
                    self.odom_pub.publish(self.odom_msg)
                self.is_gps = self.is_imu = False
            elif (self.is_1st_slam_started or self.is_2nd_slam_started) and self.is_slam_odom:
                if (self.start_position is not None) and (self.initial_yaw is not None):
                    # SLAM으로부터 받은 차량의 로컬 좌표 위치를 저장
                    x1 = self.odom_slam_msg.pose.pose.position.x
                    y1 = self.odom_slam_msg.pose.pose.position.y
                    # # 라이다 gps 센서 위치 차이보정
                    x1 += 1.33
                    # 위치 설정
                    self.odom_msg.pose.pose.position.x = self.start_position.pose.pose.position.x + cos(self.initial_yaw) * x1 - sin(self.initial_yaw) * y1
                    self.odom_msg.pose.pose.position.y = self.start_position.pose.pose.position.y + sin(self.initial_yaw) * x1 + cos(self.initial_yaw) * y1
                    self.odom_msg.pose.pose.position.z = self.start_position.pose.pose.position.z
                    self.odom_pub.publish(self.odom_msg)
                    self.odom_start_pub.publish(self.start_position)
                else: 
                    rospy.loginfo("start_position or initial_yaw is None")
                self.is_slam_odom = False
            rate.sleep()

    def get_yaw(self):    
        if self.odom_msg is not None:
            # 쿼터니언을 Euler 각도로 변환
            orientation = [self.odom_msg.pose.pose.orientation.x,
                        self.odom_msg.pose.pose.orientation.y,
                        self.odom_msg.pose.pose.orientation.z,
                        self.odom_msg.pose.pose.orientation.w]

            # 쿼터니언에서 Euler 각도 (roll, pitch, yaw)로 변환
            _, _, heading = euler_from_quaternion(orientation)
            return heading

    def convertLL2UTM(self):    
        xy_zone = self.proj_UTM(self.lon, self.lat)

        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = self.z

    def navsat_callback(self, gps_msg):
        if not (self.is_1st_slam_started or self.is_2nd_slam_started):
            self.lat = gps_msg.latitude
            self.lon = gps_msg.longitude
            self.e_o = gps_msg.eastOffset
            self.n_o = gps_msg.northOffset
            self.z = gps_msg.altitude
            
            self.is_gps=True

    def imu_callback(self, data):
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w
        self.is_imu=True

    def stop_callback(self, msg):
        self.is_stopped = msg.data

    def liorf_callback(self, msg):
        self.odom_slam_msg = msg
        self.is_slam_odom = True

    def mission_callback(self, msg):
        # GPS 음영 미션 시작
        if msg.mission_num == 3 and not self.is_1st_slam_started:
            if self.is_stopped:
                self.start_position.pose.pose.position.x = self.odom_msg.pose.pose.position.x
                self.start_position.pose.pose.position.y = self.odom_msg.pose.pose.position.y
                self.start_position.pose.pose.position.z = self.odom_msg.pose.pose.position.z
                self.start_position.pose.pose.orientation.w = self.odom_msg.pose.pose.orientation.w
                
                self.initial_yaw = self.get_yaw()

                # 시작 위치와 yaw 값 로깅
                rospy.loginfo("SLAM started at position: x = {:.6f}, y = {:.6f}, yaw = {:.6f}".format(
                    self.start_position.pose.pose.position.x,
                    self.start_position.pose.pose.position.y,
                    self.initial_yaw
                ))
                self.is_1st_slam_started = True

        # gps음영 미션 끝
        elif msg.mission_num != 3 and self.is_1st_slam_started:
            self.start_position = Odometry()
            self.is_1st_slam_started = False
            self.initial_yaw = None

        # GPS 음영 미션 시작
        elif msg.mission_num == 6 and not self.is_2nd_slam_started:
            if self.is_stopped:
                self.start_position.pose.pose.position.x = self.odom_msg.pose.pose.position.x
                self.start_position.pose.pose.position.y = self.odom_msg.pose.pose.position.y
                self.start_position.pose.pose.position.z = self.odom_msg.pose.pose.position.z

                self.initial_yaw = self.get_yaw()

                # 시작 위치와 yaw 값 로깅
                rospy.loginfo("SLAM started at position: x = {:.6f}, y = {:.6f}, yaw = {:.6f}".format(
                    self.start_position.pose.pose.position.x,
                    self.start_position.pose.pose.position.y,
                    self.initial_yaw
                ))
                self.is_2nd_slam_started = True

        # gps음영 미션 끝
        elif msg.mission_num != 6 and self.is_2nd_slam_started:
            self.start_position = Odometry()
            self.is_2nd_slam_started = False
            self.initial_yaw = None

    
if __name__ == '__main__':
    try:
        tf_node = TFNode()
    except rospy.ROSInterruptException:
        pass
