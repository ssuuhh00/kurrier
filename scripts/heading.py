#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from std_msgs.msg import Float32
from pyproj import Proj
from math import atan2, degrees, sqrt, sin, cos, pi  # 여기서 pi를 임포트

class GPSIMUHeadingCalculator:
    def __init__(self):
        rospy.init_node('GPS_IMU_heading_calculator', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps/fix", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu_raw", Imu, self.imu_callback)
        self.ego_sub = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback)
        self.heading_pub = rospy.Publisher('/heading', Float32, queue_size=1)
        
        # 초기화
        self.x, self.y = None, None
        self.prev_x, self.prev_y = None, None
        self.is_imu = False
        self.is_gps = False
        self.is_ego = False
        self.heading = None
        self.ego_heading = None

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_imu and self.is_gps and self.is_ego:
                self.calculate_heading()
                if self.heading is not None:
                    self.heading_pub.publish(self.heading)
                    rospy.loginfo(f"IMU와 GPS로 계산한 헤딩값: {self.heading:.2f} degrees")

                    if self.ego_heading is not None:
                        heading_difference = self.calculate_heading_difference()
                        rospy.loginfo(f"/Ego_topic에서 제공해주는 헤딩값: {self.ego_heading:.2f} degrees")
                        rospy.loginfo(f"참값과 계산한 근삿값과의 오차: {heading_difference:.2f} degrees")

            self.is_gps = self.is_imu = self.is_ego = False
            rate.sleep()

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.convertLL2UTM()
        self.is_gps = True

    def convertLL2UTM(self):    
        xy_zone = self.proj_UTM(self.lon, self.lat)

        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

    def imu_callback(self, data):
        # IMU 데이터를 사용해 직접 yaw (heading) 계산
        qx = data.orientation.x
        qy = data.orientation.y
        qz = data.orientation.z
        qw = data.orientation.w

        # Quaternion을 Euler 각도로 변환하여 yaw 값 (Z축 회전 각도) 계산
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.heading = atan2(siny_cosp, cosy_cosp) * (180.0 / pi)

        # heading 값을 -180도에서 180도 사이로 조정
        if self.heading > 180:
            self.heading -= 360
        elif self.heading < -180:
            self.heading += 360

        self.is_imu = True

    def ego_callback(self, ego_msg):
        # EgoVehicleStatus 메시지에서 heading 값 추출
        self.ego_heading = ego_msg.heading
        self.is_ego = True

    def calculate_heading(self):
        if self.prev_x is not None and self.prev_y is not None:
            # GPS에서의 상대적인 방향 계산은 보조적으로만 사용
            delta_x = self.x - self.prev_x
            delta_y = self.y - self.prev_y

            if sqrt(delta_x**2 + delta_y**2) > 0:  # Avoid division by zero
                gps_heading = atan2(delta_y, delta_x) * (180.0 / pi)
                if gps_heading > 180:
                    gps_heading -= 360
                elif gps_heading < -180:
                    gps_heading += 360

                # GPS를 이용한 heading과 IMU에서 계산한 heading 비교 및 보정
                self.heading = self.heading * 0.9 + gps_heading * 0.1  # 가중 평균
            else:
                rospy.logwarn("GPS movement too small to calculate heading.")
        else:
            rospy.logwarn("No previous GPS data available for heading calculation.")

        # Update previous GPS coordinates
        self.prev_x = self.x
        self.prev_y = self.y

    def calculate_heading_difference(self):
        if self.heading is not None and self.ego_heading is not None:
            # Calculate the absolute difference between the two headings
            diff = self.heading - self.ego_heading
            # Normalize the difference to be within -180 to 180 degrees
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            return diff
        return None

if __name__ == '__main__':
    try:
        GPSIMUHeadingCalculator()
    except rospy.ROSInterruptException:
        pass
