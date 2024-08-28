#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, atan2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from kurrier.msg import mission  # 사용자 정의 메시지 임포트
from std_msgs.msg import Int32MultiArray

class parking_check:
    def __init__(self):
        rospy.init_node('parking_check', anonymous=True)

        # Subscriber, Publisher 선언
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cluster_points", PointCloud2, self.object_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)

        self.parking_pub = rospy.Publisher('/parking_point', Int32MultiArray, queue_size=1)

        self.mission_info = mission()
        self.parking_point = [False] * 31  # 31개의 주차 공간을 나타내는 배열
        self.object_points = []
        self.is_odom = False

        self.is_check_fin = False

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.mission_info.mission_num == 4:
                empty_spots = [i for i, occupied in enumerate(self.parking_point) if not occupied]
                # rospy.loginfo(f"Empty parking spots: {empty_spots}")
                empty_spots_msg = Int32MultiArray()
                empty_spots_msg.data = empty_spots
                self.parking_pub.publish(empty_spots_msg)
            rate.sleep()

    def checkparking(self):
        parking_ranges = [
        (-61.18, -63.25),
        (-58.98, -61.18),
        (-56.69, -58.98),
        (-54.45, -56.69),
        (-52.10, -54.45),

        (-49.82, -52.10),
        (-47.45, -49.82),
        (-45.19, -47.45),
        (-42.86, -45.19),
        (-40.62, -42.86),

        (-38.40, -40.62),
        (-36.09, -38.40),
        (-33.58, -36.09),
        (-31.31, -33.58),
        (-29.12, -31.31),

        (-26.83, -29.12),
        (-24.54, -26.83),
        (-22.23, -24.54),
        (-19.95, -22.23),
        (-17.60, -19.95),

        (-15.33, -17.60),
        (-12.96, -15.33),
        (-10.73, -12.96),
        (-8.50, -10.73),
        (-6.15, -8.50),

        (-3.73, -6.15),
        (-1.56, -3.73),
        (0.82, -1.56),
        (3.04, 0.82),
        (5.29, 3.04),

        (7.28, 5.29)
        ]

        padding = 0.3  # 축소할 범위 (padding) 값, 필요에 따라 조절 가능

        # 각 장애물에 대해 주차공간 점유 여부 확인
        for point in self.object_points:
            for i, (x_max, x_min) in enumerate(parking_ranges):
                # 패딩을 적용해 범위를 축소
                if (x_min + padding) < point[0] < (x_max - padding) and 103.6 < point[1] < 108:
                    self.parking_point[i] = True  # 주차공간이 점유됨을 표시
                    rospy.loginfo(f"Parking spot {i} occupied by point ({point[0]}, {point[1]})")
                    break

    def mission_callback(self, msg):
        self.mission_info = msg

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        # rospy.loginfo("odom received and stored. is_odom = True")

    def object_callback(self, msg):
        if not self.is_odom:
            # rospy.logwarn("odom not received yet, skipping object processing.")
            return

        self.object_points.clear()  # 이전 장애물 데이터를 초기화
        # 클러스터링된 장애물 포인트들을 추출하여 절대 좌표로 변환하여 저장합니다.
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            abs_x, abs_y, abs_z = self.relative_to_absolute(point[0], point[1], point[2])
            self.object_points.append([abs_x, abs_y, abs_z])
        
        # rospy.loginfo(f"Received and stored {len(self.object_points)} obstacle points. is_obj = True")
        # if len(self.object_points) > 0:
        #     rospy.loginfo(f"First obstacle point (absolute): x={self.object_points[0][0]}, y={self.object_points[0][1]}, z={self.object_points[0][2]}")
        
        # 객체 포인트를 기반으로 주차 상태를 확인
        self.checkparking()
    
    def relative_to_absolute(self, rel_x, rel_y, rel_z):
        """
        장애물의 상대 좌표를 절대 좌표로 변환합니다.
        """
        # 차량의 현재 위치와 헤딩 정보를 가져옵니다.
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y
        ego_z = self.odom_msg.pose.pose.position.z

        # 오리엔테이션에서 헤딩 방향을 계산
        orientation = self.odom_msg.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        # 변환 행렬을 사용하여 상대 좌표를 절대 좌표로 변환
        abs_x = ego_x + cos(heading) * (rel_x + 1.33) - sin(heading) * rel_y
        abs_y = ego_y + sin(heading) * (rel_x + 1.33) + cos(heading) * rel_y
        abs_z = ego_z + rel_z  # z는 일반적으로 사용되지 않지만 포함

        # rospy.loginfo(f"Converted relative position ({rel_x}, {rel_y}, {rel_z}) to absolute position ({abs_x}, {abs_y}, {abs_z}) with heading {heading}")
        return abs_x, abs_y, abs_z

if __name__ == '__main__':
    try:
        parking_check()
    except rospy.ROSInterruptException:
        pass
