#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from math import sqrt, radians, sin, cos, atan2
from nav_msgs.msg import Odometry

class pathMaker:
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 초기화
        self.prev_x = None
        self.prev_y = None
        self.is_status = False

        # 패키지 경로 로드 & 파일 쓰기 모드
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = pkg_path + '/path/' + path_name + '.txt'
        self.f = open(full_path, 'w')

        rospy.spin()
        self.f.close()
    
    def odom_callback(self, msg):
        self.is_status = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        if self.prev_x is None and self.prev_y is None:
            self.prev_x = self.x
            self.prev_y = self.y
            return

        distance = self.euclidean_distance(self.x, self.y, self.prev_x, self.prev_y)
        # 이전 waypoint와의 거리가 0.3 이상이어야 기록
        if distance > 0.3:
            data = '{0}\t{1}\t{2}\n'.format(self.x, self.y, self.z)
            self.f.write(data)
            self.prev_x = self.x
            self.prev_y = self.y
            print("write : ", self.x, self.y, self.z)

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculate the Euclidean distance between two points 
        in 2D space given their coordinates.
        """
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)

    
if __name__ == '__main__':
    try:
        p_m = pathMaker("kurrier", "cityhall_path")
    except rospy.ROSInternalException:
        pass
