#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from kurrier.msg import mission  # 사용자 정의 메시지 임포트
import math

class MissionNode:
    def __init__(self):
        rospy.init_node('mission_node', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.mission_pub = rospy.Publisher('/mission', mission, queue_size=1)

        self.current_position = Point()
        self.mission_info = mission()

        self.proximity_threshold = 3.0
        self.current_mission_index = 2

        # 미션 1 차간간격1
        # 미션 2 정적장애물
        # 미션 3 gps음영1(공사장 장애물 회피)
        # 미션 4 동적장애물(주차)
        # 미션 5 차간간격2
        # 미션 6 gps음영2(장애물)
        # 미션 7 신호등
        # 미션 8 END/정지

        # Define missions with coordinates
        self.missions = [
            {'mission_number': 1, 'x': -118.53937525855144, 'y': 4.976131527218968},
            {'mission_number': 2, 'x': -147.8114318390144, 'y': 28.72403534920886},
            {'mission_number': 3, 'x': -148.67559809767408, 'y': 73.2866192725487},
            {'mission_number': 4, 'x': -72.11734004126629, 'y': 111.0132733262144},
            {'mission_number': 5, 'x': 11.36830715922406, 'y': 106.07728394400328},
            {'mission_number': 6, 'x': -65.31562037597178, 'y': 94.66656312160194},
            {'mission_number': 7, 'x': -69.88784022611799, 'y': -80.74107542820275},
            {'mission_number': 8, 'x': 0.8367664078832604, 'y': -104.31452361121774}
        ]

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            self.update_mission()
            self.mission_pub.publish(self.mission_info)
            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def update_mission(self):
        """Update the current mission based on the current position."""
        if self.current_mission_index >= len(self.missions):
            self.mission_info.mission_num = len(self.missions)
            return   # All missions completed
        
        current_mission = self.missions[self.current_mission_index]
        dist = self.distance(self.current_position.x, self.current_position.y, current_mission['x'], current_mission['y'])
        
        if dist < self.proximity_threshold:
            # Mission reached, update mission info
            self.mission_info.mission_num = self.current_mission_index  # Publish 0-based index
            self.current_mission_index += 1  # Move to the next mission
        else:
            # Continue with the current mission
            self.mission_info.mission_num = self.current_mission_index  # Publish 0-based index

if __name__ == '__main__':
    try:
        mission_node = MissionNode()
    except rospy.ROSInterruptException:
        pass
