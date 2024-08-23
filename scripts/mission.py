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

        # 미션 0 시작점
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
            {'mission_number': 0, 'x': -93.40589900134364, 'y': 17.37121018813923},
            {'mission_number': 1, 'x': -118.02613825938897, 'y': 10.89159974316135},
            {'mission_number': 2, 'x': -147.8114318390144, 'y': 28.72403534920886},
            {'mission_number': 3, 'x': -147.366516067239, 'y': 78.13069733697921},
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
        # Iterate through missions to find the one we're currently targeting
        for mission in self.missions:
            dist = self.distance(self.current_position.x, self.current_position.y, mission['x'], mission['y'])
            
            if dist < self.proximity_threshold:
                # Mission reached, update mission info
                self.mission_info.mission_num = mission['mission_number']
                
                # Optionally, remove or skip to the next mission
                # For example, if missions are sequential and you only want to complete one at a time:
                #self.missions.remove(mission)
        
        # 미션번호 강제할당
        #self.mission_info.mission_num = 2
        


if __name__ == '__main__':
    try:
        mission_node = MissionNode()
    except rospy.ROSInterruptException:
        pass
