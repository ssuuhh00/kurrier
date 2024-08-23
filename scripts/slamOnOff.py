#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kurrier.msg import mission  # 사용자 정의 메시지 임포트
import subprocess
import time

class slamOnOffNode:
    def __init__(self):
        rospy.init_node('slamOnOff_node', anonymous=True)
        rospy.Subscriber("/mission", mission, self.mission_callback)

        self.mission_info = mission()
        self.is_slam_started = False

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            self.check_mission()
            rate.sleep()

    def mission_callback(self, msg):
        self.mission_info = msg

    def check_mission(self):
        if self.mission_info.mission_num == 3 and not self.is_slam_started:
            # 1. slam 런치 파일 실행
            launch_command = ["roslaunch", "kurrier", "kurrierSlam.launch"]
            process = subprocess.Popen(launch_command)
            time.sleep(6)  # 필요한 만큼 대기 (예: 다른 작업 수행)
            self.is_slam_started = True

        elif self.mission_info.mission_num != 3 and self.is_slam_started:
            # 2. slam 런치 파일 종료
            process.terminate()  # 또는 
            #process.kill()
            process.wait()  # 프로세스가 종료될 때까지 대기
            self.is_slam_started = False

if __name__ == '__main__':
    try:
        slamOnOff_node = slamOnOffNode()
    except rospy.ROSInterruptException:
        pass
