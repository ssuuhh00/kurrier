#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kurrier.msg import mission  # 사용자 정의 메시지 임포트
import subprocess
import time
from std_msgs.msg import Bool
import os
import signal

class slamOnOffNode:
    def __init__(self):
        rospy.init_node('slamOnOff_node', anonymous=True)
        rospy.Subscriber("/mission", mission, self.mission_callback)
        rospy.Subscriber("/is_stop", Bool, self.stop_callback)

        self.mission_info = mission()
        self.slam_process = None  # SLAM 프로세스를 추적하기 위한 변수

        self.is_first_slam_started = False
        self.is_second_slam_started = False

        self.is_stopped = False

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            self.check_mission()
            rate.sleep()

    def mission_callback(self, msg):
        self.mission_info = msg

    def stop_callback(self, msg):
        self.is_stopped = msg.data

    def start_slam(self):
        if self.slam_process is None:
            launch_command = ["roslaunch", "kurrier", "kurrierSlam.launch"]
            self.slam_process = subprocess.Popen(launch_command, preexec_fn=os.setsid)
            time.sleep(3)
            rospy.loginfo("SLAM started.")

    def stop_slam(self):
        if self.slam_process is not None:
            os.killpg(os.getpgid(self.slam_process.pid), signal.SIGKILL)
            self.slam_process.wait()
            self.slam_process = None
            rospy.loginfo("SLAM stopped.")

            # 명시적으로 SLAM 관련 노드 종료
            subprocess.call(["rosnode", "kill", "/liorf_mapOptmization"])
            subprocess.call(["rosnode", "kill", "/liorf_imageProjection"])
            subprocess.call(["rosnode", "kill", "/liorf_imuPreintegration"])
            subprocess.call(["rosnode", "kill", "/liorf_rviz"])

            # imu_modifier에서 실행된 노드 종료
            subprocess.call(["rosnode", "kill", "/imu_Hz_calculator"])
            subprocess.call(["rosnode", "kill", "/imu_noise_adder"])
            subprocess.call(["rosnode", "kill", "/slam_planning"])
            
    def check_mission(self):
        if self.mission_info.mission_num == 3 and not self.is_first_slam_started:
            if self.is_stopped:
                self.start_slam()
                self.is_first_slam_started = True

        elif self.mission_info.mission_num != 3 and self.is_first_slam_started:
            self.stop_slam()
            self.is_first_slam_started = False

        elif self.mission_info.mission_num == 6 and not self.is_second_slam_started:
            if self.is_stopped:
                self.start_slam()
                self.is_second_slam_started = True

        elif self.mission_info.mission_num != 6 and self.is_second_slam_started:
            self.stop_slam()
            self.is_second_slam_started = False
            
if __name__ == '__main__':
    try:
        slamOnOff_node = slamOnOffNode()
    except rospy.ROSInterruptException:
        pass
