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

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

class pure_pursuit:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_info", obstacle, self.yolo_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)
        rospy.Subscriber("traffic_light_color", Int16, self.traffic_callback)
        rospy.Subscriber("/check_finish", Bool, self.finish_callback)

        self.stop_pub = rospy.Publisher('/is_stop', Bool, queue_size=1)
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

        self.is_finish = False
        self.is_stopped = False
        self.M7_complete = False
        
        #self.find_parking_zone = False
        self.parking_complete = False

        self.mission4_forward = True
        self.mission4_reverse = False
        self.find_parking_zone = True


        self.candidate_parking_zone = [ # 주차 공간 후보
            (-60.0301, 106.8783),
            (-57.7333, 106.8861),
            (-55.3948, 107.0587),
            (-53.2484, 106.8793),
            (-50.8352, 106.9353),
            (-48.6467, 106.8976),
            (-46.2329, 107.0335),
            (-43.9851, 106.9305),
            (-41.7079, 106.8246),
            (-39.4013, 106.9280),
            (-37.1470, 106.9276),
            (-34.6739, 106.9713),
            (-32.3096, 106.8651),
            (-30.1206, 106.8224),
            (-27.8277, 106.9032),
            (-25.6590, 106.8751),
            (-23.2933, 106.9134),
            (-20.9291, 106.8984),
            (-18.6550, 107.0005),
            (-16.3171, 106.8833),
            (-14.1133, 106.8670),
            (-11.8487, 106.8270),
            (-9.4688, 106.9106),
            (-7.2468, 106.8766),
            (-4.8295, 106.8582),
            (-2.5805, 106.9342),
            (-0.2581, 106.9435),
            (1.98631, 106.9997),
            (4.29672, 106.9429)
        ]

        # 기어 변경 서비스 설정
        rospy.loginfo("connecting service")
        rospy.wait_for_service('/Service_MoraiEventCmd', timeout=5)
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
        rospy.loginfo("service connected")

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                vehicle_position = self.current_postion
                self.is_look_forward_point = False

                translation = [vehicle_position.x, vehicle_position.y]

                t = np.array([
                    [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                    [0, 0, 1]
                ])

                det_t = np.array([
                    [t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])],
                    [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],
                    [0, 0, 1]
                ])

                for num, i in enumerate(self.path.poses):
                    path_point = i.pose.position

                    global_path_point = [path_point.x, path_point.y, 1]
                    local_path_point = det_t.dot(global_path_point)
                    if local_path_point[0] > 0:
                        dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                        if dis >= self.lfd:
                            self.forward_point = path_point
                            self.is_look_forward_point = True
                            break

                theta = atan2(local_path_point[1], local_path_point[0])
                default_vel = 10
                default_vel_m1m51 = 10
                if self.is_look_forward_point:
                    if self.mission_info.mission_num == 4:
                        #############################
                        if self.mission4_forward: # 직진하면서 주차 공간 찾기
                            end_point = [0, 109] # 주차 공간 끝 지점
                            self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                            self.ctrl_cmd_msg.velocity = default_vel
                            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                            distance_1 = sqrt((vehicle_position.x - end_point[0])**2 + (vehicle_position.y - end_point[1])**2)
                            if distance_1 < 3.0: # 끝 지점이랑 현재 차량 위치 거리가 3.0 미만이 되면
                                self.stop_vehicle() # 정지
                                self.mission4_forward = False # 직진하면서 주차 공간 찾는거 중지
                                self.mission4_reverse = True # 다시 주차 시작 위치로 복귀
                        elif self.mission4_reverse: # 주차 시작 위치로 복귀
                            self.stop_vehicle()
                            self.reverse_mode() # 리버스 모드로
                            self.find_parking_zone = False # 리버스 모드로 시작 위치로 돌아가게 되면 주차 공간 찾아서 위치로 가기
                        elif not self.find_parking_zone:
                            parking_zone = [-34.5, 109] # 주차 공간 예시, 처음에 주차 공간 찾은 구역이 입력으로 들어감
                            distance_2 = sqrt((vehicle_position.x - parking_zone[0])**2 + (vehicle_position.y - parking_zone[1])**2)
                            if distance_2 < 3.0:
                                self.stop_vehicle()
                                self.find_parking_zone = True
                            else:
                                self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                                normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                                self.ctrl_cmd_msg.velocity = default_vel
                        elif not self.parking_complete:
                            self.parking_mode()
                        else:
                            self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                            normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                            self.ctrl_cmd_msg.velocity = default_vel                    
                    else:
                        # steering
                        self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                        normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981

                        # velocity
                        if self.mission_info.mission_num == 71:
                            rospy.loginfo_once("Mission 7: Traffic light detection active")
                            
                            if not self.M7_complete:
                                if not self.is_stopped:
                                    rospy.loginfo_once("Vehicle is not stopped. Initiating stop.")
                                    self.stop_vehicle()
                                else:
                                    # 초록불에만 출발
                                    if self.traffic_color == 3:
                                        rospy.loginfo_once("Traffic light is green. Traffic light color: {}".format(self.traffic_color))
                                        rospy.loginfo_once("Resuming vehicle. Calculating velocity.")
                                        self.ctrl_cmd_msg.velocity = default_vel * (1.0 - (self.obstacle.collision_probability / 100)) * (1 - 0.5 * normalized_steer)
                                        self.is_stopped = False
                                        self.M7_complete = True
                                    else:
                                        rospy.loginfo_once("Traffic light is not green. Waiting...")
                                        self.ctrl_cmd_msg.velocity = 0
                            else:
                                rospy.loginfo_once("Mission 7 complete. Continuing with calculated velocity.")
                                self.ctrl_cmd_msg.velocity = default_vel * (1.0 - (self.obstacle.collision_probability / 100)) * (1 - 0.5 * normalized_steer)

                        elif self.mission_info.mission_num == 8:
                            rospy.loginfo_once("Mission 8: End")
                            self.M7_complete = False  # 신호등 미션 디버깅용
                            if self.is_finish:
                                self.stop_vehicle()
                                self.send_gear_cmd(Gear.P.value)
                            else:
                                self.ctrl_cmd_msg.velocity = default_vel

                        elif self.mission_info.mission_num == 3:
                            rospy.loginfo_once("Mission 3: Slam")
                            
                            if self.mission_info.count == 1:
                                self.stop_vehicle_slam()
                            else:
                                self.ctrl_cmd_msg.velocity = default_vel * (1 - 0.6 * normalized_steer)
                                self.is_stopped = False

                        elif self.mission_info.mission_num == 6:
                            rospy.loginfo_once("Mission 6: Slam")
                            
                            if self.mission_info.count == 1:
                                self.stop_vehicle_slam()
                            else:
                                self.ctrl_cmd_msg.velocity = default_vel
                                self.is_stopped = False

                        elif self.mission_info.mission_num == 1 or  self.mission_info.mission_num == 51:
                            self.ctrl_cmd_msg.velocity = default_vel_m1m51 * (1.0 - (self.obstacle.collision_probability / 100)) * (1 - 0.6 * normalized_steer)
                        else:
                            self.ctrl_cmd_msg.velocity = default_vel

                else:
                    rospy.loginfo_once("No forward point found")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 0.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            self.is_path = self.is_odom = False
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def yolo_callback(self, msg):
        self.is_yolo = True
        self.obstacle = msg

    def mission_callback(self, msg):
        self.mission_info = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def traffic_callback(self, msg):
        self.traffic_color = msg.data

    def finish_callback(self, msg):
        self.is_finish = msg.data

    def send_gear_cmd(self, gear_mode):
        try:
            gear_cmd = EventInfo()
            gear_cmd.option = 3
            gear_cmd.ctrl_mode = 3
            gear_cmd.gear = gear_mode

            response = self.event_cmd_srv(gear_cmd)

            if response:
                rospy.loginfo(f"Gear successfully changed to {gear_mode}")
                rospy.sleep(1)  # 기어 변경 후 안정화 시간 추가
                return True
            else:
                rospy.logerr(f"Failed to change gear to {gear_mode}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def stop_vehicle(self):
        self.ctrl_cmd_msg.velocity = 0.0
        self.ctrl_cmd_msg.steering = 0.0  # 조향 각도를 0으로 설정
        self.ctrl_cmd_msg.brake = 1.0  # 최대 제동력
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
        rospy.sleep(3)
        rospy.loginfo("Vehicle stopped")
        self.is_stopped = True
        self.stop_pub.publish(self.is_stopped)
        rospy.sleep(3)
        self.ctrl_cmd_msg.brake = 0

    def stop_vehicle_slam(self):
        self.ctrl_cmd_msg.velocity = 0.0
        self.ctrl_cmd_msg.steering = 0.0  # 조향 각도를 0으로 설정
        self.ctrl_cmd_msg.brake = 1.0  # 최대 제동력
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
        rospy.sleep(6)
        rospy.loginfo("Vehicle stopped")
        self.is_stopped = True
        self.stop_pub.publish(self.is_stopped)
        rospy.sleep(6)
        self.ctrl_cmd_msg.brake = 0


    def parking_mode(self):
        rospy.loginfo("Starting parking maneuver")
        # 전진: 기어 D로 변경 후 5초 동안 전진
        if self.send_gear_cmd(Gear.D.value):
            self.set_velocity_and_steering(5.0, 40, 6)
            self.stop_vehicle()

        rospy.sleep(2)  # 명령 후 추가 대기

        # 후진: 기어 R로 변경 후 1초 동안 조향 각도 -40도
        if self.send_gear_cmd(Gear.R.value):
            rospy.sleep(1)
            self.set_velocity_and_steering(5.0, -40, 0.5)
            self.stop_vehicle()

        rospy.sleep(2)  # 명령 후 추가 대기

        # 후진: 기어 R 상태에서 6초 동안 조향 각도 0도로 후진
        self.set_velocity_and_steering(5.0, 0, 7)
        self.stop_vehicle()

        rospy.sleep(2)  # 명령 후 추가 대기

        # 정지: 기어 P로 변경
        if self.send_gear_cmd(Gear.P.value):
            self.stop_vehicle()

        rospy.sleep(2)

        if self.send_gear_cmd(Gear.D.value):
            self.set_velocity_and_steering(5.0, 0, 2)
            self.stop_vehicle()

        rospy.sleep(2)

        if self.send_gear_cmd(Gear.D.value):
            self.set_velocity_and_steering(5.0, -40, 5.5)
            self.stop_vehicle()

        rospy.loginfo("Parking maneuver completed")
        self.parking_complete = True  # 주차 완료 상태 설정

    def reverse_mode(self):
        rospy.loginfo("reverse path")
        if self.send_gear_cmd(Gear.R.value):
            rospy.sleep(1)
            self.set_velocity_and_steering(10.0, 0.0, 30)
            self.stop_vehicle() 
        
        rospy.sleep(2)

        if self.send_gear_cmd(Gear.D.value):
            self.stop_vehicle()

        self.mission4_reverse = False

    def set_velocity_and_steering(self, velocity, steering_angle_deg, duration):
        """
        속도와 조향 각도를 설정하고 일정 시간 동안 명령을 발행
        """
        rate = rospy.Rate(10)  # 10Hz 주기
        end_time = rospy.Time.now() + rospy.Duration(duration)

        while rospy.Time.now() < end_time:
            self.ctrl_cmd_msg.steering = steering_angle_deg * pi / 180  # 조향 각도를 라디안으로 변환
            self.ctrl_cmd_msg.velocity = velocity
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()

        rospy.loginfo(f"Completed velocity {velocity} and steering {steering_angle_deg} degrees for {duration} seconds")

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
