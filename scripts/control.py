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
from math import radians
from geometry_msgs.msg import Quaternion
import tf
import sensor_msgs.point_cloud2 as pc2
from math import cos, sin, sqrt, pow, atan2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

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
        rospy.Subscriber("/parking_point", Int32MultiArray, self.parking_callback)

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


        rospy.Subscriber("/liorf/mapping/odometry", Odometry, self.slam_imu_callback)
        # slam 상태비교를 위한 초기화
        self.slam_status = True

        self.slam_yaw = Odometry()
        self.slam_vehicle_yaw = None
        self.last_odom_status=Point()
        self.last_odom_status.x = None
        self.last_odom_status.y = None
        self.odom_threshold = 1
        self.ang_threshold = radians(15)   

        # 슬램 펑
        self.is_1st_lattice_started = False
        self.is_2nd_lattice_started = False

        self.is_1st_slam_started = False
        self.is_2nd_slam_started = False


        self.is_obj = False

        base_offset2 = 0.7 * 0.3 * 20  # 인덱스 1당 30cm의 증가율 적용

        self.lane_weight = [ 4, 3, 2, 1, 1, 2, 3, 4 ]

        offset_steps2 = 10
        step_size2 = base_offset2 * 2 / offset_steps2

        self.lane_off_set2 = [
            -1*base_offset2,
            -1*base_offset2 + step_size2 * 2.0,
            -1*base_offset2 + step_size2 * 3.7,
            -1*base_offset2 + step_size2 * 4.9,
            base_offset2 - (step_size2 * 4.9),
            base_offset2 - (step_size2 * 3.7),
            base_offset2 - (step_size2 * 2.0),
            base_offset2,
        ]

        # slam 상태비교를 위한 초기화
        self.last_odom_status = Point()
        self.last_yaw_status = 0
        self.odom_threshold = 1
        self.ang_threshold = radians(15)
        
        #self.find_parking_zone = False
        self.parking_complete = False

        self.mission4_forward = True
        self.mission4_reverse = False
        self.find_parking_zone = True

        self.empty_zone = None
        self.parking_index = 0

        self.candidate_parking_zone = [ # 주차 공간 후보
            (0, 0),
            (-59.8301, 106.8783), #(-60.0301, 106.8783)
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
            (4.29672, 106.9429),
            (6.40897, 106.8754)
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
                default_vel_m1 = 15

                if (self.is_1st_slam_started):
                    if self.slam_check():
                        self.local_path = self.generate_local_path(self.odom_msg)
                        if self.is_obj:
                            if self.checkObject2(self.local_path, self.object_points):
                                lattice_path = self.latticePlanner2(self.local_path, self.odom_msg)
                                lattice_path_index = self.collision_check2(self.object_points, lattice_path)
                                self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                            else:
                                self.lattice_path_pub.publish(self.local_path)

                        # path의 절반 지점 찾기
                        mid_index = len(self.path.poses) // 2
                        mid_pose = self.path.poses[mid_index].pose.position

                        # local_path_point 설정
                        local_path_point = [mid_pose.x, mid_pose.y]

                        theta = atan2(local_path_point[1], local_path_point[0])
                        
                        default_vel = 4

                        # steering
                        self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                        normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                        self.ctrl_cmd_msg.velocity = default_vel * (1 - 0.7 * normalized_steer)
                    else:
                        self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                        normalized_steer = abs(self.ctrl_cmd_msg.steering) / 0.6981
                        if self.mission_info.mission_num == 3:
                            rospy.loginfo_once("Mission 3: Slam")
                            
                            if self.mission_info.count == 1:
                                self.stop_vehicle_slam()
                            else:
                                self.ctrl_cmd_msg.velocity = default_vel * (1 - 0.6 * normalized_steer)
                                self.is_stopped = False
                else:
                    if self.is_look_forward_point:
                        if self.mission_info.mission_num == 4:
                            #############################
                            if self.mission4_forward: # 직진하면서 주차 공간 찾기
                                # end_point = [3, 109] # 주차 공간 끝 지점 right path
                                end_point = [3, 111] # center path
                                self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)
                                self.ctrl_cmd_msg.velocity = default_vel
                                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                                distance_1 = sqrt((vehicle_position.x - end_point[0])**2 + (vehicle_position.y - end_point[1])**2)
                                if distance_1 < 2.0: # 끝 지점이랑 현재 차량 위치 거리가 3.0 미만이 되면
                                    self.stop_vehicle() # 정지
                                    self.mission4_forward = False # 직진하면서 주차 공간 찾는거 중지
                                    self.mission4_reverse = True # 다시 주차 시작 위치로 복귀
                            elif self.mission4_reverse: # 주차 시작 위치로 복귀
                                self.stop_vehicle()
                                self.reverse_mode() # 리버스 모드로
                                self.find_parking_zone = False # 리버스 모드로 시작 위치로 돌아가게 되면 주차 공간 찾아서 위치로 가기
                            elif not self.find_parking_zone:
                                parking_zone_find = self.candidate_parking_zone[self.parking_index]
                                #parking_zone = [parking_zone_find[0] - 4.5, 109] # right path
                                parking_zone = [parking_zone_find[0] - 4.5, 111] # center path

                                #parking_zone_find = self.candidate_parking_zone[29]
                                #parking_zone = [parking_zone_find[0] - 4.5, 111]
                                rospy.loginfo("go parking point : {}".format(parking_zone))
                                distance_2 = sqrt((vehicle_position.x - parking_zone[0])**2 + (vehicle_position.y - parking_zone[1])**2)
                                if distance_2 < 2.5:
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

                            elif self.mission_info.mission_num == 6:
                                rospy.loginfo_once("Mission 6: Slam")
                                
                                if self.mission_info.count == 1:
                                    self.stop_vehicle_slam()
                                else:
                                    self.ctrl_cmd_msg.velocity = default_vel
                                    self.is_stopped = False

                            elif self.mission_info.mission_num == 1:
                                self.ctrl_cmd_msg.velocity = default_vel_m1 * (1.0 - (self.obstacle.collision_probability / 100)) * (1 - 0.6 * normalized_steer)
                            else:
                                self.ctrl_cmd_msg.velocity = default_vel* (1 - 0.6 * normalized_steer)

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

        if self.mission_info.mission_num == 3 and not self.is_1st_slam_started:
            self.is_1st_slam_started = True

        elif self.mission_info.mission_num != 3 and self.is_1st_slam_started:
            self.is_1st_slam_started = False

        elif self.mission_info.mission_num == 6 and not self.is_2nd_slam_started:
            self.is_2nd_slam_started = True

        elif self.mission_info.mission_num != 6 and self.is_2nd_slam_started:
            self.is_2nd_slam_started = False

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y
        self.odom_msg = msg


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

    def parking_callback(self, msg):
        # Extract the data from the message
        self.empty_zone = msg.data
        self.parking_index = self.empty_zone[-1]

#        if (self.empty_zone[-1] == len(self.empty_zone) - 1):
#            self.parking_index = self.empty_zone[1]
#        else:
#            self.parking_index = self.empty_zone[-1] # ori

            #self.parking_index = self.empty_zone[1]

    def parking_mode(self):
        rospy.loginfo("Starting parking maneuver")
        # 전진: 기어 D로 변경 후 5초 동안 전진
        if self.send_gear_cmd(Gear.D.value):
            #self.set_velocity_and_steering(5.0, 40, 5) #6 right path
            self.set_velocity_and_steering(5.0, 40, 4.5) # center path
            self.stop_vehicle()

        rospy.sleep(1)  # 명령 후 추가 대기

        # 후진: 기어 R로 변경 후 1초 동안 조향 각도 -40도
        if self.send_gear_cmd(Gear.R.value):
            rospy.sleep(1)
            #self.set_velocity_and_steering(5.0, -40, 0.5) #0.8 right path
            self.set_velocity_and_steering(5.0, -40, 1.1)
            self.stop_vehicle()

        rospy.sleep(1)  # 명령 후 추가 대기

        # 후진: 기어 R 상태에서 6초 동안 조향 각도 0도로 후진
        self.set_velocity_and_steering(5.0, 0, 6.7) #7
        self.stop_vehicle()

        rospy.sleep(1)  # 명령 후 추가 대기

        # 정지: 기어 P로 변경
        if self.send_gear_cmd(Gear.P.value):
            self.stop_vehicle()

        rospy.sleep(1)

        if self.send_gear_cmd(Gear.D.value):
            self.set_velocity_and_steering(5.0, 0, 2)
            self.stop_vehicle()

        rospy.sleep(1)

        if self.send_gear_cmd(Gear.D.value):
            self.set_velocity_and_steering(5.0, -40, 5)
            self.stop_vehicle()

        rospy.sleep(1)

        if self.send_gear_cmd(Gear.R.value):
            self.set_velocity_and_steering(5.0, 0, 5)
            self.stop_vehicle()

        rospy.sleep(1)

        if self.send_gear_cmd(Gear.D.value):
            self.set_velocity_and_steering(5.0, 0, 0.5)
            self.stop_vehicle()

        rospy.loginfo("Parking maneuver completed")
        self.parking_complete = True  # 주차 완료 상태 설정

    def reverse_mode(self):
        rospy.loginfo("reverse path")
        if self.send_gear_cmd(Gear.R.value):
            rospy.sleep(1)
            self.set_velocity_and_steering(10.0, 0.0, 28)
            self.stop_vehicle() 
        
        rospy.sleep(1)

        if self.send_gear_cmd(Gear.D.value):
            self.stop_vehicle()

        self.mission4_reverse = False

    def slam_imu_callback(self, data):
        if data.pose.pose.orientation.w == 0:
            self.slam_yaw.pose.pose.orientation.x = 0.0
            self.slam_yaw.pose.pose.orientation.y = 0.0
            self.slam_yaw.pose.pose.orientation.z = 0.0
            self.slam_yaw.pose.pose.orientation.w = 1.0
        else:
            self.slam_yaw.pose.pose.orientation.x = data.pose.pose.orientation.x
            self.slam_yaw.pose.pose.orientation.y = data.pose.pose.orientation.y
            self.slam_yaw.pose.pose.orientation.z = data.pose.pose.orientation.z
            self.slam_yaw.pose.pose.orientation.w = data.pose.pose.orientation.w

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

    def slam_check(self):
        if not self.slam_status:
            rospy.loginfo("슬램 고장남")
            return self.slam_status
        else:
            rospy.loginfo("슬램 정상")
            if self.current_postion.x is None or self.current_postion.y is None or self.slam_yaw is None:
                self.slam_status = True
                return True
            else:
                if self.last_odom_status is None or self.last_yaw_status is None:
                    self.last_odom_status = Point(self.current_postion.x, self.current_postion.y, 0.0)
                    self.slam_status = True
                    return True
                else:
                    slam_odom_quaternion = (self.slam_yaw.pose.pose.orientation.x, self.slam_yaw.pose.pose.orientation.y, self.slam_yaw.pose.pose.orientation.z, self.slam_yaw.pose.pose.orientation.w)
                    _, _, self.slam_vehicle_yaw = euler_from_quaternion(slam_odom_quaternion)
                    # rospy.loginfo("slam_vehicle_yaw = ", self.slam_vehicle_yaw)
                    self.ang_diff = abs(self.vehicle_yaw - self.slam_vehicle_yaw)
                    self.odom_diff = sqrt(pow(self.current_postion.x - self.last_odom_status.x, 2) + pow(self.current_postion.y - self.last_odom_status.y, 2))
                    
                    if self.odom_diff > self.odom_threshold:
                        rospy.logwarn(f"Odometry difference too large: {self.odom_diff} > {self.odom_threshold}")
                        self.slam_status = False
                        return self.slam_status
                    if self.ang_diff > self.ang_threshold:
                        rospy.logwarn(f"angle difference too large: {self.ang_diff} > {self.ang_threshold}")
                        self.slam_status = False
                        return self.slam_status
                    self.slam_status = True
                    self.last_odom_status.x = self.current_postion.x
                    self.last_odom_status.y = self.current_postion.y
                    return self.slam_status


    def latticePlanner2(self, ref_path, vehicle_status):
        out_path = []

        # vehicle_velocity = max(vehicle_status.twist.twist.linear.x * 3.6, 20)  # 정지 시에도 기본 속도(20)를 사용하여 look_distance 계산
        # look_distance = int(vehicle_velocity * 0.2 * 2)

        # 오리엔테이션에서 헤딩 방향을 계산
        orientation = vehicle_status.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))
        wheel_base = -1  # 차량의 wheel base (m)
     
        # 시작점과 끝점을 차량의 현재 위치 기준으로 정함
        start_pos = {'x': vehicle_status.pose.pose.position.x  , 'y': vehicle_status.pose.pose.position.y }

        # self.local_path의 마지막 점을 end_pos에 저장하는 코드
        if self.local_path and len(self.local_path.poses) > 0:
            last_pose = self.local_path.poses[18]
            end_pos = {'x': last_pose.pose.position.x, 'y': last_pose.pose.position.y}
            #rospy.loginfo(f"End position set to x: {end_pos['x']}, y: {end_pos['y']}")
        else:
            #rospy.logwarn("local_path is empty or not set.")
            end_pos = None

        theta = atan2(end_pos['y'] - start_pos['y'], end_pos['x'] - start_pos['x'])
        translation = [start_pos['x'], start_pos['y']]

        trans_matrix = np.array([[cos(theta), -sin(theta), translation[0]],
                                 [sin(theta), cos(theta), translation[1]],
                                 [0, 0, 1]])

        det_trans_matrix = np.array([[trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])],
                                     [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                     [0, 0, 1]])

        world_end_point = np.array([[end_pos['x']], [end_pos['y']], [1]])
        local_end_point = det_trans_matrix.dot(world_end_point)
        world_ego_vehicle_position = np.array([[vehicle_status.pose.pose.position.x], [vehicle_status.pose.pose.position.y], [1]])
        local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
        # lane_off_set2 = [-2.0, -1.5, -1.0, 1.0, 1.5, 2.0]

        local_lattice_points = []

        for i in range(len(self.lane_off_set2)):
            local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + self.lane_off_set[i], 1])

        for end_point in local_lattice_points:
            lattice_path = Path()
            lattice_path.header.frame_id = 'map'
            x = []
            y = []
            x_interval = 0.15
            xs = 0
            xf = end_point[0]
            ps = local_ego_vehicle_position[1][0]
            pf = end_point[1]
            x_num = xf / x_interval

            for i in range(xs, int(x_num)):
                x.append(i * x_interval)

            a = [0.0, 0.0, 0.0, 0.0]
            a[0] = ps
            a[1] = 0
            a[2] = 4.7 * (pf - ps) / (xf * xf)
            a[3] = -3.7 * (pf - ps) / (xf * xf * xf)

            for i in x:
                result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                y.append(result)

            for i in range(0, len(y)):
                local_result = np.array([[x[i]], [y[i]], [1]])
                global_result = trans_matrix.dot(local_result)

                read_pose = PoseStamped()
                read_pose.pose.position.x = global_result[0][0]
                read_pose.pose.position.y = global_result[1][0]
                read_pose.pose.position.z = 0
                read_pose.pose.orientation.x = 0
                read_pose.pose.orientation.y = 0
                read_pose.pose.orientation.z = 0
                read_pose.pose.orientation.w = 1
                lattice_path.poses.append(read_pose)

            out_path.append(lattice_path)
                       
        return out_path
    


    def checkObject2(self, ref_path, object_points):
        is_crash = False
        for point in object_points:
            for path in ref_path.poses:
                dis = sqrt(pow(path.pose.position.x - point[0], 2) + pow(path.pose.position.y - point[1], 2))
                if dis < 1.7:  # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일 때 충돌이라 판단
                    # dis < 2.35 에서 2.35보다 크게 설정하면 경로와 장애물 사이의 거리를 더 멀게 설정
                    is_crash = True
                    break
        return is_crash

    def generate_local_path(self, vehicle_status):
        """
        Generates a local path starting from (0, 0, 0) and extending straight along the x-axis.
        The path extends for 20 points with 0.3 meters between each point.
        """
        path = Path()
        path.header.frame_id = 'map'

        # The number of points (steps) in the path
        num_points = 30

        # Interval between points in meters
        interval = 0.3

        # Generate points along the x-axis
        for i in range(num_points):
            x = i * interval
            y = 0.0  # No deviation in y, so it stays on the x-axis
            z = 0.0  # Assuming a flat 2D plane

            # Create a PoseStamped for each point
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            # Set the orientation to face straight along the x-axis
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # No rotation, aligned with x-axis
            pose.pose.orientation = Quaternion(*quaternion)

            # Add the pose to the path
            path.poses.append(pose)

        return path
    
    def collision_check2(self, object_points, out_path):
        selected_lane = 12
        self.lane_weight = [ 4, 3, 2, 1, 1, 2, 3, 4 ]
        max_weight = 5000
        for point in object_points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point[0] - path_pos.pose.position.x, 2) + pow(point[1] - path_pos.pose.position.y, 2))
                    
                    if  2.5 < dis < 3.0:
                        self.lane_weight[path_num] += 1
                    elif 2.2 < dis < 2.5 :
                        self.lane_weight[path_num] += 3
                    elif  1.9 < dis < 2.2:
                        self.lane_weight[path_num] += 5
                    elif  1.7 < dis < 1.9:
                        self.lane_weight[path_num] += 8
                    elif  1.5 < dis < 1.7:
                        self.lane_weight[path_num] += 12
                    elif  1.3 < dis < 1.5:
                        self.lane_weight[path_num] += 17
                    elif  1.1 < dis < 1.3:
                        self.lane_weight[path_num] += 23
                    elif  0.9 < dis < 1.1:
                        self.lane_weight[path_num] += 29
                    elif   dis < 0.9 :
                        self.lane_weight[path_num] += 50
                    else: 
                        self.lane_weight[path_num] -= 1
                    self.lane_weight[path_num] = min(max_weight, self.lane_weight[path_num])

        for i, weight in enumerate(self.lane_weight):
            rospy.loginfo(f"Lane {i} weight: {weight}")        
        selected_lane = self.lane_weight.index(min(self.lane_weight))
        return selected_lane


    def object_callback(self, msg):
        if not self.is_odom:
            rospy.logwarn("odom not received yet, skipping object processing.")
            return

        self.is_obj = True
        self.object_points = []
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y
        min_distance = float('inf')  # 초기화: 아주 큰 값으로 설정
        # 클러스터링된 장애물 포인트들을 추출하여 절대 좌표로 변환하여 저장합니다.
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            abs_x, abs_y, abs_z = self.relative_to_absolute(point[0], point[1], point[2])
            self.object_points.append([abs_x, abs_y, abs_z])
            
            distance = sqrt(pow(abs_x - ego_x, 2) + pow(abs_y - ego_y, 2))
            if distance < min_distance:
                min_distance = distance
        rospy.loginfo(f"장애물과 차량 사이의 최소 거리: {min_distance}m")
        #rospy.loginfo(f"Received and stored {len(self.object_points)} obstacle points. is_obj = True")
        #if len(self.object_points) > 0:
            #rospy.loginfo(f"First obstacle point (absolute): x={self.object_points[0][0]}, y={self.object_points[0][1]}, z={self.object_points[0][2]}")
    
if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
