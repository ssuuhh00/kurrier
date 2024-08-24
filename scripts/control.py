#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,sqrt,pow,atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd, EventInfo
from std_msgs.msg import Int16, Bool
import numpy as np
from tf.transformations import euler_from_quaternion
from kurrier.msg import mission, obstacle   # 사용자 정의 메시지 임포트 
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvRequest

class pure_pursuit :
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_info", obstacle, self.yolo_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)
        rospy.Subscriber("traffic_light_color", Int16, self.traffic_callback)
        rospy.Subscriber("/check_finish", Bool, self.parking_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=10)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        self.is_odom=False
        self.is_yolo=False

        self.traffic_light_color = ""
        self.obstacle = obstacle()
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.vehicle_length=3
        self.lfd=5

        self.is_waiting_time = False
        self.count = 0
        self.mission_info = mission()

        self.doing_parking = False
        self.check_finish = False

        # MoraiEventCmdSrv 서비스 클라이언트 (서비스가 사용 가능할 때까지 대기)
        rospy.loginfo("Waiting for /Service_MoraiEventCmd service...")
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_client = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        rospy.loginfo("/Service_MoraiEventCmd service is now available.")

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_odom==True  :
                
                vehicle_position=self.current_postion
                self.is_look_forward_point= False

                translation=[vehicle_position.x, vehicle_position.y]

                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])

                det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])

                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position

                    global_path_point=[path_point.x,path_point.y,1]
                    local_path_point=det_t.dot(global_path_point)           
                    if local_path_point[0]>0 :
                        dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                        if dis>= self.lfd :
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            break
                
                theta=atan2(local_path_point[1],local_path_point[0])
                default_vel = 15

                if (self.mission_info.mission_num == 8) and (self.check_finish):
                    if not self.doing_parking:
                        self.ctrl_cmd_msg.velocity = 0
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                        # 3초 후에 이벤트 명령을 보내기 위해 대기
                        rospy.sleep(3)

                        # MoraiEventCmdSrv 서비스 요청 생성
                        event_cmd_request = MoraiEventCmdSrvRequest()
                        
                        # EventInfo 메시지 설정
                        event_info = EventInfo()
                        event_info.option = 2  # 기어 변경 옵션
                        event_info.ctrl_mode = 3  # 자동 주행 모드 유지
                        event_info.gear = 1  # 기어를 P로 설정

                        # 서비스 요청에 EventInfo 메시지 설정
                        event_cmd_request.request = event_info

                        # 서비스 호출
                        try:
                            rospy.loginfo("Sending MoraiEventCmdSrv request with option=2, ctrl_mode=3, gear=1")
                            response = self.event_cmd_client(event_cmd_request)
                            rospy.loginfo(f"Received response: {response}")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"Service call failed: {e}")

                        self.doing_parking = True

                elif self.is_look_forward_point :

                    self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)  
                    normalized_steer = abs(self.ctrl_cmd_msg.steering)/0.6981          
                    
                    if self.mission_info.mission_num == 7:
                        
                        if  self.traffic_light_color==0:
                            self.ctrl_cmd_msg.velocity = 0.7 * default_vel*(1.0-(self.obstacle.collision_probability/100))*(1.0-(self.obstacle.collision_probability/100))*(1-0.6*normalized_steer)
                        elif self.traffic_light_color==1 or self.traffic_light_color==2:
                            self.ctrl_cmd_msg.velocity = 0
                           
                        elif self.traffic_light_color==3:
                            self.ctrl_cmd_msg.velocity = default_vel*(1.0-(self.obstacle.collision_probability/100))*(1.0-(self.obstacle.collision_probability/100))*(1-0.6*normalized_steer)
                    else :
                            self.ctrl_cmd_msg.velocity = default_vel*(1.0-(self.obstacle.collision_probability/100))*(1.0-(self.obstacle.collision_probability/100))*(1-0.7*normalized_steer)
                        
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0
                    self.ctrl_cmd_msg.velocity=0.0
                
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            self.is_path = self.is_odom = False
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg      
        
    def yolo_callback(self,msg):
        self.is_yolo = True
        self.obstacle = msg

    def mission_callback(self,msg):
        self.mission_info = msg

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def traffic_callback(self, msg):
        self.traffic_color = msg 

    def parking_callback(self, msg):
        self.check_finish = msg 
    
if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
