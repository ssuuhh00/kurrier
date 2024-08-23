#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import String, Int16
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from kurrier.msg import mission, obstacle  # 사용자 정의 메시지 임포트 

class pure_pursuit :
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/obstacle_info", obstacle, self.yolo_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)
        rospy.Subscriber("traffic_light_color", Int16, self.traffic_callback)
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
                default_vel = 18
                if self.is_look_forward_point :
                    if self.is_waiting_time:               
                        self.ctrl_cmd_msg.velocity = 0
                    else:
                        self.ctrl_cmd_msg.steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)  
                        normalized_steer = abs(self.ctrl_cmd_msg.steering)/0.6981          
                        self.ctrl_cmd_msg.velocity = default_vel*(1.0-(self.obstacle.collision_probability/100))*(1.0-(self.obstacle.collision_probability/100))*(1-0.7*normalized_steer)
                        
                        if self.mission_info.mission_num == 7 and (self.traffic_light_color==0):
                            self.ctrl_cmd_msg.velocity = default_vel*(1.0-(self.obstacle.collision_probability/100))*(1-0.6*normalized_steer)
                        elif self.mission_info.mission_num == 7 and (self.traffic_light_color==1 or self.traffic_light_color==2):
                            self.ctrl_cmd_msg.velocity = 0.0
                        elif self.mission_info.mission_num == 7 and self.traffic_light_color==3:
                            self.ctrl_cmd_msg.velocity = default_vel*(1.0-(self.obstacle.collision_probability/100))*(1.0-(self.obstacle.collision_probability/100))*(1-0.6*normalized_steer)
        
                        os.system('clear')
                        print("-------------------------------------")
                        print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180/pi)
                        print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
                        print("-------------------------------------")
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0
                    self.ctrl_cmd_msg.velocity=0.0
                
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            else:
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")
            
            self.is_path = self.is_odom = False
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg      
        
    def yolo_callback(self,msg):
        self.is_yolo = True
        self.obstacle = msg

    def mission_callback(self,msg):
        if self.mission_info.mission_num != msg.mission_num:
            if self.count < 45: #15hz 이니까 3초 대기
                self.is_waiting_time = True
                self.count += 1
            else:
                self.count = 0
                self.is_waiting_time = False
                self.mission_info.mission_num = msg.mission_num
        else:
            self.mission_info = msg

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y
    def traffic_callback(self, msg):
        self.traffic_color = msg 
    
if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
