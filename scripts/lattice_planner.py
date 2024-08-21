#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, sqrt, pow, atan2, radians
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from kurrier.msg import mission  # 사용자 정의 메시지 임포트

class LatticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        # Subscriber, Publisher 선언
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cluster_points", PointCloud2, self.object_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)

        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)

        self.is_path = False
        self.is_obj = False
        self.is_lattice_started = False
        self.local_path = None
        self.is_odom=False

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_lattice_started:
                if self.is_path and self.is_odom and self.is_obj:
                    if self.checkObject(self.local_path, self.object_points):
                        lattice_path = self.latticePlanner(self.local_path, self.odom_msg)
                        lattice_path_index = self.collision_check(self.object_points, lattice_path)
                        self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                    else:
                        self.lattice_path_pub.publish(self.local_path)
                else:
                    #rospy.logwarn("Not enough information to compute lattice path.")
                    if self.local_path is not None:
                        self.lattice_path_pub.publish(self.local_path)
            else:
                if self.local_path is not None:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def mission_callback(self, msg):
        # gps음영 미션 시작
        if msg.mission_num == 2 and not self.is_lattice_started:
            self.is_lattice_started = True
        # gps음영 미션 끝
        elif msg.mission_num != 2 and self.is_lattice_started:
            self.is_lattice_started = False

    def checkObject(self, ref_path, object_points):
        is_crash = False
        for point in object_points:
            for path in ref_path.poses:
                dis = sqrt(pow(path.pose.position.x - point[0], 2) + pow(path.pose.position.y - point[1], 2))
                if dis < 2.3:  # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일 때 충돌이라 판단
                    # dis < 2.35 에서 2.35보다 크게 설정하면 경로와 장애물 사이의 거리를 더 멀게 설정
                    is_crash = True
                    break
        return is_crash

    def collision_check(self, object_points, out_path):
        selected_lane = -1
        
        lane_weight = [3, 2, 1, 1, 2, 3]  # 원본

        for point in object_points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point[0] - path_pos.pose.position.x, 2) + pow(point[1] - path_pos.pose.position.y, 2))
                    if dis < 1.5:  # 1.5보다 크게 설정을 하게 되면 장애물과의 거리가 더 큰 경로를 선택할 수 있게
                        lane_weight[path_num] += 100

        selected_lane = lane_weight.index(min(lane_weight))
        return selected_lane

    def path_callback(self, msg):
        self.is_path = True
        self.local_path = msg
        rospy.loginfo("Local path received and stored. is_path = True")

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        rospy.loginfo("Vehicle status received and stored. is_odom = True")

    def object_callback(self, msg):
        if not self.is_odom:
            rospy.logwarn("Vehicle status not received yet, skipping object processing.")
            return

        self.is_obj = True
        self.object_points = []

        # 클러스터링된 장애물 포인트들을 추출하여 절대 좌표로 변환하여 저장합니다.
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            abs_x, abs_y, abs_z = self.relative_to_absolute(point[0], point[1], point[2])
            self.object_points.append([abs_x, abs_y, abs_z])
        
        rospy.loginfo(f"Received and stored {len(self.object_points)} obstacle points. is_obj = True")
        if len(self.object_points) > 0:
            rospy.loginfo(f"First obstacle point (absolute): x={self.object_points[0][0]}, y={self.object_points[0][1]}, z={self.object_points[0][2]}")

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
        abs_x = ego_x + cos(heading) * rel_x - sin(heading) * rel_y
        abs_y = ego_y + sin(heading) * rel_x + cos(heading) * rel_y
        abs_z = ego_z + rel_z  # z는 일반적으로 사용되지 않지만 포함

        rospy.loginfo(f"Converted relative position ({rel_x}, {rel_y}, {rel_z}) to absolute position ({abs_x}, {abs_y}, {abs_z}) with heading {heading}")
        return abs_x, abs_y, abs_z

    def latticePlanner(self, ref_path, vehicle_status):
        out_path = []

        vehicle_velocity = max(vehicle_status.twist.twist.linear.x * 3.6, 20)  # 정지 시에도 기본 속도(20)를 사용하여 look_distance 계산
        look_distance = int(vehicle_velocity * 0.2 * 2)

        # 오리엔테이션에서 헤딩 방향을 계산
        orientation = vehicle_status.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        # 시작점과 끝점을 차량의 현재 위치 기준으로 정함
        start_pos = {'x': vehicle_status.pose.pose.position.x, 'y': vehicle_status.pose.pose.position.y}
        end_pos = {'x': vehicle_status.pose.pose.position.x + look_distance * cos(heading),
                   'y': vehicle_status.pose.pose.position.y + look_distance * sin(heading)}

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
        lane_off_set = [-1.6, -1.2, -1.0, 1.0, 1.2, 1.6]

        local_lattice_points = []

        for i in range(len(lane_off_set)):
            local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])

        for end_point in local_lattice_points:
            lattice_path = Path()
            lattice_path.header.frame_id = 'map'
            x = []
            y = []
            x_interval = 0.5
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
            a[2] = 3.0 * (pf - ps) / (xf * xf)
            a[3] = -2.0 * (pf - ps) / (xf * xf * xf)

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


if __name__ == '__main__':
    try:
        LatticePlanner()
    except rospy.ROSInterruptException:
        pass
