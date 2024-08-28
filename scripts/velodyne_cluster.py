#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from sklearn.cluster import DBSCAN

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.clusterpoints_pub = rospy.Publisher("/cluster_points", PointCloud2, queue_size=10)
        self.pc_np = None

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return

        # 거리별로 클러스터링 수행
        cluster_points = []
        for dist_range, params in self.get_dbscan_params_by_distance().items():
            # 해당 거리 범위의 포인트 필터링
            mask = (self.pc_np[:, 4] >= dist_range[0]) & (self.pc_np[:, 4] < dist_range[1])
            pc_xy = self.pc_np[mask, :2]

            if len(pc_xy) == 0:
                continue

            # DBSCAN 적용
            dbscan = DBSCAN(eps=params['eps'], min_samples=params['min_samples'])
            db = dbscan.fit_predict(pc_xy)
            n_cluster = np.max(db) + 1

            for c in range(n_cluster):
                c_tmp = np.mean(pc_xy[db == c, :], axis=0)
                cluster_points.append([c_tmp[0], c_tmp[1], 1])  # Z 좌표는 1로 고정

        self.publish_point_cloud(cluster_points)

    def get_dbscan_params_by_distance(self):
        """
        거리 범위에 따라 DBSCAN의 eps와 min_samples 파라미터를 설정합니다.
        """
        return {
            (0, 5): {'eps': 0.1, 'min_samples': 20},  # 0m ~ 5m 거리
            (5, 10): {'eps': 0.2, 'min_samples': 15},  # 5m ~ 10m 거리
            (10, 15): {'eps': 0.3, 'min_samples': 10},  # 10m ~ 15m 거리
            # 필요에 따라 더 많은 거리 범위를 추가할 수 있습니다.
        }

    def publish_point_cloud(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        # PointCloud2 메시지 생성
        pc2_msg = pc2.create_cloud(header, fields, points)

        # PointCloud2 메시지 발행
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            # point[0] = x / point[1] = y / point[2] = z
            if point[0] > 0 and -5 < point[1] < 5 and (dist < 15) and (-1.33 < point[2] < 1):
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin()
