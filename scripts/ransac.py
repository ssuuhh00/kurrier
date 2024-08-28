#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.linear_model import RANSACRegressor

class RANSACFilter:
    def __init__(self):
        rospy.init_node('ransac_filter', anonymous=True)
        self.filtered_points_pub = rospy.Publisher('/filtered_cluster_points', PointCloud2, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.object_callback)
        rospy.spin()

    def object_callback(self, msg):
        # 클러스터링된 포인트들을 받음
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
        
        if points.shape[0] < 10:
            rospy.logwarn("Not enough points for RANSAC.")
            return
        
        # RANSAC을 사용하여 주요 포인트 필터링
        ransac = RANSACRegressor(residual_threshold=2.2)
        ransac.fit(points[:, :2], points[:, 2])  # x, y를 독립 변수로, z를 종속 변수로 사용
        inlier_mask = ransac.inlier_mask_

        filtered_points = points[inlier_mask]

        # 필터링된 포인트를 PointCloud2로 변환하여 퍼블리시
        filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)
        self.filtered_points_pub.publish(filtered_msg)

        rospy.loginfo(f"Filtered {len(filtered_points)} points from original {len(points)} points using RANSAC.")

if __name__ == '__main__':
    try:
        RANSACFilter()
    except rospy.ROSInterruptException:
        pass
