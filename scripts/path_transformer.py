#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from math import sin, cos

class PathTransformer:
    def __init__(self):
        rospy.init_node('path_transformer')
        
        self.path_sub = rospy.Subscriber('/local_path', Path, self.path_callback)
        self.path_pub = rospy.Publisher('/local_path2', Path, queue_size=10)
        
        # Assume heading angle is published on a topic '/vehicle_heading'
        self.heading_sub = rospy.Subscriber('/vehicle_heading', Float64, self.heading_callback)
        
        self.heading = 0.0  # Initialize heading to 0

    def heading_callback(self, msg):
        self.heading = msg.data

    def path_callback(self, path_msg):
        transformed_path = Path()
        transformed_path.header = path_msg.header
        
        # Assuming first pose in path is the origin
        first_pose = path_msg.poses[0].pose.position
        
        for pose_stamped in path_msg.poses:
            pose = pose_stamped.pose.position
            x, y = pose.x - first_pose.x, pose.y - first_pose.y
            
            # Rotate the point by the negative heading angle to get relative coordinates
            x_rel = x * cos(-self.heading) - y * sin(-self.heading)
            y_rel = x * sin(-self.heading) + y * cos(-self.heading)
            
            # Update the pose with new relative coordinates
            new_pose = PoseStamped()
            new_pose.header = pose_stamped.header
            new_pose.pose.position = Point(x_rel, y_rel, pose.z)  # Assuming z remains unchanged
            
            transformed_path.poses.append(new_pose)
        
        self.path_pub.publish(transformed_path)

if __name__ == '__main__':
    try:
        path_transformer = PathTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
