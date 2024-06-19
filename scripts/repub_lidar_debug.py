#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl

def callback(point_cloud_msg):
    pc = pcl.PointCloud()
    pc.from_list([p for p in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)])

    filtered_points = []
    for point in pc:
        x, y, z = point
        # if 2 < x < 10 and 0.2 < y < 1.5 and 0.5 < z < 5:
        # if 2 < x < 10:
        if 1.5 < y < 1.7:
            filtered_points.append(point)
        # filtered_points.append(point)

    header = point_cloud_msg.header
    filtered_pc_msg = pc2.create_cloud_xyz32(header, filtered_points)

    pub.publish(filtered_pc_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_filter_node', anonymous=True)

    rospy.Subscriber('/ouster/points', PointCloud2, callback)
    pub = rospy.Publisher('/repub_points', PointCloud2, queue_size=10)
    rospy.spin()
