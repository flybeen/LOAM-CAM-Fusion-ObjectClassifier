#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import os
import time
import struct


class PointCloudSaver:
    def __init__(self):
        rospy.init_node('pointcloud_saver', anonymous=True)
        self.sub = rospy.Subscriber('/step_2_point', PointCloud2, self.callback)
        self.last_msg = None
        self.last_time = None
        self.timeout = 3.0  # 3-second timeout

    def callback(self, msg):
        self.last_msg = msg
        self.last_time = time.time()

    def save_pcd(self):
        while not rospy.is_shutdown():
            if self.last_msg and time.time() - self.last_time > self.timeout:
                self.process_and_save_cloud(self.last_msg)
                self.last_msg = None
            rospy.sleep(0.1)

    def process_and_save_cloud(self, cloud_msg):
        cloud = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        processed_points = []

        for point in cloud:
            x, y, z, rgb = point
            # Convert 'rgb' field from float to int
            packed_rgb = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (packed_rgb >> 16) & 0xFF
            g = (packed_rgb >> 8) & 0xFF
            b = packed_rgb & 0xFF
            # Filter out white (RGB all 255)
            if r == 255 and g == 255 and b == 255:
                continue
            # Store XYZ and RGB data
            processed_points.append([x, y, z, r, g, b])

        if processed_points:
            self.save_to_pcd(processed_points)

    def save_to_pcd(self, points):
        # Convert points to open3d PointCloud (XYZ + RGB)
        point_cloud = o3d.geometry.PointCloud()
        xyz = np.array(points)[:, :3]
        rgb = np.array(points)[:, 3:] / 255.0  # Normalize RGB to 0-1 range
        point_cloud.points = o3d.utility.Vector3dVector(xyz)
        point_cloud.colors = o3d.utility.Vector3dVector(rgb)

        # Publish as PointCloud2 message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]

        points_with_rgb = []
        for point in points:
            x, y, z = point[:3]
            r, g, b = point[3:]
            rgb_float = struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0]
            points_with_rgb.append([x, y, z, rgb_float])

        cloud_msg = pc2.create_cloud(header, fields, points_with_rgb)

        # Publisher (initialize once)
        if not hasattr(self, 'pub'):
            self.pub = rospy.Publisher('/final_result', PointCloud2, queue_size=10)

        # Publish the message 5 times with 0.2-second intervals
        for _ in range(2):
            self.pub.publish(cloud_msg)
            rospy.loginfo("Published point cloud to /final_result")
            rospy.sleep(0.2)  # Sleep for 0.2 seconds before publishing again

if __name__ == '__main__':
    saver = PointCloudSaver()
    saver.save_pcd()
