#!/usr/bin/env python

############################################################
# com3_topic_pub.py에서 final_result으로 토픽을 보내는 경우가 아닌
# .pcd 파일을 저장하는 코드 -> 학습데이터를 만들때 사용
# 그래서 com3_topic_pub.py와 거의 유사
############################################################
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
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
        self.timeout = 3.0  # 3초 타임아웃

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
            # 'rgb' 필드를 float에서 int로 변환
            packed_rgb = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (packed_rgb >> 16) & 0xFF
            g = (packed_rgb >> 8) & 0xFF
            b = packed_rgb & 0xFF
            # 하얀색 필터링 (RGB 값이 모두 255일 때)
            if r == 255 and g == 255 and b == 255:
                continue
            # RGB 정보를 포함하여 저장
            processed_points.append([x, y, z, r, g, b])

        if processed_points:
            self.save_to_pcd(processed_points)

    
    def save_to_pcd(self, points):
        # 포인트 클라우드를 open3d로 변환 (XYZ + RGB)
        point_cloud = o3d.geometry.PointCloud()
        xyz = np.array(points)[:, :3]
        rgb = np.array(points)[:, 3:] / 255.0  # RGB 값을 0-1 범위로 정규화
        point_cloud.points = o3d.utility.Vector3dVector(xyz)
        point_cloud.colors = o3d.utility.Vector3dVector(rgb)

        directory = "/home/kriso/sf_ws/src/data_mach/green_box"
        if not os.path.exists(directory):
            os.makedirs(directory)

        file_index = 1
        while os.path.exists(os.path.join(directory, f"green_box_{file_index}.pcd")):
            file_index += 1

        save_path = os.path.join(directory, f"green_box_{file_index}.pcd")
        o3d.io.write_point_cloud(save_path, point_cloud)
        rospy.loginfo(f"Saved point cloud to {save_path}")

if __name__ == '__main__':
    saver = PointCloudSaver()
    saver.save_pcd()
