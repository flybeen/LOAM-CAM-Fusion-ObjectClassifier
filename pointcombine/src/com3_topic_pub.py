#!/usr/bin/env python3

#######################################################
# /step_2_point 토픽을 받아서, 일단 저장하고
# 계속 받다가 /step_2_point 토픽이 3초동안 안오면 처리 시작
# 마지막으로 온 /step_2_point에서 rgb가 희색인 포인트는 다 달리고
# 이후 /final_result으로 퍼블리쉬
#######################################################
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import time
import struct


class PointCloudSaver:
    def __init__(self):
        rospy.init_node('pointcloud_saver', anonymous=True)
        self.sub = rospy.Subscriber('/step_2_point', PointCloud2, self.callback)
        self.last_msg = None     # step_2_point를 받은 데이터를 저장하는 변수
        self.last_time = None    # step_2_point를 받은 시간 기록
        self.timeout = 3.0       # x초동안 데이터가 안오는지?

    #========================================#
    # /step_2_point를 토픽을 받으면 실행하는 함수
    #========================================#
    def callback(self, msg):
        self.last_msg = msg           # 데이터를 저장
        self.last_time = time.time()  # 시간 저장


    #=============================================================#
    # 시간을 비교, 현재 시간과 토픽을 받은 시간을 비교 후, timeout보다 크면 
    # process_and_save_cloud를 실행
    #=============================================================#
    def save_pcd(self):
        while not rospy.is_shutdown():
            if self.last_msg and time.time() - self.last_time > self.timeout:
                self.process_and_save_cloud(self.last_msg)
                self.last_msg = None #데이터 보내고 리셋
            rospy.sleep(0.1)

    #=================================================#
    # 마지막으로 받은 토픽을 하얀색 포인트를 날려버리는 함수
    #=================================================#
    def process_and_save_cloud(self, cloud_msg):
        cloud = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)) 
        processed_points = []

        for point in cloud:
            x, y, z, rgb = point
            # rgb가 하나의 float형태로 되어있어서, r,g,b로 나누는 작업
            packed_rgb = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (packed_rgb >> 16) & 0xFF
            g = (packed_rgb >> 8) & 0xFF
            b = packed_rgb & 0xFF

            # 그래서 하얀색이면 저장을 안함
            if r == 255 and g == 255 and b == 255:
                continue
            # 하얀색이 아니면 저장
            processed_points.append([x, y, z, r, g, b])

        if processed_points: #for문을 마치고 processed_points에 값이 있다면 save_to_pcd실행
            self.save_to_pcd(processed_points)



    #====================================================================#
    # 전달받은 processed_points를 pcd(point cloud data)형식으로 변환후 퍼블리쉬
    # 학습도 .pcd파일로 하였기 때문
    #====================================================================#
    def save_to_pcd(self, points):
        # open3d PointCloud 객체 형성
        point_cloud = o3d.geometry.PointCloud()
        xyz = np.array(points)[:, :3]
        rgb = np.array(points)[:, 3:] / 255.0  # open3d은 rgb를 0~255가 아니라 0~1로 저장하기 때문에

        #포인트 저장
        point_cloud.points = o3d.utility.Vector3dVector(xyz)
        point_cloud.colors = o3d.utility.Vector3dVector(rgb)

        # Publish as PointCloud2 message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        #필드 정의
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
            rgb_float = struct.unpack('f', struct.pack('I', (int(r) << 16) | (int(g) << 8) | int(b)))[0] #나누어졌던 rgb를 다시 하나로 합침
            points_with_rgb.append([x, y, z, rgb_float])

        cloud_msg = pc2.create_cloud(header, fields, points_with_rgb) #포인트클라우드 형성


        #퍼블리쉬하는 기능, 한번만 퍼블리쉬하면 생략되는 경우가 있어서 2번 발송
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
