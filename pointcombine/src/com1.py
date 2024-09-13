#!/usr/bin/env python3

###################################################################################
#  A-LOAM을 이용해 .pcd파일을 만드는 과정 중. 첫번째 단계
#
#  원래 A-LOAM에서 rgb값을 그대로 유지하면 참 편리하겠지만....
#  A-LOAM에서는 포인트클라우드가 pointXYZI로 선언되고 계산하기때문에 RGB값을 넣지 못한다.
#  그래서.... pointXYZIRGB라고 직접 클래스(?)를 만들어서 할려고 했으나...
#  KDtree에서는 이미 정의된 point형식에서만 사용이 가능했다.
#  
#  그래서 방식을 변경하였다. 먼저 a-loam을 진행하면 /laser_cloud_map으로 맵핑되는 포인트클라우드랑
#  /aft_mapped_path으로 현재 위치 및 지나온 경로를 토픽으로 보낸다.
#  이 토픽을 이용해서 .pcd파일을 만들기로 하였다.
#
#  우선 첫번째는 lider_camera_calibration에서 작업된 토픽 /transformed_pointcloud와
#  /aft_mapped_path 토픽을 이용해 원점에서 /transformed_pointcloud를 좌표변환 하는 코드이다.
#
#####################################################################################
import rospy
import tf
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import tf.transformations
import struct

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('pointcloud_transformer', anonymous=True)
        
        # Ensure ROS uses simulation time
        rospy.set_param('/use_sim_time', True)

        # Subscribers
        self.pc_sub = rospy.Subscriber('/transformed_pointcloud', PointCloud2, self.pc_callback)
        self.path_sub = rospy.Subscriber('/aft_mapped_path', Path, self.path_callback)

        # 좌표 변환이 완료된 토픽은 /step_1_point으로 퍼블리쉬된다.
        self.pc_pub = rospy.Publisher('/step_1_point', PointCloud2, queue_size=10)

        self.latest_pc = None       # /transformed_pointcloud으로 오는 포인트클라우드를 저장하는 변수
        self.latest_path = None     # /aft_mapped_path으로 오는 경로와 현재위치를 저장하는 변수
        
        # 시뮬레이션 속의 라이다의 fixed frame이랑 aloam의 fixed frame이 다르기 때문에 하나로 묶어줄 필요가 있음.
        #  라이다의 fixed frame랑 aloam의 fixed frame이 어떤 tf관계를 가지는가를 정의하기 위해 만든 tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    #====================================================#
    # fixed frame가 /로 시작하면 /를 삭제하는 함수
    # a-loam의 fixed frame이 /camera_init으로 시작한다
    # 현재버전들(?) fixed frame가 /로 시작하여 계산되면 안되기때문에 제거
    #====================================================#
    def sanitize_frame_id(self, frame_id):
        """Remove leading slashes from frame_id."""
        if frame_id.startswith('/'):
            return frame_id[1:]
        return frame_id

    #=======================================================#
    # transformed_pointcloud을 받으면 latest_pc에 저장하는 함수
    #=======================================================#
    def pc_callback(self, msg):
        self.latest_pc = msg
        self.latest_pc.header.frame_id = self.sanitize_frame_id(self.latest_pc.header.frame_id) # ( / )제거, 굳이 할 필요 없긴하다.

        
    #======================================================#
    # aft_mapped_path를 받으면 latest_path에 저장하는 함수
    # 이후, latest_pc에 값이 있으면 process_data함수 실행
    # 즉, aft_mapped_path이 들어올때마다만, 프로그램이 실행된다.
    # (여기에 Subscriber('/transformed_pointcloud'를 넣어서
    # aft_mapped_path들어올때, transformed_pointcloud을 받아서
    # 오차값을 안만들기)
    #======================================================#
    def path_callback(self, msg):
        self.latest_path = msg
        self.latest_path.header.frame_id = self.sanitize_frame_id(self.latest_path.header.frame_id)
        
        if self.latest_pc is not None:
            self.process_data()
        
    #==================================================================#
    # 최종적으로 /aft_mapped_path와 /transformed_pointcloud를 결합하는 함수
    #==================================================================#
    def process_data(self):
        # 두개의 데이터가 준비되지 않았으면 실행 X
        if self.latest_pc is None or self.latest_path is None:
            return

        last_pose = self.latest_path.poses[-1].pose # 파이썬에서 [-1]은 마지막 데이터 = 현재위치
        translation = last_pose.position
        rotation = last_pose.orientation

        # 변환 매트릭스 만들기
        transform_matrix = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w]) # 쿼터니언을 매트릭스로 변환
        transform_matrix[0:3, 3] = [translation.x, translation.y, translation.z] #위치 행렬을 매트릭스로 변환

        # point 추출하기. x, y, z, rgb값 추출, Nan값은 스킵
        points = list(point_cloud2.read_points(self.latest_pc, field_names=("x", "y", "z", "rgb"), skip_nans=True))

        # points를 numpy배열로 변환 -> 계산을 빠르게 하기 위해
        points = np.array(points)
        points_homogeneous = np.ones((points.shape[0], 4)) # [x, y, z, 1]으로 동차 좌표로 만들기
        points_homogeneous[:, 0:3] = points[:, 0:3]  # Copy XYZ values

        # 좌표 변환 계산
        transformed_points = np.dot(transform_matrix, points_homogeneous.T).T[:, 0:3]

        # 좌표 변환된 xyz와 기존의 rgb값 합치기
        transformed_points_with_rgb = []
        for (x, y, z, rgb) in zip(transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2], points[:, 3]):
            transformed_points_with_rgb.append([x, y, z, rgb])

        # Create a new PointCloud2 message
        header = Header()
        header.stamp = self.latest_pc.header.stamp  # Maintain the original timestamp
        header.frame_id = "/camera_init"  # Set the frame to camera_init after transformation
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)  # Use FLOAT32 for RGB
        ]
        transformed_pc_msg = point_cloud2.create_cloud(header, fields, transformed_points_with_rgb)

        # Publish the transformed point cloud
        self.pc_pub.publish(transformed_pc_msg)

       
if __name__ == '__main__':
    try:
        PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
