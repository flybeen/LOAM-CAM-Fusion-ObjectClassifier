#!/usr/bin/env python3

########################################################
#
#  라이다-카메라 퓨전 (혹은 캘리브레이션)함수
#  라이다에서 오는 /velodyne_points 토픽과, 카메라에서 오는 /camera/image_raw 토픽을 받아서
#  캘리브레이션에 필요한 내부 파라미터와 외부 파라미터를 계산 후에
#  퓨전 하는 코드
#
#  원래는 체크보드 등 라이다와 카메라의 외부파라미터를 PnP(Persepective-n-Point)문제로 해결하나
#  코드가 복잡하고 잘구해지지 않아, 직접 계산하는 방식으로 해결함
#  이는 카메라와 라이다의 관계가 urdf로 명확하게 계산되어 있어서 가능함
#
########################################################
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, Image, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros
import threading
import time
import std_msgs.msg
import struct
import random



class LidarCameraCalibration:
    def __init__(self):
        rospy.loginfo("Initializing LidarCameraCalibration node")
        self.bridge = CvBridge()
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.camera_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        self.lidar_data = None             # 오리지널 라이다(포인트클라우드2) 저장할 변수
        self.camera_data = None            # 오리지널 카메라데이터를 저장할 변수
        self.overlay_image = None          # 카메라 데이터 위에 라이다를 오버레이한 이미지를 저장할 변수
        self.overlayed_points = None       # 오버레이한 이미지에서 rgb값을 획득 후, 포인트클라우드로 보내기 위해 임시 저장하는 변수

        self.transform_pub = rospy.Publisher('/lidar_camera_transform', std_msgs.msg.Float64MultiArray, queue_size=10)
        self.image_pub = rospy.Publisher('/overlay_image', Image, queue_size=1)
        self.pointcloud_pub = rospy.Publisher('/overlay_pointcloud', PointCloud2, queue_size=1)
        self.transformed_cloud_pub = rospy.Publisher('/transformed_pointcloud', PointCloud2, queue_size=1)

        #테스트용
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


        # pnp문제의 내부 파라미터
        self.camera_matrix = np.array([[1131.20, 0.000000, 640.0],
                                       [0.000000, 1131.20, 360.0],
                                       [0.000000, 0.000000, 1.000000]])
        # 카메라 왜곡 계수 (시뮬레이션 속의 카메라이기 때문에 변수가 없음) 현실에서는 렌즈에 의해 왜곡이 생김
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # 외부 파라미터를 계산하기 위해 기구학적 회전매트릭스
        self.rotation_matrix = np.array([
            [1, 0,  0],
            [0,  0, -1],
            [0,  1,  0]
        ])
        # 외부 파라미터를 계산하기 위해 기구학적 이동매트릭스
        self.translation_vector = np.array([0, 0.1, 0.1])


        self.display_thread = threading.Thread(target=self.display_loop)   #멀티 스레딩을 이용하여, display_loop함수 실행
        self.display_thread.daemon = True #메인 스레드가 종료될때, 같이 종료
        self.display_thread.start()

        #라이다 업데이트 주기(카메라 데이터 주기와 맞추기 위해)
        self.last_lidar_update_time = time.time()   
        self.lidar_update_interval = 0.5



    #=========================================#
    #  오버레이된 이미지를 GUI창으로 띄우기 위한 함수
    #=========================================#
    def display_loop(self):
        while not rospy.is_shutdown():
            if self.overlay_image is not None:
                cv2.imshow('Image with Lidar Points', self.overlay_image)
                cv2.waitKey(1)
        cv2.destroyAllWindows()

    
    #=================================================#
    # /velodyne_points 토픽이 오면 실행하는 함수
    # 현재시간이랑 받은 시간을 비교해서 0.5초가 지나지 않았으면 
    # 토픽을 저장하지 않은 함수
    #=================================================#
    def lidar_callback(self, data):
        current_time = time.time()
        if current_time - self.last_lidar_update_time < self.lidar_update_interval:
            return
        self.last_lidar_update_time = current_time

        try:
            self.lidar_data = data
            self.perform_calibration()
        except Exception as e:
            rospy.logerr(f"Error in lidar_callback: {e}")


    #=================================================#
    # /camera/image_raw 토픽을 받으면 실행되는 함수
    # 먼저 bgr8(OpenCV 이미지 형식)으로 바꿔 camera_data에 저장
    # 이후 캘리브레이션 실행
    #=================================================#
    def camera_callback(self, data):
        try:
            self.camera_data = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.perform_calibration()
        except Exception as e:
            rospy.logerr(f"Error in camera_callback: {e}")



    #==============================================================#
    # 카메라-라이다 캘리브레이션 함수. 즉, 카메라와 라이다의 위치를 맞추는 함수
    #==============================================================#
    def perform_calibration(self):
        # 라이다 데이터와 카메라 데이터가 있는지 확인. 없으면 실행 X
        if self.lidar_data is None or self.camera_data is None:
            return

        #라이다 데이터를 NumPy 배열로 변환, skip_nans=True은 너무 멀리 있거나 너무 가까운(Nan)을 0으로 치환
        try:
            points = np.array(list(pc2.read_points(self.lidar_data, skip_nans=True, field_names=("x", "y", "z", "intensity"))))

            if points.shape[1] == 4:
                xyz = points[:, :3]        
                intensities = points[:, 3]
            else:
                xyz = points
                intensities = np.zeros(points.shape[0]) #강도 값이 없는경우 0으로 치환


            try:    
                transformation_matrix = np.eye(4) #4x4함수 생성 (단위 행렬)
                #기구학 4x4행렬로 변환.
                transformation_matrix[:3, :3] = self.rotation_matrix   
                transformation_matrix[:3, 3] = self.translation_vector


                self.publish_transform(self.rotation_matrix, self.translation_vector)
                self.overlay_points_on_image(xyz, transformation_matrix, intensities)

                if self.overlay_image is not None:  #오버레이 된 이미지가 있는지 확인
                    image_msg = self.bridge.cv2_to_imgmsg(self.overlay_image, "bgr8") #이미지를 토픽 메세지로 변환
                    self.image_pub.publish(image_msg)   #오버레이 이미지 토픽 퍼블리쉬

            except Exception as e:
                rospy.logerr(f"Error in applying transformation: {e}")
        except Exception as e:
            rospy.logerr(f"Error in perform_calibration: {e}")


    #===========================================#
    #  강도값을 rgb형식으로 바꾸는 함수
    #  라이다 포인트를 이미지에 오버레이하기 위해 필요
    #===========================================#
    def intensities_to_rgb(self, intensities):
        # 강도값을 정규화하기 위해 필요, 정규화를 하지 않을시, 오버레이할 때 포인트의 색상이 동일함.
        # 강도값을 지원하는 라이다만가능. 시뮬레이션에서는 아마 동일한 색상으로 나올것 같음
        min_intensity = np.min(intensities)
        max_intensity = np.max(intensities)

        # 정규화 코드
        if max_intensity - min_intensity == 0:  #정규화가 의미 없을때,
            norm_intensities = np.zeros_like(intensities)  #그냥 0으로해서, 계산을 빠르게
        else:
            norm_intensities = (intensities - min_intensity) / (max_intensity - min_intensity) * 255   #최소 강도값과 최대 강도값을 0~255값으로 환산(정규화)
        

        norm_intensities = norm_intensities.astype(np.uint8)   #정규화 된 값을 8비트 정수로 환산, openCV에서는 8비트 정수형태로 계산하기 때문에
        colormap = cv2.applyColorMap(norm_intensities.reshape(-1, 1), cv2.COLORMAP_JET)   #0~255값을 빨강~파랑으로 rgb로 변경
        return colormap[:, 0, :] #rgb값만 리턴



    #=====================================================#
    # 라이다와 카메라의 거리를 눈으로 직접 눈으로 보기 위해 만든 함수
    # /lidar_camera_transform 토픽으로 퍼블리쉬 되지만
    # 확인용이라서 다른곳에서 사용하지 않음
    #=====================================================#
    def publish_transform(self, rotation_matrix, translation_vector):
        try:
            t = std_msgs.msg.Float64MultiArray()

            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = translation_vector.flatten()

            t.data = transform_matrix.flatten().tolist()

            self.transform_pub.publish(t)
        except Exception as e:
            rospy.logerr(f"Error in publish_transform: {e}")



    #======================================================================#
    # perform_calibration함수에서 카메라와 라이다의 위치를 맞추었으니(캘리브레이션)
    # 이제 카메라 사진위에 라이다 포인트를 띄우는 함수
    #======================================================================#
    def overlay_points_on_image(self, points, transformation_matrix, intensities):
        try:
            #xyz에 1을 추가한 동차좌표 만들기(기구학 계산을 하기위해)
            points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
            # 기구학 계산
            transformed_points_homogeneous = points_homogeneous.dot(transformation_matrix.T)
            transformed_points = transformed_points_homogeneous[:, :3] / transformed_points_homogeneous[:, 3][:, np.newaxis] #뭔지 모름 chatGPT에게 감사를.....

            # 포인트클라우드 좌표를 제한, 우선 제한을 하지않으면 앞뒤의 포인트클라우드가 겹쳐서 오버레이가 제대로되지않음
            # 그래서 일단 카메라 정면 방향의 포인트만 출력    앞으로(1.5~4)  양옆으로(-1~1)
            # 즉, 직육면체의 공간이 생기는데, 이공간을 벗어난 오버레이를 하게되면, 카메라와 라이다의 초점(?)이 안맞기 때문에.....
            # 카메라기준 [:, 2] 앞뒤, [:, 0] 좌우, [:, 1] 위아래
            front_points_idx = (transformed_points[:, 2] > 1.5) & (transformed_points[:, 2] < 4) & (transformed_points[:, 0] > -1) & (transformed_points[:, 0] < 1) & (transformed_points[:, 1] < 0.2)
            front_points = transformed_points[front_points_idx] 
            front_intensities = intensities[front_points_idx]

            front_colors = self.intensities_to_rgb(front_intensities)   #강도를 rgb로 변환하는 함수
            image_points, _ = cv2.projectPoints(front_points, np.zeros((3, 1)), np.zeros((3, 1)), self.camera_matrix, self.dist_coeffs) #변환된 좌표를 카메라 좌표로 맞춤

            #이제 오버레이 이미지 만들준비
            self.overlay_image = self.camera_data.copy()
            valid_points = 0    # 카메라 범위 내에 있는 포인트클라우드
            invalid_points = 0  # 카메라 범위 밖에 있는 포인트클라우드
            new_points = []     # rgb값을 갖고 다시 퍼블리쉬할 예정인 포인트클라우드 저장변수

            # 역변환 (오버레이 이미지용 X, /overlay_pointcloud으로 보내기 위해서 역변환)
            inverse_transformation_matrix = np.linalg.inv(transformation_matrix)
            restored_points_homogeneous = np.hstack((front_points, np.ones((front_points.shape[0], 1))))
            restored_points_homogeneous = restored_points_homogeneous.dot(inverse_transformation_matrix.T)
            restored_points = restored_points_homogeneous[:, :3] / restored_points_homogeneous[:, 3][:, np.newaxis]
            
            #이제 오버레이하면서 새로운 포인트클라우드 형성
            for idx, (point, color) in enumerate(zip(image_points, front_colors)):
                x, y = point.ravel()
                if (0 <= x < self.camera_data.shape[1]) and (0 <= y < self.camera_data.shape[0]):
                    bgr_color = tuple(map(int, color))
                    cv2.circle(self.overlay_image, (int(x), int(y)), 1, bgr_color, -1)
                    valid_points += 1
                    b, g, r = self.camera_data[int(y), int(x)]
                    rgb_float = self.rgb_to_float(r, g, b)
                    new_points.append([restored_points[idx, 0], restored_points[idx, 1], restored_points[idx, 2], rgb_float])
                else:
                    invalid_points += 1
            
            # /overlay_pointcloud으로 보낼때, 원래 있던거에서 어느정도 비율로 다시 보낼것인가.
            sample_ratio = 1.0
            new_points = random.sample(new_points, int(len(new_points) * sample_ratio))

            rospy.loginfo(f"Valid points: {valid_points}, Invalid points: {invalid_points}, Sampled points: {len(new_points)}")

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base_footprint'
            point_cloud = self.create_cloud_xyzrgb(header, new_points)

            self.overlayed_points = new_points
            self.pointcloud_pub.publish(point_cloud)
            self.publish_transformed_pointcloud(transformed_points, transformation_matrix)

        except Exception as e:
            rospy.logerr(f"Error in overlay_points_on_image: {e}")



    #다시 rgb를 float형태로 변경하는 함수
    def rgb_to_float(self, r, g, b):
        return struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]

    #pointXYZRGB를 만드는 함수. pcl라이브러리 참조.
    def create_cloud_xyzrgb(self, header, points):
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]

        cloud_data = []
        for point in points:
            x, y, z, rgb = point
            cloud_data.append([x, y, z, rgb])

        return pc2.create_cloud(header, fields, cloud_data)

    #================================================#
    # transformed_pointcloud 토픽을 보내기 위해 만든함수.
    # overlayed_points와 velodyne_points을 합치는데,
    # overlayed_points는 rgb값이 들어있기 때문에 그대로 냅두고
    # velodyne_points은 rgb값이 없기 때문에, 하얀색으로 출력
    # ===============================================#
    def publish_transformed_pointcloud(self, transformed_points, transformation_matrix):
        try:
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base_footprint'

            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.FLOAT32, 1)
            ]

            inverse_transformation_matrix = np.linalg.inv(transformation_matrix)
            restored_points_homogeneous = np.hstack((transformed_points, np.ones((transformed_points.shape[0], 1))))
            restored_points_homogeneous = restored_points_homogeneous.dot(inverse_transformation_matrix.T)
            restored_points = restored_points_homogeneous[:, :3] / restored_points_homogeneous[:, 3][:, np.newaxis]

            overlayed_points_set = set(tuple(p[:3]) for p in self.overlayed_points)

            unique_transformed_points = []
            for point in restored_points:
                point_tuple = tuple(point[:3])
                if point_tuple not in overlayed_points_set:
                    unique_transformed_points.append([point[0], point[1], point[2], self.rgb_to_float(255, 255, 255)])

            final_points = unique_transformed_points + self.overlayed_points

            cloud_data = []
            for point in final_points:
                x, y, z, rgb = point
                cloud_data.append([x, y, z, rgb])

            final_cloud = pc2.create_cloud(header, fields, cloud_data)
            self.transformed_cloud_pub.publish(final_cloud)

        except Exception as e:
            rospy.logerr(f"Error in publish_transformed_pointcloud: {e}")

if __name__ == "__main__":
    rospy.init_node('lidar_camera_calibration')
    calibration = LidarCameraCalibration()
    rospy.spin()
