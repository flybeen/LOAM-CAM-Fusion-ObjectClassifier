#!/usr/bin/env python3

#######################################################################
#  com1.py에서 좌표변환이 된 토픽 step_1_point을 받고, A-LOAM에서 맵핑이 된
#  /laser_cloud_map 토픽을 받아서 step_1_point에 있는 rgb을 /laser_cloud_map 
#  에 색칠하는 코드이다. 
#  하지만, /laser_cloud_map 토픽을 계속 받으면 기존에 있던 포인트가 사라지고 
#  새로운 포인트가 생기기 때문에, 한 바퀴를 돈 후, 딱 한번만 토픽을 받는다.
#######################################################################
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import struct
import threading
import time
import open3d as o3d
from std_srvs.srv import Empty, EmptyResponse


class PointCloudColorMapper:
    def __init__(self):
        rospy.init_node('pointcloud_color_mapper', anonymous=True)
        
        self.map_sub = None  # laser_cloud_map에 있는 포인트를 저장할 변수

        # 색칠이 완료된 포인트를 퍼블리시할 Publisher
        self.step_2_pub = rospy.Publisher('/step_2_point', PointCloud2, queue_size=10)

        self.laser_cloud_map_storage = None  # 최종 맵핑 저장 변수 (색칠 되어 있는)

        # Contr
        self.is_running = False   #현재 서비스가 진행중인가?
        self.processing_lock = threading.Lock()  # 중복 작업 방지

        # /start_processing 서비스가 call되면 작업을 시작함
        self.start_service = rospy.Service('start_processing', Empty, self.start_processing) 

        rospy.loginfo("PointCloudColorMapper node initialized. Waiting for start command...")


    #===========================================================================#
    # start_processing서비스를 받으면 시작되는 함수
    # /laser_cloud_map를 토픽 구독을 시작한다. 즉, 한 바퀴 돌았다는(맵핑)이 완료됐다는 가정
    # 그리고 끝난면 Empty를 반환한다.
    #===========================================================================#
    def start_processing(self, req):
        with self.processing_lock:
            if not self.is_running:
                rospy.loginfo("Start command received via ROS service. Subscribing to /laser_cloud_map...")
                
                # Subscribe to /laser_cloud_map once when the service is called
                self.map_sub = rospy.Subscriber('/laser_cloud_map', PointCloud2, self.map_callback)
                
                return EmptyResponse()
            else:
                rospy.logwarn("Processing is already running.")
                return EmptyResponse()


    #================================================#
    # /laser_cloud_map 토픽을 받으면 실행되는 함수
    #================================================#
    def map_callback(self, msg):
        with self.processing_lock:  # 포인트 클라우드 데이터가 다른 함수에서 건들지 않게 락을 건다.
            if self.laser_cloud_map_storage is None: # laser_cloud_map_storage에 아무런 값이 없을때, if문 실행 (값이 없기 때문에 반드시 실행됨)
                rospy.loginfo("Received /laser_cloud_map point cloud. Storing and ready to start processing.")

                #/laser_cloud_map(msg)는 pointXYZI로 선언되어 있어서 xyz만 추출
                map_points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))) 

                if map_points.size == 0: #xyz가 하나도 없으면? a-loam이 제대로 작동되지 않았다는 뜻인데, 이건 create_data/run* 에서 해결함
                    rospy.logwarn("Received /laser_cloud_map is empty. Waiting for non-empty point cloud.")
                    return
                
                # /laser_cloud_map(msg)의 rgb값은 존재 하지않기 때문에 하얀색으로 초기화.
                white_rgb = struct.unpack('f', struct.pack('I', 0xFFFFFFFF))[0]
                #하얀색과 xyz를 laser_cloud_map_storage에 대입
                self.laser_cloud_map_storage = np.hstack((map_points, np.full((map_points.shape[0], 1), white_rgb)))

                self.map_sub.unregister() # 구독해제 (한번만 받을꺼라서).
                self.is_running = True
                self.start_step_1_listener()

    #==================================================#
    # /laser_cloud_map의 전처리 과정을 끝내면 실행되는 함수
    # /step_1_point을 받기 시작한다.
    #==================================================#
    def start_step_1_listener(self):
        # 구독을 시작
        self.step_1_sub = rospy.Subscriber('/step_1_point', PointCloud2, self.step_1_callback)

        #step_1_point를 55초동안만 받음
        self.start_time = time.time() #현재 시간 저장
        self.timer_thread = threading.Thread(target=self.time_progress) #스레드사용 (병럴처리)
        self.timer_thread.start() #시작

        self.timer = threading.Timer(55.0, self.stop_step_1_listener) #55초후 stop_step_1_listener함수 실행하는걸 저장
        self.timer.start() #시작

    #=================================================#
    # 구독을 시작하면 55초동안 실행되는데, 현재 몇 초 진행되는지
    # loginfo(터미널)에서 알려주는 함수
    #=================================================#
    def time_progress(self):
        while self.is_running:
            elapsed_time = time.time() - self.start_time
            rospy.loginfo("Elapsed time: {:.2f} seconds / 55sec".format(elapsed_time))
            if elapsed_time >= 55.0:
                break
            time.sleep(5)
    #=====================================================#
    # 55초 뒤, 진행되는 함수, /step_1_point 토픽 구독을 해제함
    #=====================================================#
    def stop_step_1_listener(self):
        with self.processing_lock:
            if self.step_1_sub:
                self.step_1_sub.unregister()
                rospy.loginfo("Stopped listening to /step_1_point.")
            
            self.is_running = False
            
            # laser_cloud_map_storage 초기화
            self.laser_cloud_map_storage = None

            rospy.loginfo("Ready for new start command.")

    
    #============================================#
    # /step_1_point 토픽을 받자마자 실행되는 함수
    #============================================#
    def step_1_callback(self, msg):
        with self.processing_lock:

            # /step_1_point을 받으면 xyz rgb값을 따와서 데이터를 저장
            step_1_points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)))

            # /step_1_point가 없으면 진행 X
            if step_1_points.size == 0:
                rospy.logwarn("Received /step_1_point is empty. Skipping processing.")
                return

            # /step_1_point토픽 데이터가 매우 많기 때문에 계산량이 매우 많음 -> 느림
            # 그래서 Voxel Grid 필터링을 이용해, 포인트의 개수를 줄임
            step_1_points = self.voxel_grid_filter(step_1_points, leaf_size=0.1)

            # 다운 샘플링된 포인트를 전달
            self.process_point_clouds(step_1_points)


    #=========================================================================#
    # 다운 샘플링 하는 함수 
    # leaf_size은 0.1단위로 나눔 -> 숫자를 증가시, 더 다운샘플링함(정육면체 길이라고 생각)
    #=========================================================================#
    def voxel_grid_filter(self, points, leaf_size=0.1):
        voxel_indices = np.floor(points[:, :3] / leaf_size).astype(np.int32)
        dtype = np.dtype((np.void, voxel_indices.dtype.itemsize * voxel_indices.shape[1]))
        voxel_indices_view = np.ascontiguousarray(voxel_indices).view(dtype)
        _, unique_indices = np.unique(voxel_indices_view, return_index=True)
        downsampled_points = points[unique_indices]

        return downsampled_points


    #==================================================================#
    # laser_cloud_map_storage에 하얀색으로 기록된 포인트 클라우드를
    # 다운샘플링된 포인트 xyz와 laser_cloud_map_storage의 xyz를 비교해서
    # 비슷한곳(0.1m)에 있으면 laser_cloud_map_storage에 다운샘플링 rgb 대입
    #==================================================================#
    def process_point_clouds(self, step_1_points):
        # laser_cloud_map_storage, step_1_points에 데이터가 있는지 확인
        if self.laser_cloud_map_storage is None or step_1_points is None:
            rospy.logwarn("Point clouds are missing during processing. Aborting.")
            return

        # open3d 포인트클라우드 객체 생성.
        # KDtree를 사용하기 위해서는, open3d.geometry.PointCloud 객체가 필요하기 때문에
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(step_1_points[:, :3]) #xyz만 필요해서 대입

        # 다운샘플링된 데이터가 없으면 실행 X
        if len(pcd.points) == 0:
            rospy.logwarn("step_1_points is empty after downsampling. Skipping processing.")
            return

        # KDtree 객체 형성
        # KDtree -> 다차원 공간에서 효율적인 검색을 위한 데이터 구조(최근접 포인트 찾기위해)
        step_1_tree = o3d.geometry.KDTreeFlann(pcd)

        tolerance = 0.1  # 하나의 점에 대해서 10cm 주변의 포인트찾기

        # 기준점은 laser_cloud_map_storage 포인트
        # 찾은 이웃과의 거리가 허용 거리(예: 10cm) 이내인 경우, 그 이웃(step_1_point)의 RGB 값을 복사하여 laser_cloud_map_storage에 색칠하는 방식
        for i, map_point in enumerate(self.laser_cloud_map_storage):
            current_rgb = map_point[3]

            #흰색이 아닌 경우 색칠하지 않음.
            if struct.unpack('I', struct.pack('f', current_rgb))[0] != 0xFFFFFFFF:
                continue

            # xyz좌표
            query_point = map_point[:3].tolist()

            # 가장 가까운 포인트 1개를 찾음 (10cm 이내)
            try:
                [k, idx, dist] = step_1_tree.search_knn_vector_3d(query_point, 1) # k 발견된 이웃의 개수, idx은 이웃의 인덱스, dist는 거리
            except Exception as e:
                continue

            if k > 0 and dist[0] < tolerance ** 2:  # 거리의 제곱값이 0.1m 이하인지, open3d는 거리를 제곱값으로 반환함
                rgb = step_1_points[idx[0], 3]
                self.laser_cloud_map_storage[i, 3] = rgb

        self.publish_colored_point_cloud() # 색칠된 값 퍼블리쉬


    #========================================================#
    # laser_cloud_map_storage을 /step_2_point 토픽으로 보내는 함수
    #========================================================#
    def publish_colored_point_cloud(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/camera_init'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]
        colored_pc_msg = point_cloud2.create_cloud(header, fields, self.laser_cloud_map_storage)

        self.step_2_pub.publish(colored_pc_msg)

    #=====#
    # run #
    #=====#
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        mapper = PointCloudColorMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
