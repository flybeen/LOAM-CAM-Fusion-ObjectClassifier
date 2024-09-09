import os
import pickle
import numpy as np
import open3d as o3d
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.metrics import accuracy_score
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy



#==========================================================#
# opt/ros/noetic/lib/python3/dist-packages/ros_numpy/point_cloud2.py 
# def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float): # 기존 코드
# def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float64):  # 수정된 코드
#==========================================================#

# 폴더 및 레이블 정의
global labels 
labels = ['green_cylinder', 'green_sphere', 'green_box', 'red_box', 'red_cylinder', 
          'red_sphere', 'yellow_box', 'yellow_cylinder', 'yellow_sphere']

# 포인트 간 거리의 평균, 포인트 분산, RGB 평균을 사용하는 특징 추출 함수
def extract_features(points, colors):
    # 중심점(centroid) 계산
    centroid = np.mean(points, axis=0)
    
    # 각 포인트와 중심점 사이의 거리 계산
    distances = np.linalg.norm(points - centroid, axis=1)
    
    # 포인트 간 거리의 평균 계산
    mean_distance = np.mean(distances)
    
    # 포인트 간 거리의 분산 계산
    variance_distance = np.var(distances)

    # RGB 값의 평균 계산
    mean_rgb = np.mean(colors, axis=0)

    # 최종 특징 벡터 구성 (포인트 간 거리의 평균, 분산, RGB 평균)
    features = np.concatenate([[mean_distance, variance_distance], mean_rgb])

    return features



# 학습 데이터가 있는지 확인하고 불러오기
def load_or_train_data(data_dir, study_file):
    global labels 
    rospy.loginfo("학습 데이터를 불러오거나 새로 학습을 시작합니다...")
    
    if os.path.exists(study_file):
        rospy.loginfo(f"기존 학습 데이터 {study_file}을(를) 불러옵니다...")
        with open(study_file, 'rb') as f:
            knn, labels = pickle.load(f)
        rospy.loginfo("학습 데이터 불러오기 완료!")
        rospy.loginfo(f"/final_result 토픽 대기 중")
        return knn, labels
    else:
        rospy.loginfo("새로 학습을 시작합니다...")
        data, labels_list = [], []
        
        # 각 폴더에서 .pcd 파일을 처리
        for label in labels:
            rospy.loginfo(f"폴더 {label}에서 .pcd 파일을 처리합니다...")
            for i in range(1, 51):
                file_path = f"{data_dir}/{label}/{label}_{i}.pcd"
                if os.path.exists(file_path):
                    rospy.loginfo(f"파일 {file_path}을(를) 처리 중...")
                    
                    # Open3D로 .pcd 파일을 로드
                    pcd = o3d.io.read_point_cloud(file_path)
                    points = np.asarray(pcd.points)  # XYZ 좌표 추출
                    colors = np.asarray(pcd.colors)  # RGB 값 추출

                    # 새로운 특징 추출 함수 사용
                    features = extract_features(points, colors)

                    data.append(features)
                    labels_list.append(label)
                else:
                    rospy.logwarn(f"파일을 찾을 수 없습니다: {file_path}")

        # 학습 데이터와 검증 데이터를 분리 (40개 학습, 10개 검증)
        rospy.loginfo("학습 및 검증 데이터를 분리합니다...")
        train_data, test_data, train_labels, test_labels = train_test_split(
            data, labels_list, test_size=0.2, stratify=labels_list, random_state=42
        )

        # KNN 모델 및 그리드 서치 설정
        rospy.loginfo("KNN 모델의 최적 K값을 찾습니다...")
        param_grid = {'n_neighbors': np.arange(1, 21)}  # K값 범위 설정
        knn = KNeighborsClassifier()
        grid_search = GridSearchCV(knn, param_grid, cv=5)  # 5-fold cross-validation
        grid_search.fit(train_data, train_labels)

        # 최적의 K값 출력
        best_knn = grid_search.best_estimator_
        rospy.loginfo(f"최적의 K값: {grid_search.best_params_['n_neighbors']}")

        # 검증 데이터로 성능 평가
        rospy.loginfo("검증 데이터로 성능을 평가합니다...")
        test_predictions = best_knn.predict(test_data)
        accuracy = accuracy_score(test_labels, test_predictions)
        rospy.loginfo(f"검증 데이터 정확도: {accuracy * 100:.2f}%")

        # 학습 데이터 저장
        rospy.loginfo(f"학습 데이터를 {study_file}에 저장합니다...")
        with open(study_file, 'wb') as f:
            pickle.dump((best_knn, labels), f)

        rospy.loginfo(f"/final_result 토픽 대기 중")

        return best_knn, labels

# ROS 메시지를 포인트 클라우드로 변환하고 KNN 분류를 수행
def point_cloud_callback(point_cloud_msg):
    rospy.loginfo("새로운 포인트 클라우드 메시지를 수신했습니다...")
    
    # PointCloud2 메시지를 numpy 배열로 변환 (ros_numpy 사용)
    cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(point_cloud_msg)

    # XYZ 좌표 추출
    points = np.column_stack((cloud_arr['x'], cloud_arr['y'], cloud_arr['z']))
    
    # RGB 값 추출 및 분리
    rgb_float = cloud_arr['rgb']
    rgb_int = rgb_float.view(np.uint32)  # float32에서 uint32로 변환
    r = (rgb_int >> 16) & 0xFF
    g = (rgb_int >> 8) & 0xFF
    b = rgb_int & 0xFF
    
    # RGB 값을 0~1 범위로 변환
    rgb_colors = np.column_stack((r, g, b)) / 255.0

    # 특징 추출
    features = extract_features(points, rgb_colors)

    # KNN으로 분류
    prediction = knn.predict([features])
    rospy.loginfo(f"예측된 클래스: {prediction}")

if __name__ == "__main__":
    rospy.init_node('knn_classifier')

    # 학습 데이터 준비
    data_dir = '/home/<username>/catkin_ws/src'
    study_file = os.path.join(data_dir, 'data_study')
    knn, labels = load_or_train_data(data_dir, study_file)

    # ROS 토픽 구독
    rospy.Subscriber('/final_result', PointCloud2, point_cloud_callback)

    rospy.spin()
