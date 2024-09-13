import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import rospy
import numpy as np
import os
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import ros_numpy
from std_msgs.msg import String
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score

# 포인트 클라우드 전처리 함수
def preprocess_small_pointcloud(points, colors, target_size=128):
    if len(points) < target_size:
        extra_indices = np.random.choice(len(points), target_size - len(points))
        points = np.vstack((points, points[extra_indices]))
        colors = np.vstack((colors, colors[extra_indices]))
    elif len(points) > target_size:
        indices = np.random.choice(len(points), target_size, replace=False)
        points = points[indices]
        colors = colors[indices]
    
    centroid = np.mean(points, axis=0)
    points -= centroid
    max_dist = np.max(np.sqrt(np.sum(points**2, axis=1)))
    points /= max_dist
    
    combined = np.hstack((points, colors))
    tensor = torch.from_numpy(combined).float().transpose(0, 1).unsqueeze(0)
    return tensor

# CNN 모델 정의
class SmallPointCloudCNN(nn.Module):
    def __init__(self, num_classes, input_channels=6):
        super(SmallPointCloudCNN, self).__init__()
        self.conv1 = nn.Conv1d(input_channels, 32, 1)
        self.conv2 = nn.Conv1d(32, 64, 1)
        self.conv3 = nn.Conv1d(64, 128, 1)

        self.fc1 = nn.Linear(128, 64)
        self.fc2 = nn.Linear(64, num_classes)

        self.bn1 = nn.BatchNorm1d(32)
        self.bn2 = nn.BatchNorm1d(64)
        self.bn3 = nn.BatchNorm1d(128)
        
        self.dropout = nn.Dropout(0.3)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 128)
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = self.fc2(x)
        return F.log_softmax(x, dim=1)

# 모델 저장 함수
def save_model(model, file_path):
    torch.save(model.state_dict(), file_path)
    rospy.loginfo(f"모델이 {file_path}에 저장되었습니다.")

# 모델 로드 함수
def load_model(file_path):
    global model
    model.load_state_dict(torch.load(file_path))
    model.eval()
    rospy.loginfo(f"모델 {file_path}이(가) 성공적으로 불러와졌습니다.")

# .pcd 파일로부터 데이터를 준비하는 함수
# .pcd 파일로부터 데이터를 준비하는 함수
def prepare_data(data_dir, labels, target_size=128):
    data = []
    targets = []
    for idx, label in enumerate(labels):
        folder_path = os.path.join(data_dir, label)
        for file_name in os.listdir(folder_path):
            if file_name.endswith('.pcd'):
                file_path = os.path.join(folder_path, file_name)
                pcd = o3d.io.read_point_cloud(file_path)
                points = np.asarray(pcd.points)
                colors = np.asarray(pcd.colors)

                # 전처리 후 데이터 수집
                tensor = preprocess_small_pointcloud(points, colors, target_size=target_size)
                data.append(tensor)
                targets.append(idx)
    
    # 데이터를 Tensor로 변환
    data = torch.cat(data)
    targets = torch.tensor(targets)
    return data, targets


# 학습 함수 (학습 및 검증 데이터로 분리하여 사용)
def train_model(model, data, targets, num_epochs=500, learning_rate=0.0001):
    # 학습 데이터와 검증 데이터 분리 (80% 학습, 20% 검증)
    train_data, val_data, train_targets, val_targets = train_test_split(
        data, targets, test_size=0.2, random_state=42, stratify=targets
    )
    
    model.train()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    loss_fn = nn.CrossEntropyLoss()

    for epoch in range(num_epochs):
        optimizer.zero_grad()
        outputs = model(train_data.to(device))
        loss = loss_fn(outputs, train_targets.to(device))
        loss.backward()
        optimizer.step()
        rospy.loginfo(f"에포크 {epoch + 1}/{num_epochs}, 손실: {loss.item()}")
    
    rospy.loginfo("모델 학습 완료!")

    # 검증 데이터로 성능 평가
    model.eval()
    with torch.no_grad():
        val_outputs = model(val_data.to(device))
        _, val_predictions = torch.max(val_outputs, 1)
        accuracy = accuracy_score(val_targets.cpu(), val_predictions.cpu())
        rospy.loginfo(f"검증 데이터 정확도: {accuracy * 100:.2f}%")

# ROS 메시지를 처리하여 분류 수행
def point_cloud_callback(point_cloud_msg):
    global model, labels
    
    # 포인트 클라우드 데이터를 처리
    cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(point_cloud_msg)
    points = np.column_stack((cloud_arr['x'], cloud_arr['y'], cloud_arr['z']))
    rgb_float = cloud_arr['rgb']
    rgb_int = rgb_float.view(np.uint32)
    r = (rgb_int >> 16) & 0xFF
    g = (rgb_int >> 8) & 0xFF
    b = rgb_int & 0xFF
    colors = np.column_stack((r, g, b)) / 255.0
    
    # 전처리 및 텐서 변환
    input_tensor = preprocess_small_pointcloud(points, colors)
    
    # 모델에 입력 및 예측 수행
    with torch.no_grad():
        input_tensor = input_tensor.to(device)
        output = model(input_tensor)
        probabilities = F.softmax(output, dim=1)  # 확률 계산
        predicted_probs, predicted_classes = torch.max(probabilities, 1)
    
    # 각 클래스별 확률 출력
    for i, prob in enumerate(probabilities[0]):
        rospy.loginfo(f"{labels[i]}일 확률: {prob.item() * 100:.2f}%")
    
    predicted_label = labels[predicted_classes.item()]
    rospy.loginfo(f"예측된 클래스: {predicted_label}")
    
    # 결과를 ROS 메시지로 퍼블리시
    result_msg = String()
    result_msg.data = predicted_label
    result_publisher.publish(result_msg)

if __name__ == "__main__":
    try:
        rospy.init_node('small_pointcloud_cnn_classifier')
        
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model = SmallPointCloudCNN(num_classes=9).to(device)
        
        labels = ['green_cylinder', 'green_sphere', 'green_box', 'red_box', 'red_cylinder', 'red_sphere', 'yellow_box', 'yellow_cylinder', 'yellow_sphere']
        
        model_path = rospy.get_param('~model_path', '/home/kriso/sf_ws/src/data_mach/model.pth')
        
        # 학습된 모델이 없으면 .pcd 파일을 불러와 학습
        if not os.path.exists(model_path):
            rospy.loginfo("학습된 모델이 없습니다. .pcd 파일을 사용하여 모델을 학습합니다.")
            data_dir = '/home/kriso/sf_ws/src/data_mach'
            data, targets = prepare_data(data_dir, labels)
            train_model(model, data, targets)
            save_model(model, model_path)
        else:
            load_model(model_path)
        
        result_publisher = rospy.Publisher('/object_classification', String, queue_size=10)
        rospy.Subscriber('/final_result', PointCloud2, point_cloud_callback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("노드가 종료되었습니다.")
