### A-LOAM, LiDAR-카메라 융합, KNN을 이용한 객체 분류 (Ubuntu 20.04, ROS Noetic)

#### 문의 사항: peicai69@naver.com

---

먼저, 이 프로그램이 제작될 수 있었던 것은 아래의 기여자들 덕분입니다:

- **A-LOAM (Modifier: Tong Qin, Shaozu Cao)**  
  [https://github.com/HKUST-Aerial-Robotics/A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)

- **Turtlebot3 Velodyne (Tevhit Karsli)**  
  [https://github.com/Tevhit/pcl_velodyne_ws/tree/main/src/turtlebot3_velodyne](https://github.com/Tevhit/pcl_velodyne_ws/tree/main/src/turtlebot3_velodyne)

---

### 프로그램 실행 전 준비 사항

#### Step 1: 종속성 프로그램 설치

아래 링크를 참조하여 설치를 진행하세요:  
[https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start)

다음 명령어를 실행합니다:

```bash
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh
```

이후, 아래의 ROS 패키지를 설치하세요:

```bash
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

추가로 다음 종속성들을 설치하세요:

```bash
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
$ sudo apt install gnome-terminal
$ pip install open3d
```

`.bashrc` 파일을 수정하여 Turtlebot3 모델을 설정하세요:

```bash
export TURTLEBOT3_MODEL=waffle
```

마지막으로, `/opt/ros/noetic/lib/python3/dist-packages/ros_numpy/point_cloud2.py` 파일에서 아래 코드를 수정합니다:

```python
# 기존 코드
def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):

# 수정된 코드
def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float64):
```

#### Step 2: 파일 수정

1. **.pcd 파일 생성**  
   직접 .pcd 파일을 생성하고 싶다면, `create_data/src/run_creat_data.py` 파일에서 `sdf_model` 변수를 수정하세요.  
   이 변수에 대한 예시는 `sdf/block.sdf` 파일에 있습니다.

2. **디렉토리 경로 수정**  
   `pointcombine/src/com3.py` 파일에서 아래 줄을 본인의 디렉토리 경로에 맞게 수정하세요:

   - 59번째 줄: `directory = "/home/<username>/catkin_ws/src/data_mach/green_box"`
   - 64번째 줄과 67번째 줄도 동일하게 수정하세요.

3. **프로그램 실행**  
   프로그램을 실행하고 싶다면, `mach_learn/src/knn_run.py` 파일에서 `data_dir` 경로를 수정하세요:

   - 149번째 줄: `data_dir = '/home/<username>/catkin_ws/src'`

---

### Step 3: 프로그램 실행 (6개의 터미널 필요)

결과값을 얻기 위해 프로그램을 실행하려면, 다음 단계를 따르세요:

1. **첫 번째 터미널**:  
   빈 월드를 실행합니다:

   ```bash
   roslaunch turtlebot3_velodyne_gazebo turtlebot3_empty_world.launch
   ```

   그 후, 주변 블럭을 복사하여 빨간 점으로 붙여넣기 합니다.

2. **두 번째 터미널**:  
   KNN 분류 스크립트를 실행합니다:

   ```bash
   rosrun mach_learn knn_run.py
   ```

   버그가 발생하면 `data_study` 경로를 확인하고, 파일 권한을 제대로 주었는지 확인하세요 (`chmod` 사용).

3. **세 번째 터미널**:  
   LiDAR-카메라 캘리브레이션 스크립트를 실행합니다:

   ```bash
   rosrun lidar_camera_calibration lidar_camera_calibration.py
   ```

   빨간 점으로 제대로 붙여넣기를 했다면, 버그는 발생하지 않습니다.

4. **네 번째 터미널**:  
   데이터 생성 스크립트를 실행합니다:

   ```bash
   rosrun create_data run.py
   ```

5. **다섯 번째 터미널**:  
   포인트 결합 실행 파일을 실행합니다:

   ```bash
   roslaunch pointcombine run.launch
   ```

6. **여섯 번째 터미널**:  
   로봇의 모션 제어를 위한 `/cmd_vel` 토픽을 퍼블리시하고, `Ctrl + C`를 누른 후 다음 서비스를 실행합니다:

   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear:
     x: 0.3
     y: 0.0
     z: 0.0
   angular:
     x: 0.0
     y: 0.0
     z: 0.12"
   ```

   이후, 다음 서비스를 실행하세요:

   ```bash
   rosservice call /create_data_start
   ```



### Object Classification using A-LOAM, LiDAR-Camera Fusion, and KNN on Ubuntu 20.04 with ROS Noetic

#### Contact: peicai69@naver.com

---

First of all, I would like to thank the following contributors who made this project possible:

- **A-LOAM (Modifier: Tong Qin, Shaozu Cao)**  
  [https://github.com/HKUST-Aerial-Robotics/A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)

- **Turtlebot3 Velodyne (Tevhit Karsli)**  
  [https://github.com/Tevhit/pcl_velodyne_ws/tree/main/src/turtlebot3_velodyne](https://github.com/Tevhit/pcl_velodyne_ws/tree/main/src/turtlebot3_velodyne)

---

### Before Running the Program

#### Step 1: Install Dependencies

Refer to the following link for installation instructions:  
[https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start)

Run the following commands:

```bash
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh
```

Then, install the following ROS packages:

```bash
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

Next, install additional dependencies:

```bash
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
$ sudo apt install gnome-terminal
$ pip install open3d
```

Modify the `.bashrc` file to set the Turtlebot3 model:

```bash
export TURTLEBOT3_MODEL=waffle
```

Finally, modify the following line in the `point_cloud2.py` file located at `/opt/ros/noetic/lib/python3/dist-packages/ros_numpy/`:

```python
# Original code
def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):

# Modified code
def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float64):
```

#### Step 2: Modify Some Files

1. **Creating .pcd Files**  
   If you want to create .pcd files yourself, modify the `sdf_model` variable in `create_data/src/run_creat_data.py`.  
   An example is available in the `sdf/block.sdf` file.

2. **Modifying Directory Paths**  
   In `pointcombine/src/com3.py`, modify the following lines to your specific directories:

   - Line 59: `directory = "/home/<username>/catkin_ws/src/data_mach/green_box"`
   - Also modify lines 64 and 67 accordingly.

3. **Running the Program**  
   If you just want to run the program, modify the `data_dir` path in `mach_learn/src/knn_run.py`:

   - Line 149: `data_dir = '/home/<username>/catkin_ws/src'`

---

### Step 3: Running the Program (Requires 6 Terminals)

If you simply want to execute the program to obtain results, follow these steps:

1. **First Terminal**:  
   Run the empty world simulation:

   ```bash
   roslaunch turtlebot3_velodyne_gazebo turtlebot3_empty_world.launch
   ```

   Then, copy the surrounding blocks and attach them as red points.

2. **Second Terminal**:  
   Run the KNN classification script:

   ```bash
   rosrun mach_learn knn_run.py
   ```

   If you encounter any bugs, verify the `data_study` path and ensure you have given appropriate file permissions (use `chmod`).

3. **Third Terminal**:  
   Run the LiDAR-camera calibration script:

   ```bash
   rosrun lidar_camera_calibration lidar_camera_calibration.py
   ```

   As long as the red point attachment was done correctly, no bugs should occur.

4. **Fourth Terminal**:  
   Run the data creation script:

   ```bash
   rosrun create_data run.py
   ```

5. **Fifth Terminal**:  
   Run the point combination launch file:

   ```bash
   roslaunch pointcombine run.launch
   ```

6. **Sixth Terminal**:  
   Publish the `/cmd_vel` topic for robot motion control, then press `Ctrl + C` before running the service:

   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear:
     x: 0.3
     y: 0.0
     z: 0.0
   angular:
     x: 0.0
     y: 0.0
     z: 0.12"
   ```

   Afterward, run the following service:

   ```bash
   rosservice call /create_data_start
   ```



