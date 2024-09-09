
A-LOAM과 lidar-camera-fusion, KNN을 이용한 객체 분류
ubuntu-20.04 ROS-noetic

문의 사항 : peicai69@naver.com

먼저, 이 프로그램을 제작 될 수 있었던건, 아래의 분들 덕분입니다.

A-LOAM(Modifier: Tong Qin, Shaozu Cao)
https://github.com/HKUST-Aerial-Robotics/A-LOAM

turtlebot3_velodyne(Tevhit Karsli)
https://github.com/Tevhit/pcl_velodyne_ws/tree/main/src/turtlebot3_velodyne



실행하기에 앞서, 준비해야합니다.

첫번째는 종속성 프로그램을 설치하세요.
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start (출처)

$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh

$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
  
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3

export TURTLEBOT3_MODEL=waffle    (~/.bashrc)

sudo apt install gnome-terminal
pip install open3d


# opt/ros/noetic/lib/python3/dist-packages/ros_numpy/point_cloud2.py 
# def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float): # 기존 코드
# def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float64):  # 수정된 코드으로 변경하십쇼


두번째는 일부 파일 수정이 필요합니다.

만약, 당신이 .pcd파일을 직접 만들기 원한다면
create_data/src/run_creat_data.py  에서 변수 sdf_model를 직접 수정해야합니다.
이 변수는 sdf/block.sdf 파일에 예시가 있습니다.
pointcombine/src/com3.py 에서 59 directory = "/home/<username>/catkin_ws/src/data_mach/green_box"과 64, 67 줄을 직접 수정하십쇼


만약, 실행만 한다면
mach_learn/src/knn_run.py에서 149  data_dir = '/home/<username>/catkin_ws/src' 을 수정하십쇼



세번째, 실행은 최소 6개의 터미널이 필요합니다.
	당신이 실행만 하여 결과값을 얻고 싶다면
	첫번째 터미널에서 roslaunch turtlebot3_velodyne_gazebo turtlebot3_empty_world.launch
	을 실행후, 주변 블럭에서 복사를 하여 빨간색 점으로 붙여놓기를 하세요.
	
	두번째 터미널에서 rosrun mach_learn knn_run.py
	버그가 난다면 data_study 경로를 확인하시고, 파일에 권한(chmod)를 주었는지 확인하세요.
	
	세번째 터미널에서는 rosrun lidar_camera_calibration lidar_camera_calibration.py
	만약 빨간색 점으로 붙여놓기를 잘했다면 버그는 발생하지 않을 것입니다.
	
	네번째 터미널에서는 rosrun create_data run.py

	다섯번째 터미널에서는 roslaunch pointcombine run.launch

	여섯번째 터미널에서는 rostopic pub /cmd_vel geometry_msgs/Twist "linear:
			  x: 0.3
			  y: 0.0
			  z: 0.0
			angular:
			  x: 0.0
			  y: 0.0
			  z: 0.12"
	을 먼저 실행 후, crtl + C를 누른 후에, rosservice call /create_data_start


====================================================================================================================================================================
English

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



