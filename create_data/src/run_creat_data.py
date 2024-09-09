#!/usr/bin/env python3

import rospy
import random
import os
import subprocess
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel
from rospy import AnyMsg
import time

global model_name
model_name = 'green_box'

global sdf_model
sdf_model = """<sdf version="1.6">
                    <model name="green_box">
                    <static>0</static>
                    <pose>0 0 0.5 0 0 0</pose>
                    <link name="box_link">
                        <collision name="box_collision">
                        <geometry>
                            <box>
                            <size>1.0 1.0 1.0</size>
                            </box>
                        </geometry>
                        <surface>
                            <contact>
                            <collide_bitmask>0x01</collide_bitmask>
                            </contact>
                        </surface>
                        </collision>
                        <visual name="box_visual">
                        <geometry>
                            <box>
                            <size>1.0 1.0 1.0</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0 1 0 1</ambient>
                            <diffuse>0 1 0 1</diffuse>
                        </material>
                        </visual>
                    </link>
                    </model>
                </sdf>"""

# Function to spawn the model at a random position
def spawn_model(model_name, model_xml, x_range, y_range):
    rospy.loginfo(f"Spawning model: {model_name}")
    
    x = random.uniform(x_range[0], x_range[1])
    y = random.uniform(y_range[0], y_range[1])
    
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = 0
    initial_pose.orientation.w = 1

    try:
        spawn_service(model_name, model_xml, '', initial_pose, 'world')
        rospy.loginfo(f"{model_name} spawned at x: {x}, y: {y}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn model: {e}")

# Function to delete the model
def delete_model(model_name):
    rospy.loginfo(f"Deleting model: {model_name}")
    try:
        delete_service(model_name)
        rospy.loginfo(f"{model_name} deleted successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to delete model: {e}")

# Function to publish to /cmd_vel topic
def publish_cmd_vel():
    rospy.loginfo("Publishing to /cmd_vel")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = 0.12
    pub.publish(twist)
    rospy.loginfo("Published /cmd_vel")

# Function to check /laser_cloud_map for data, and determine if it is working properly
def check_laser_cloud_map():
    rospy.loginfo("Checking /laser_cloud_map for 5 messages within 5 seconds.")
    
    message_count = 0
    start_time = time.time()

    def callback(msg):
        nonlocal message_count
        message_count += 1

    sub = rospy.Subscriber('/laser_cloud_map', AnyMsg, callback)

    while time.time() - start_time < 5:
        if message_count >= 2:
            rospy.loginfo(f"Received {message_count} messages from /laser_cloud_map. A-LOAM is working properly.")
            sub.unregister()  # Unregister the subscriber after success
            return True
        rospy.sleep(0.1)

    rospy.logwarn(f"Only received {message_count} messages from /laser_cloud_map in 5 seconds. A-LOAM may not be working properly.")
    sub.unregister()  # Unregister the subscriber after failure
    return False

# Function to start the A-LOAM launch file in a new terminal window
def start_aloam_launch():
    rospy.loginfo("Starting A-LOAM launch file in a new terminal using gnome-terminal")
    process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "roslaunch /home/kriso/sf_ws/src/A-LOAM/launch/aloam_velodyne_VLP_16.launch; exec bash"])
    rospy.loginfo("A-LOAM launch started successfully in a new terminal")
    return process

# Function to stop the A-LOAM launch file (terminate subprocess and close terminal)
def stop_aloam_launch(process):
    rospy.loginfo("Stopping A-LOAM launch")

    # 강제로 A-LOAM 노드를 종료
    subprocess.call(["rosnode", "kill", "/alaserMapping"])
    subprocess.call(["rosnode", "kill", "/alaserOdometry"])
    subprocess.call(["rosnode", "kill", "/ascanRegistration"])

    # 기존 A-LOAM 프로세스 종료
    process.kill()
    process.terminate()
    process.wait()
    rospy.loginfo("A-LOAM launch stopped successfully, terminal closed.")

# Callback for /create_data_start service
def start_process(req):
    x_range = [-0.3, 0.3]
    y_range = [2.2, 2.8]
    
    rospy.loginfo("/create_data_start service called")
    for i in range(50):
        rospy.loginfo(f"Iteration {i+1}/50")

        # Spawn the red_sphere
        spawn_model(model_name, sdf_model, x_range, y_range)

        # Publish to /cmd_vel
        publish_cmd_vel()

        # Start A-LOAM in a new terminal
        aloam_process = start_aloam_launch()

        # Check if /laser_cloud_map has sufficient data
        while not check_laser_cloud_map():
            rospy.loginfo("No sufficient data from /laser_cloud_map, waiting 3 seconds and restarting A-LOAM")
            rospy.sleep(3)  # Wait for 3 seconds before restarting A-LOAM
            stop_aloam_launch(aloam_process)
            rospy.sleep(3)
            aloam_process = start_aloam_launch()

        # Wait for 100 seconds before processing
        rospy.loginfo("Waiting for 50 seconds")
        rospy.sleep(50)

        # Call /start_processing service
        rospy.loginfo("Calling /start_processing service")
        try:
            start_processing_service()
            rospy.loginfo("/start_processing service called")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /start_processing: {e}")

        # Wait for 60 seconds
        rospy.loginfo("Waiting for 57 seconds")
        rospy.sleep(57)

        # Stop A-LOAM and close terminal
        stop_aloam_launch(aloam_process)

        # Delete the red_sphere
        delete_model(model_name)

        # Wait for 3 seconds before next iteration
        rospy.loginfo("Waiting for 3 seconds")
        rospy.sleep(3)

    

# Main function
def main():
    rospy.init_node('create_data_node')
    rospy.loginfo("create_data_node started")

    # Wait for the necessary services
    rospy.loginfo("Waiting for services to be available")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.wait_for_service('/start_processing')

    # Define the service proxies
    global start_processing_service
    start_processing_service = rospy.ServiceProxy('/start_processing', Empty)
    global spawn_service
    spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    global delete_service
    delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    # Set up the /create_data_start service
    rospy.Service('/create_data_start', Empty, start_process)

    rospy.loginfo("/create_data_start service ready")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException caught! Shutting down.")
