#!/usr/bin/env python3

import rospy
import random
import subprocess
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from rospy import AnyMsg
import time

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
    rospy.loginfo("/create_data_start service called")

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

    # Wait for 50 seconds before processing
    rospy.loginfo("Waiting for 50 seconds")
    rospy.sleep(50)

    # Call /start_processing service
    rospy.loginfo("Calling /start_processing service")
    try:
        start_processing_service()
        rospy.loginfo("/start_processing service called")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call /start_processing: {e}")

    # Wait for 57 seconds
    rospy.loginfo("Waiting for 57 seconds")
    rospy.sleep(57)

    # Stop A-LOAM and close terminal
    stop_aloam_launch(aloam_process)

    # Wait for 3 seconds before next iteration
    rospy.loginfo("Waiting for 3 seconds")
    rospy.sleep(3)

# Main function
def main():
    rospy.init_node('create_data_node')
    rospy.loginfo("create_data_node started")

    # Wait for the necessary services
    rospy.loginfo("Waiting for services to be available")
    rospy.wait_for_service('/start_processing')

    # Define the service proxies
    global start_processing_service
    start_processing_service = rospy.ServiceProxy('/start_processing', Empty)

    # Set up the /create_data_start service
    rospy.Service('/create_data_start', Empty, start_process)

    rospy.loginfo("/create_data_start service ready")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException caught! Shutting down.")
