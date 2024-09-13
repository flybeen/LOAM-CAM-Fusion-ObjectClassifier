#!/usr/bin/env python3

import rospy
import subprocess
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from rospy import AnyMsg
import time

#==================================================#
# 이 run_creat_data.py가 실행되면 로봇이 움직여야하는데....
# 왜인지 모르겠으나, 작동이 안됨. 터미널에서 직업 입력
#==================================================#
def publish_cmd_vel():
    rospy.loginfo("Publishing to /cmd_vel")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = 0.12
    pub.publish(twist)
    rospy.loginfo("Published /cmd_vel")

#=============================================================#
# A-LOAM이 제대로 실행되나 확인하는 함수, 가끔씩 실행이 안되기 때문에 제작
# A-LOAM이 실행되면 최종적으로 /laser_cloud_map 토픽이 출력됨
# 따라서 토픽이 출력이 안되면 -> A-LOAM이 제대로 실행되지 않는다고 판단.
#=============================================================#
def check_laser_cloud_map():
    rospy.loginfo("Checking /laser_cloud_map for 5 messages within 5 seconds.")
    
    message_count = 0
    start_time = time.time()

    def callback(msg):          # /laser_cloud_map 토픽이 들어면 실행
        nonlocal message_count  #nonlocal는 함수내 에 정의된 함수에서 외분 함수의 변수에 접근하기 위해
        message_count += 1 

    sub = rospy.Subscriber('/laser_cloud_map', AnyMsg, callback)

    # 5초동안 2개 이상의 /laser_cloud_map 토픽이 들어온다면 a-loam 작동 확인 완료
    while time.time() - start_time < 5:
        if message_count >= 2:
            rospy.loginfo(f"Received {message_count} messages from /laser_cloud_map. A-LOAM is working properly.")
            sub.unregister()  # 구독자 해제 -> 잘 들어오는것을 확인했기 때문에(리소스 절약)
            return True
        rospy.sleep(0.1)

    rospy.logwarn(f"Only received {message_count} messages from /laser_cloud_map in 5 seconds. A-LOAM may not be working properly.")
    sub.unregister()  # 구독자 해제 -> 잘 들어오는것을 확인했기 때문에(리소스 절약)
    return False


#=================================================#
# A-LOAM 실행함수, gnome-terminal를 이용함.
# 터미네이터로 변경 가능할것임
#=================================================#
def start_aloam_launch():
    rospy.loginfo("Starting A-LOAM launch file in a new terminal using gnome-terminal")
    process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "roslaunch /home/kriso/sf_ws/src/A-LOAM/launch/aloam_velodyne_VLP_16.launch; exec bash"])
    rospy.loginfo("A-LOAM launch started successfully in a new terminal")
    return process

#======================================================#
#       필요한 데이터를 얻었다면 A-LOAM 종료함수
# 굳이 종료하는 이유는, 모델이 삭제되었다 다시 생성하면 기존의 
# path와 맵이 이상하게 꼬여버리기 때문에
#======================================================#
def stop_aloam_launch(process):
    rospy.loginfo("Stopping A-LOAM launch")

    # 강제로 A-LOAM 노드를 종료(이렇게 해야지 제대로 종료됨)
    subprocess.call(["rosnode", "kill", "/alaserMapping"])
    subprocess.call(["rosnode", "kill", "/alaserOdometry"])
    subprocess.call(["rosnode", "kill", "/ascanRegistration"])

    # 기존 A-LOAM 프로세스 종료
    process.kill()
    process.terminate()
    process.wait()
    rospy.loginfo("A-LOAM launch stopped successfully, terminal closed.")


#=====================================================#
# /create_data_start 서비스가 call되면 작동하는 함수
#=====================================================#
def start_process(req):
    rospy.loginfo("/create_data_start service called")

    #작동안됨....
    #publish_cmd_vel()

    # step 1, a-loam 실행
    aloam_process = start_aloam_launch()

    # step 2, a-loam 작동 여부 확인
    while not check_laser_cloud_map():
        rospy.loginfo("No sufficient data from /laser_cloud_map, waiting 3 seconds and restarting A-LOAM")
        rospy.sleep(3)  # Wait for 3 seconds before restarting A-LOAM
        stop_aloam_launch(aloam_process)
        rospy.sleep(3)
        aloam_process = start_aloam_launch()

    # step 3, 50초 대기, 먼저, 맵핑을 하는 시간 필요,  
    # twist.linear.x = 0.3 twist.angular.z = 0.12 이라면 한 바퀴 도는데 걸리는시간
    # 50초 정도, 속도를 빠르게 하면 줄여도됨
    rospy.loginfo("Waiting for 50 seconds")
    rospy.sleep(50)

    # step 4, 한바퀴 다 돌았으면 /start_processing 서비스 call하기
    # 색칠하는 작업임. 자세한건 rviz에서 /step_2_point 확인하면 이해가능
    rospy.loginfo("Calling /start_processing service")
    try:
        start_processing_service()
        rospy.loginfo("/start_processing service called")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call /start_processing: {e}")

    # step 5, 색칠하는 작업 걸리는 시간 57초
    # 물론 똑같이 한바퀴이기 때문에 50초이긴한데, 여유를 조금 주었음
    rospy.loginfo("Waiting for 57 seconds")
    rospy.sleep(57)

    # step 6, 색칠 끝나면, a-loam종료
    stop_aloam_launch(aloam_process)

    # step 7, 여유시간 3초
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


    # 이 /create_data_start가 핵심. 즉, 터미널에서 /create_data_start가 call되야지 모든게 실행
    rospy.loginfo("/create_data_start service ready")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException caught! Shutting down.")
