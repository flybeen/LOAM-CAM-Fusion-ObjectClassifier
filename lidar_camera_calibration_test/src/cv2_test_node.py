#!/usr/bin/env python3
import rospy
import cv2

def test_cv2():
    try:
        # 이미지 파일 경로 설정
        image_path = '/home/kriso/Pictures/Screenshot from 2024-07-23 13-42-07.png'
        
        # 이미지 읽기
        image = cv2.imread(image_path)
        
        # 이미지가 제대로 읽혔는지 확인
        if image is None:
            rospy.logerr("Failed to load image")
        else:
            rospy.loginfo("Image loaded successfully")
            # 이미지 표시
            cv2.imshow('Test Image', image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    except Exception as e:
        rospy.logerr(f"Error in test_cv2: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('cv2_test_node', anonymous=True)
    test_cv2()
    rospy.spin()
