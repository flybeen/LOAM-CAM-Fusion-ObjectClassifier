import cv2
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, Image, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_conversions
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
        self.lidar_data = None
        self.camera_data = None
        self.overlay_image = None
        self.overlayed_points = None

        self.transform_pub = rospy.Publisher('/lidar_camera_transform', std_msgs.msg.Float64MultiArray, queue_size=10)
        self.image_pub = rospy.Publisher('/overlay_image', Image, queue_size=1)
        self.pointcloud_pub = rospy.Publisher('/overlay_pointcloud', PointCloud2, queue_size=1)
        self.transformed_cloud_pub = rospy.Publisher('/transformed_pointcloud', PointCloud2, queue_size=1)

        #테스트용
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.camera_matrix = np.array([[1131.20, 0.000000, 640.0],
                                       [0.000000, 1131.20, 360.0],
                                       [0.000000, 0.000000, 1.000000]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        self.rotation_matrix = np.array([
            [1, 0,  0],
            [0,  0, -1],
            [0,  1,  0]
        ])

        self.translation_vector = np.array([0, 0.1, 0.1])

        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()

        self.last_lidar_update_time = time.time()
        self.lidar_update_interval = 0.5

    def display_loop(self):
        while not rospy.is_shutdown():
            if self.overlay_image is not None:
                cv2.imshow('Image with Lidar Points', self.overlay_image)
                cv2.waitKey(1)
        cv2.destroyAllWindows()

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

    def camera_callback(self, data):
        try:
            self.camera_data = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.perform_calibration()
        except Exception as e:
            rospy.logerr(f"Error in camera_callback: {e}")

    def perform_calibration(self):
        if self.lidar_data is None or self.camera_data is None:
            return

        try:
            points = np.array(list(pc2.read_points(self.lidar_data, skip_nans=True, field_names=("x", "y", "z", "intensity"))))

            if points.shape[1] == 4:
                xyz = points[:, :3]
                intensities = points[:, 3]
            else:
                xyz = points
                intensities = np.zeros(points.shape[0])

            try:
                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = self.rotation_matrix
                transformation_matrix[:3, 3] = self.translation_vector

                self.publish_transform(self.rotation_matrix, self.translation_vector)

                self.overlay_points_on_image(xyz, transformation_matrix, intensities)

                if self.overlay_image is not None:
                    image_msg = self.bridge.cv2_to_imgmsg(self.overlay_image, "bgr8")
                    self.image_pub.publish(image_msg)
            except Exception as e:
                rospy.logerr(f"Error in applying transformation: {e}")
        except Exception as e:
            rospy.logerr(f"Error in perform_calibration: {e}")

    def intensities_to_rgb(self, intensities):
        min_intensity = np.min(intensities)
        max_intensity = np.max(intensities)

        if max_intensity - min_intensity == 0:
            norm_intensities = np.zeros_like(intensities)
        else:
            norm_intensities = (intensities - min_intensity) / (max_intensity - min_intensity) * 255
        
        norm_intensities = norm_intensities.astype(np.uint8)
        colormap = cv2.applyColorMap(norm_intensities.reshape(-1, 1), cv2.COLORMAP_JET)
        return colormap[:, 0, :]

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

    def overlay_points_on_image(self, points, transformation_matrix, intensities):
        try:
            points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
            transformed_points_homogeneous = points_homogeneous.dot(transformation_matrix.T)
            transformed_points = transformed_points_homogeneous[:, :3] / transformed_points_homogeneous[:, 3][:, np.newaxis]

            front_points_idx = (transformed_points[:, 2] > 1.5) & (transformed_points[:, 2] < 4) & (transformed_points[:, 0] > -1) & (transformed_points[:, 0] < 1) & (transformed_points[:, 1] < 0.2)
            front_points = transformed_points[front_points_idx]
            front_intensities = intensities[front_points_idx]

            front_colors = self.intensities_to_rgb(front_intensities)
            image_points, _ = cv2.projectPoints(front_points, np.zeros((3, 1)), np.zeros((3, 1)), self.camera_matrix, self.dist_coeffs)

            self.overlay_image = self.camera_data.copy()
            valid_points = 0
            invalid_points = 0
            new_points = []

            inverse_transformation_matrix = np.linalg.inv(transformation_matrix)
            restored_points_homogeneous = np.hstack((front_points, np.ones((front_points.shape[0], 1))))
            restored_points_homogeneous = restored_points_homogeneous.dot(inverse_transformation_matrix.T)
            restored_points = restored_points_homogeneous[:, :3] / restored_points_homogeneous[:, 3][:, np.newaxis]

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

    def rgb_to_float(self, r, g, b):
        return struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]

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
