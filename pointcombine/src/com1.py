#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import tf.transformations
import struct

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('pointcloud_transformer', anonymous=True)
        
        # Ensure ROS uses simulation time
        rospy.set_param('/use_sim_time', True)

        # Subscribers
        self.pc_sub = rospy.Subscriber('/transformed_pointcloud', PointCloud2, self.pc_callback)
        self.path_sub = rospy.Subscriber('/aft_mapped_path', Path, self.path_callback)

        # Publisher
        self.pc_pub = rospy.Publisher('/step_1_point', PointCloud2, queue_size=10)

        # Buffer to store the latest point cloud and path
        self.latest_pc = None
        self.latest_path = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def sanitize_frame_id(self, frame_id):
        """Remove leading slashes from frame_id."""
        if frame_id.startswith('/'):
            return frame_id[1:]
        return frame_id

    def pc_callback(self, msg):
        self.latest_pc = msg
        self.latest_pc.header.frame_id = self.sanitize_frame_id(self.latest_pc.header.frame_id)
        # Processing is now triggered by path_callback
        

    def path_callback(self, msg):
        self.latest_path = msg
        self.latest_path.header.frame_id = self.sanitize_frame_id(self.latest_path.header.frame_id)
        
        # Ensure the latest_pc is available before proceeding
        if self.latest_pc is not None:
            # Directly call the process_data function
            self.process_data()
        

    def process_data(self):
        if self.latest_pc is None or self.latest_path is None:
            return

        # Get the last pose from the path
        last_pose = self.latest_path.poses[-1].pose
        translation = last_pose.position
        rotation = last_pose.orientation

        # Convert rotation quaternion to a transformation matrix
        transform_matrix = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        transform_matrix[0:3, 3] = [translation.x, translation.y, translation.z]

        # Extract points from the PointCloud2 message, including RGB values
        points = list(point_cloud2.read_points(self.latest_pc, field_names=("x", "y", "z", "rgb"), skip_nans=True))

        # Convert the points to a numpy array to apply the transformation
        points = np.array(points)
        points_homogeneous = np.ones((points.shape[0], 4))
        points_homogeneous[:, 0:3] = points[:, 0:3]  # Copy XYZ values

        # Apply the transformation to the XYZ values
        transformed_points = np.dot(transform_matrix, points_homogeneous.T).T[:, 0:3]

        # Reconstruct the list of transformed points with the original RGB values
        transformed_points_with_rgb = []
        for (x, y, z), rgb in zip(transformed_points, points[:, 3]):
            # Convert the float32 RGB value to uint32
            rgb_uint32 = struct.unpack('I', struct.pack('f', rgb))[0]
            # Extract the RGB components
            r = (rgb_uint32 >> 16) & 0xFF
            g = (rgb_uint32 >> 8) & 0xFF
            b = rgb_uint32 & 0xFF
            # Convert back to float32
            rgb_float = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
            transformed_points_with_rgb.append([x, y, z, rgb_float])

        # Create a new PointCloud2 message
        header = Header()
        header.stamp = self.latest_pc.header.stamp  # Maintain the original timestamp
        header.frame_id = "/camera_init"  # Set the frame to camera_init after transformation
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)  # Use FLOAT32 for RGB
        ]
        transformed_pc_msg = point_cloud2.create_cloud(header, fields, transformed_points_with_rgb)

        # Publish the transformed point cloud
        self.pc_pub.publish(transformed_pc_msg)

       
if __name__ == '__main__':
    try:
        PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass