#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import struct
import threading
import time
import open3d as o3d
from std_srvs.srv import Empty, EmptyResponse

class PointCloudColorMapper:
    def __init__(self):
        rospy.init_node('pointcloud_color_mapper', anonymous=True)
        
        # Subscribers
        self.map_sub = None  # Initialize subscriber later when service is called
        self.step_1_sub = None  # Step 1 subscriber will be initialized upon start

        # Publishers
        self.step_2_pub = rospy.Publisher('/step_2_point', PointCloud2, queue_size=10)
        self.temp_pub = rospy.Publisher('/temp', PointCloud2, queue_size=10)

        # Storage for point clouds
        self.laser_cloud_map_storage = None
        self.step_1_point_temp = None

        # Control flags
        self.is_running = False
        self.processing_lock = threading.Lock()

        # Service to start the process
        self.start_service = rospy.Service('start_processing', Empty, self.start_processing)

        rospy.loginfo("PointCloudColorMapper node initialized. Waiting for start command...")

    def start_processing(self, req):
        with self.processing_lock:
            if not self.is_running:
                rospy.loginfo("Start command received via ROS service. Subscribing to /laser_cloud_map...")
                
                # Subscribe to /laser_cloud_map once when the service is called
                self.map_sub = rospy.Subscriber('/laser_cloud_map', PointCloud2, self.map_callback)
                
                return EmptyResponse()
            else:
                rospy.logwarn("Processing is already running.")
                return EmptyResponse()

    def map_callback(self, msg):
        with self.processing_lock:
            if self.laser_cloud_map_storage is None:
                rospy.loginfo("Received /laser_cloud_map point cloud. Storing and ready to start processing.")
                map_points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
                if map_points.size == 0:
                    rospy.logwarn("Received /laser_cloud_map is empty. Waiting for non-empty point cloud.")
                    return
                white_rgb = struct.unpack('f', struct.pack('I', 0xFFFFFFFF))[0]
                self.laser_cloud_map_storage = np.hstack((map_points, np.full((map_points.shape[0], 1), white_rgb)))

                # Unsubscribe after receiving the message once
                self.map_sub.unregister()
                self.is_running = True
                self.start_step_1_listener()

    def start_step_1_listener(self):
        self.step_1_sub = rospy.Subscriber('/step_1_point', PointCloud2, self.step_1_callback)
        self.start_time = time.time()
        self.timer_thread = threading.Thread(target=self.time_progress)
        self.timer_thread.start()
        self.timer = threading.Timer(55.0, self.stop_step_1_listener)
        self.timer.start()

    def stop_step_1_listener(self):
        with self.processing_lock:
            if self.step_1_sub:
                self.step_1_sub.unregister()
                rospy.loginfo("Stopped listening to /step_1_point.")
            
            self.is_running = False
            
            # Reset the storage for the next round
            self.laser_cloud_map_storage = None
            self.step_1_point_temp = None

            rospy.loginfo("Ready for new start command.")
            # The node does not shut down here; it waits for a new start command.

    def step_1_callback(self, msg):
        with self.processing_lock:
            step_1_points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)))

            if step_1_points.size == 0:
                rospy.logwarn("Received /step_1_point is empty. Skipping processing.")
                return

            # Downsample the point cloud
            step_1_points = self.voxel_grid_filter(step_1_points, leaf_size=0.1)
            self.step_1_point_temp = step_1_points

            # Process the point clouds
            self.process_point_clouds()

    def voxel_grid_filter(self, points, leaf_size=0.1):
        voxel_indices = np.floor(points[:, :3] / leaf_size).astype(np.int32)
        dtype = np.dtype((np.void, voxel_indices.dtype.itemsize * voxel_indices.shape[1]))
        voxel_indices_view = np.ascontiguousarray(voxel_indices).view(dtype)
        unique_voxels, unique_indices = np.unique(voxel_indices_view, return_index=True)
        downsampled_points = points[unique_indices]

        return downsampled_points

    def process_point_clouds(self):
        if self.laser_cloud_map_storage is None or self.step_1_point_temp is None:
            rospy.logwarn("Point clouds are missing during processing. Aborting.")
            return

        # Create Open3D PointCloud object from step_1_point_temp
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.step_1_point_temp[:, :3])

        # Check if the point cloud is empty
        if len(pcd.points) == 0:
            rospy.logwarn("step_1_point_temp is empty after downsampling. Skipping processing.")
            return

        # Create KDTree from step_1_point_temp
        step_1_tree = o3d.geometry.KDTreeFlann(pcd)

        tolerance = 0.1  # 10 cm tolerance for matching points

        initial_colored_count = np.sum(self.laser_cloud_map_storage[:, 3] != struct.unpack('f', struct.pack('I', 0xFFFFFFFF))[0])

        for i, map_point in enumerate(self.laser_cloud_map_storage):
            x, y, z, current_rgb = map_point

            if struct.unpack('I', struct.pack('f', current_rgb))[0] != 0xFFFFFFFF:
                continue

            # Prepare query point as a list of floats
            query_point = map_point[:3].tolist()

            # Perform KNN search
            try:
                [k, idx, dist] = step_1_tree.search_knn_vector_3d(query_point, 1)
            except Exception as e:
                continue

            if k > 0 and dist[0] < tolerance ** 2:  # Open3D returns squared distances
                rgb = self.step_1_point_temp[idx[0], 3]
                self.laser_cloud_map_storage[i, 3] = rgb

        # Publish the final colored point cloud
        self.publish_colored_point_cloud()

    def publish_colored_point_cloud(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/camera_init'

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]
        colored_pc_msg = point_cloud2.create_cloud(header, fields, self.laser_cloud_map_storage)

        self.step_2_pub.publish(colored_pc_msg)

    def time_progress(self):
        while self.is_running:
            elapsed_time = time.time() - self.start_time
            rospy.loginfo("Elapsed time: {:.2f} seconds / 55sec".format(elapsed_time))
            if elapsed_time >= 55.0:
                break
            time.sleep(5)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        mapper = PointCloudColorMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
