#This file is modified to read from the /image_raw node and then publish to the /depth_raw map and process the monocular depth image

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import argparse
import cv2
import glob
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import torch
import depth_pro
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_estimation_node')

        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.depth_publisher = self. create_publisher(
            Image,
            '/depth_raw',
            10
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            '/point_cloud',
            10
        )

        self.bridge = CvBridge()

        self.device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        self.model, self.transform = depth_pro.create_model_and_transforms()
        self.model.eval()      

        self.get_logger().info('Depth Estimation Node is initialized')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # perform depth estimation
        # depth = self.inferr_depth(frame)
        # with torch.no_grad():
        image = self.transform(frame)   
        actual_focal_length = 1752.92  
        focal_length_tensor = torch.tensor(actual_focal_length).float()    
        prediction = self.model.infer(image, f_px=focal_length_tensor)
        depth = prediction["depth"]  # Depth in [m].
        focallength_px = prediction["focallength_px"]  

        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth = depth.to(torch.uint8)

        cmap = plt.get_cmap('Spectral_r')
        depth_colored = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)

        depth_msgs = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')

        self.depth_publisher.publish(depth_msgs)
        self.get_logger().info('Published depth image')
        
        # Convert depth map to point cloud and publish it
        point_cloud = self.depth_to_point_cloud(depth, frame.shape[1], frame.shape[0], actual_focal_length)
        pc_msg = self.create_point_cloud_msg(point_cloud)
        self.pointcloud_publisher.publish(pc_msg)
        self.get_logger().info('Published point cloud')
        
        
    def depth_to_point_cloud(self, depth_map, width, height, focal_length):
        """Convert depth map to 3D point cloud."""
        points = []
        
        K = np.array([[3.10196184e+04, 0, 2.07475300e+03],  # f_x, 0, c_x
                      [0, 3.08370233e+04, 1.24599106e+03],  # 0, f_y, c_y
                      [0, 0, 1]])     # 0, 0, 1
        f_x, _, c_x = K[0]
        _, f_y, c_y = K[1]
        for v in range(height):
            for u in range(width):
                Z = depth_map[v, u] / 255.0  # Scale to [0, 1] range
                if Z == 0:
                    continue  # Skip invalid depth values (0)

                # Compute 3D coordinates
                X = (u - c_x) * Z / f_x
                Y = (v - c_y) * Z / f_y

                points.append((X, Y, Z))
        return points

    def create_point_cloud_msg(self, points):
        """Create PointCloud2 message from list of 3D points."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'  # You can set this to the actual frame ID of your camera

        # Create the PointCloud2 message using the points (X, Y, Z)
        pc_data = pc2.create_cloud_xyz32(header, points)
        return pc_data
        
    """   
    def inferr_depth(self,raw_image):
         input_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)
         input_image = cv2.resize(input_image, (518,518))
         input_image = np.transpose(input_image, (2,0,1))
         input_image = torch.tensor(input_image, dtype=torch.float32)
         input_image = input_image.unsqueeze(0)

         with torch.no_grad():
              depth_map = self.depth_anything(input_image.to(self.device))

         return depth_map.squeeze().cpu().numpy()
     """

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
