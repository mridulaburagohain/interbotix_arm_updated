#This file is modified to read from the /image_raw node and then publish to the /depth_raw map and process the monocular depth image

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
import cv2
import glob
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import torch
from depth_anything_v2.dpt import DepthAnythingV2

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
            '/depth_image',
            10
        )

        self.bridge = CvBridge()

        self.device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

        model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        encoder_type = 'vits'
        self.image_size =518
        self.depth_anything = DepthAnythingV2(**model_configs[encoder_type])
        self.depth_anything.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{encoder_type}.pth', map_location='cpu'))
        self.depth_anything = self.depth_anything.to(self.device).eval()

        self.get_logger().info('Depth Estimation Node is initialized')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # perform depth estimation
        # depth = self.inferr_depth(frame)
        with torch.no_grad():
            depth = self.depth_anything.infer_image(frame, self.image_size)

        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth = depth.astype(np.uint8)

        cmap = plt.get_cmap('Spectral_r')
        depth_colored = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)

        depth_msgs = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')

        self.depth_publisher.publish(depth_msgs)
        self.get_logger().info('Published depth image')

    def inferr_depth(self,raw_image):
         input_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)
         input_image = cv2.resize(input_image, (518,518))
         input_image = np.transpose(input_image, (2,0,1))
         input_image = torch.tensor(input_image, dtype=torch.float32)
         input_image = input_image.unsqueeze(0)

         with torch.no_grad():
              depth_map = self.depth_anything(input_image.to(self.device))

         return depth_map.squeeze().cpu().numpy()


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
