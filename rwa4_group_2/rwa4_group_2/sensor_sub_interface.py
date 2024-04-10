import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from ariac_msgs.msg import BasicLogicalCameraImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.right_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/right_bins_rgb_camera/rgb_image",self.right_bins_rgb_camera_cb,10)
        self.left_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/left_bins_rgb_camera/rgb_image",self.left_bins_rgb_camera_cb,10)
        self._bridge = CvBridge()


    def right_bins_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Right Bin Image', cv_image)
            cv2.waitKey(1)
            self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))
    
    def left_bins_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Left Bin Image', cv_image)
            cv2.waitKey(1)
            self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))
    
    # def bin_parts(self, msg):
    #     if type(self.left_bins_rgb_camera_image) == type(np.ndarray([])) and type(self.right_bins_rgb_camera_image) == type(np.ndarray([])):














