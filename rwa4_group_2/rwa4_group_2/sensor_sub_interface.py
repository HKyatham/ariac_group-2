import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ariac_msgs.msg import BasicLogicalCameraImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.right_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/right_bins_rgb_camera/rgb_image",self._right_bins_rgb_camera_cb,10)
        self.right_bins_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/right_bins_camera/image",self._right_bins_camera_cb, qos_profile_sensor_data)
        self.left_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/left_bins_rgb_camera/rgb_image",self._left_bins_rgb_camera_cb,10)
        # self.left_bins_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/left_bins_camera/image",self._left_bins_camera_cb,10)
        self.kts1_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts1_rgb_camera/rgb_image",self._kts1_rgb_camera_cb,10)
        self.kts2_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts2_rgb_camera/rgb_image",self._kts2_rgb_camera_cb,10)
        self._bridge = CvBridge()


    def _right_bins_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Right Bin Image', cv_image)
            cv2.waitKey(1)
            # self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))

    def _right_bins_camera_cb(self,msg):
        try:
            if len(msg._part_poses) == 0:
                self.get_logger().info('NO Part DETECTED')
            for i, part_pose in enumerate(msg._part_poses):
                self.get_logger().info(f'DETECTED part {i+1} ')
                X=part_pose.position.x
                Y=part_pose.position.y
                Z=part_pose.position.z
                self.get_logger().info(f'Part {i+1} Position: {X} ,{Y} ,{Z}')


            
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))


    def _left_bins_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Left Bin Image', cv_image)
            cv2.waitKey(1)
            # self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))
    
    def _kts1_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Left Tray Image', cv_image)
            cv2.waitKey(1)
            # self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))
    
    def _kts2_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Right Tray Image', cv_image)
            cv2.waitKey(1)
            # self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))
    # def bin_parts(self, msg):
    #     if type(self.left_bins_rgb_camera_image) == type(np.ndarray([])) and type(self.right_bins_rgb_camera_image) == type(np.ndarray([])):














