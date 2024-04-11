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
        self._part_list=[]
        self.right_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/right_bins_rgb_camera/rgb_image",self._right_bins_rgb_camera_cb,10)
        self.right_bins_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/right_bins_camera/image",self._right_bins_camera_cb, qos_profile_sensor_data)
        self.left_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/left_bins_rgb_camera/rgb_image",self._left_bins_rgb_camera_cb,10)
        self.left_bins_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/left_bins_camera/image",self._left_bins_camera_cb, qos_profile_sensor_data)
        self.kts1_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts1_rgb_camera/rgb_image",self._kts1_rgb_camera_cb,10)
        self.kts1_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/kts1_camera/image",self._kts1_camera_cb,qos_profile_sensor_data)
        self.kts2_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts2_rgb_camera/rgb_image",self._kts2_rgb_camera_cb,10)
        self.kts2_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/kts2_camera/image",self._kts2_camera_cb,qos_profile_sensor_data)

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
                oX=part_pose.orientation.x
                oY=part_pose.orientation.y
                oZ=part_pose.orientation.z
                oW=part_pose.orientation.w
                # self.get_logger().info(f'Part {i+1} Position: {X} ,{Y} ,{Z}')
                if abs(-0.595-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (1.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.9')
                        self._part_list.append((1.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.415-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.8')
                elif abs(-0.235-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.7')
                elif abs(-0.595-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.6')
                elif abs(-0.415-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.5')
                elif abs(-0.235-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.4')
                elif abs(-0.595-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.3')
                elif abs(-0.415-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.2')
                elif abs(-0.235-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 1.1')
                

                elif abs(0.155-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.9')
                elif abs(0.335-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.8')
                elif abs(0.515-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.7')
                elif abs(0.155-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.6')
                elif abs(0.335-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.5')
                elif abs(0.515-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.4')
                elif abs(0.155-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.3')
                elif abs(0.335-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.2')
                elif abs(0.515-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 2.1')

                elif abs(0.155-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.9')
                elif abs(0.335-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.8')
                elif abs(0.515-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.7')
                elif abs(0.155-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.6')
                elif abs(0.335-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.5')
                elif abs(0.515-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.4')
                elif abs(0.155-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.3')
                elif abs(0.335-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.2')
                elif abs(0.515-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 3.1')

                elif abs(-0.595-Y)<0.001 and abs(0.1848-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.9')
                elif abs(-0.415-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.8')
                elif abs(-0.235-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.7')
                elif abs(-0.595-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.6')
                elif abs(-0.415-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.5')
                elif abs(-0.235-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.4')
                elif abs(-0.595-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.3')
                elif abs(-0.415-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.2')
                elif abs(-0.235-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 4.1')
                




                
                # self.get_logger().info(f'Part {i+1} Orientation: {oX} ,{oY} ,{oZ}, {oW}')


            
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

    def _left_bins_camera_cb(self,msg):
        try:
            if len(msg._part_poses) == 0:
                self.get_logger().info('NO Part DETECTED')
            for i, part_pose in enumerate(msg._part_poses):
                self.get_logger().info(f'DETECTED part {i+1} in LEFT BIN')
                X=part_pose.position.x
                Y=part_pose.position.y
                Z=part_pose.position.z
                oX=part_pose.orientation.x
                oY=part_pose.orientation.y
                oZ=part_pose.orientation.z
                oW=part_pose.orientation.w
                # self.get_logger().info(f'Part {i+1} Position: {X} ,{Y} ,{Z}')

                if abs(0.595-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.7')
                elif abs(0.415-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.8')
                elif abs(0.235-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.9')
                elif abs(0.595-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.4')
                elif abs(0.415-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.5')
                elif abs(0.235-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.6')
                elif abs(0.595-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.1')
                elif abs(0.415-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.2')
                elif abs(0.235-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 5.3')
                

                elif abs(-0.155-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.7')
                elif abs(-0.335-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.8')
                elif abs(-0.515-Y)<0.001 and abs(-0.566-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.9')
                elif abs(-0.155-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.4')
                elif abs(-0.335-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.5')
                elif abs(-0.515-Y)<0.001 and abs(-0.386-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.6')
                elif abs(-0.155-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.1')
                elif abs(-0.335-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.2')
                elif abs(-0.515-Y)<0.001 and abs(-0.206-Z)<0.001:
                    self.get_logger().info('Found Item in bin 6.3')

                elif abs(-0.155-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.7')
                elif abs(-0.335-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.8')
                elif abs(-0.515-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.9')
                elif abs(-0.155-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.4')
                elif abs(-0.335-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.5')
                elif abs(-0.515-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.6')
                elif abs(-0.155-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.1')
                elif abs(-0.335-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.2')
                elif abs(-0.515-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 7.3')

                elif abs(0.595-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.7')
                elif abs(0.415-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.8')
                elif abs(0.235-Y)<0.001 and abs(0.184-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.9')
                elif abs(0.595-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.4')
                elif abs(0.415-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.5')
                elif abs(0.235-Y)<0.001 and abs(0.364-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.6')
                elif abs(0.595-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.1')
                elif abs(0.415-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.2')
                elif abs(0.235-Y)<0.001 and abs(0.544-Z)<0.001:
                    self.get_logger().info('Found Item in bin 8.3')
                
                # self.get_logger().info(f'Part {i+1} Orientation: {oX} ,{oY} ,{oZ}, {oW}')
            
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
    
    def _kts1_camera_cb(self,msg):
        try:
            if len(msg._tray_poses) == 0:
                self.get_logger().info('NO Tray DETECTED')
            for i, tray_pose in enumerate(msg._tray_poses):
                self.get_logger().info(f'DETECTED part {i+1} in LEFT Tray')
                X=tray_pose.position.x
                Y=tray_pose.position.y
                Z=tray_pose.position.z
                oX=tray_pose.orientation.x
                oY=tray_pose.orientation.y
                oZ=tray_pose.orientation.z
                oW=tray_pose.orientation.w
                self.get_logger().info(f'Part {i+1} Position: {X} ,{Y} ,{Z}')
                self.get_logger().info(f'Part {i+1} Orientation: {oX} ,{oY} ,{oZ}, {oW}')
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
    
    def _kts2_camera_cb(self,msg):
        try:
            if len(msg._tray_poses) == 0:
                self.get_logger().info('NO Tray DETECTED')
            for i, tray_pose in enumerate(msg._tray_poses):
                self.get_logger().info(f'DETECTED part {i+1} in LEFT Tray')
                X=tray_pose.position.x
                Y=tray_pose.position.y
                Z=tray_pose.position.z
                oX=tray_pose.orientation.x
                oY=tray_pose.orientation.y
                oZ=tray_pose.orientation.z
                oW=tray_pose.orientation.w
                self.get_logger().info(f'Part {i+1} Position: {X} ,{Y} ,{Z}')
                self.get_logger().info(f'Part {i+1} Orientation: {oX} ,{oY} ,{oZ}, {oW}')
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))
    # def bin_parts(self, msg):
    #     if type(self.left_bins_rgb_camera_image) == type(np.ndarray([])) and type(self.right_bins_rgb_camera_image) == type(np.ndarray([])):














