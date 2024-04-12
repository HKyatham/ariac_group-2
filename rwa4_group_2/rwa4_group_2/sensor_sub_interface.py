import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ariac_msgs.msg import BasicLogicalCameraImage
from .aruco_detector_interface import ArucoDetector
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
        cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite('right_bin_image.png', cv_image)  # Save the image from the callback
        self._colour_identification_right()
        # try:
        #     cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        #     cv2.imshow('Right Bin Image', cv_image)
        #     # cv2.imwrite('right_bin_image.png',cv_image)
        #     cv2.waitKey(1)
        #     self._colour_identification_right()
        #     # self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        # except Exception as e:
        #     self.get_logger().error('Failed to convert image: %r' % (e,))

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
                    if not (1.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.8')
                        self._part_list.append((1.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.235-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (1.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.7')
                        self._part_list.append((1.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.595-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (1.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.6')
                        self._part_list.append((1.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.415-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (1.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.5')
                        self._part_list.append((1.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.235-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (1.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.4')
                        self._part_list.append((1.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.595-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (1.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.3')
                        self._part_list.append((1.3,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.415-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (1.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.2')
                        self._part_list.append((1.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.235-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (1.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 1.1')
                        self._part_list.append((1.1,[X,Y,Z],[oX,oY,oZ,oW]))
                

                elif abs(0.155-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (2.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.9')
                        self._part_list.append((2.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.335-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (2.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.8')
                        self._part_list.append((2.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.515-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (2.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.7')
                        self._part_list.append((2.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.155-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (2.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.6')
                        self._part_list.append((2.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.335-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (2.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.5')
                        self._part_list.append((2.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.515-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (2.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.4')
                        self._part_list.append((2.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.155-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (2.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.3')
                        self._part_list.append((2.3,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.335-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (2.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.2')
                        self._part_list.append((2.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.515-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (2.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 2.1')
                        self._part_list.append((2.1,[X,Y,Z],[oX,oY,oZ,oW]))

                elif abs(0.155-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (3.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.9')
                        self._part_list.append((3.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.335-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (3.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.8')
                        self._part_list.append((3.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.515-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (3.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.7')
                        self._part_list.append((3.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.155-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (3.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.6')
                        self._part_list.append((3.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.335-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (3.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.5')
                        self._part_list.append((3.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.515-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (3.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.4')
                        self._part_list.append((3.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.155-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (3.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.3')
                        self._part_list.append((3.3,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.335-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (3.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.2')
                        self._part_list.append((3.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.515-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (3.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 3.1')
                        self._part_list.append((3.1,[X,Y,Z],[oX,oY,oZ,oW]))

                elif abs(-0.595-Y)<0.001 and abs(0.1848-Z)<0.001:
                    if not (4.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.9')
                        self._part_list.append((4.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.415-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (4.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.8')
                        self._part_list.append((4.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.235-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (4.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.7')
                        self._part_list.append((4.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.595-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (4.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.6')
                        self._part_list.append((4.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.415-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (4.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.5')
                        self._part_list.append((4.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.235-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (4.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.4')
                        self._part_list.append((4.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.595-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (4.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.3')
                        self._part_list.append((4.3,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.415-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (4.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.2')
                        self._part_list.append((4.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.235-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (4.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 4.1')
                        self._part_list.append((4.1,[X,Y,Z],[oX,oY,oZ,oW]))
                




                
                # self.get_logger().info(f'Part {i+1} Orientation: {oX} ,{oY} ,{oZ}, {oW}')


            
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))


    def _left_bins_rgb_camera_cb(self,msg):
        cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite('left_bin_image.png', cv_image)  # Save the image from the callback
        self._colour_identification_left()
        # try:
        #     cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        #     cv2.imshow('Left Bin Image', cv_image)
        #     # cv2.imwrite('left_bin_image.jpg',cv_image)
        #     cv2.waitKey(1)
        #     self._colour_identification_left()
        #     # self.get_logger().info('Received an image of shape: {}'.format(cv_image.shape))
        # except Exception as e:
        #     self.get_logger().error('Failed to convert image: %r' % (e,))

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
                    if not (5.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.7')
                        self._part_list.append((5.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.415-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (5.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.8')
                        self._part_list.append((5.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.235-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (5.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.9')
                        self._part_list.append((5.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.595-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (5.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.4')
                        self._part_list.append((5.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.415-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (5.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.5')
                        self._part_list.append((5.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.235-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (5.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.6')
                        self._part_list.append((5.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.595-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (5.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.1')
                        self._part_list.append((5.1,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.415-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (5.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.2')
                        self._part_list.append((5.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.235-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (5.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 5.3')
                        self._part_list.append((5.3,[X,Y,Z],[oX,oY,oZ,oW]))
                

                elif abs(-0.155-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (6.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.7')
                        self._part_list.append((6.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.335-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (6.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.8')
                        self._part_list.append((6.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.515-Y)<0.001 and abs(-0.566-Z)<0.001:
                    if not (6.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.9')
                        self._part_list.append((6.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.155-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (6.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.4')
                        self._part_list.append((6.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.335-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (6.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.5')
                        self._part_list.append((6.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.515-Y)<0.001 and abs(-0.386-Z)<0.001:
                    if not (6.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.6')
                        self._part_list.append((6.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.155-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (6.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.1')
                        self._part_list.append((6.1,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.335-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (6.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.2')
                        self._part_list.append((6.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.515-Y)<0.001 and abs(-0.206-Z)<0.001:
                    if not (6.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 6.3')
                        self._part_list.append((6.3,[X,Y,Z],[oX,oY,oZ,oW]))

                elif abs(-0.155-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (7.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.7')
                        self._part_list.append((7.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.335-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (7.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.8')
                        self._part_list.append((7.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.515-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (7.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.9')
                        self._part_list.append((7.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.155-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (7.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.4')
                        self._part_list.append((7.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.335-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (7.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.5')
                        self._part_list.append((7.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.515-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (7.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.6')
                        self._part_list.append((7.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.155-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (7.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.1')
                        self._part_list.append((7.1,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.335-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (7.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.2')
                        self._part_list.append((7.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(-0.515-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (7.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 7.3')
                        self._part_list.append((7.3,[X,Y,Z],[oX,oY,oZ,oW]))

                elif abs(0.595-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (8.7,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.7')
                        self._part_list.append((8.9,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.415-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (8.8,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.8')
                        self._part_list.append((8.8,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.235-Y)<0.001 and abs(0.184-Z)<0.001:
                    if not (8.9,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.9')
                        self._part_list.append((8.7,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.595-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (8.4,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.4')
                        self._part_list.append((8.6,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.415-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (8.5,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.5')
                        self._part_list.append((8.5,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.235-Y)<0.001 and abs(0.364-Z)<0.001:
                    if not (8.6,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.6')
                        self._part_list.append((8.4,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.595-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (8.1,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.1')
                        self._part_list.append((8.1,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.415-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (8.2,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.2')
                        self._part_list.append((8.2,[X,Y,Z],[oX,oY,oZ,oW]))
                elif abs(0.235-Y)<0.001 and abs(0.544-Z)<0.001:
                    if not (8.3,[X,Y,Z],[oX,oY,oZ,oW]) in self._part_list:
                        self.get_logger().info('Found Item in bin 8.3')
                        self._part_list.append((8.3,[X,Y,Z],[oX,oY,oZ,oW]))
                
                # self.get_logger().info(f'Part {i+1} Orientation: {oX} ,{oY} ,{oZ}, {oW}')
            
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))

    
    def _kts1_rgb_camera_cb(self,msg):
        try:
            detect_kts1 = ArucoDetector("KTS1")
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            kits_1_aruco_crops = [
            {"x1": 104, "y1": 199, "x2": 149, "y2": 244},
            {"x1": 297, "y1": 199, "x2": 342, "y2": 244},
            {"x1": 490, "y1": 199, "x2": 535, "y2": 244},
            ]
            for crop in kits_1_aruco_crops:
                self._kts1_tray_data = detect_kts1.crop_aruco(cv_image, crop["x1"], crop["y1"], crop["x2"], crop["y2"], detect_kts1._aruco_dict)
            self.get_logger().info(f'Kts1  - Tray data: {self._kts1_tray_data}')
        
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
            detect_kts2 = ArucoDetector("KTS2")
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            kits_2_aruco_crops = [
            {"x1": 104, "y1": 199, "x2": 149, "y2": 244},
            {"x1": 297, "y1": 199, "x2": 342, "y2": 244},
            {"x1": 490, "y1": 199, "x2": 535, "y2": 244},
            ]
            for crop in kits_2_aruco_crops:
                self._kts2_tray_data = detect_kts2.crop_aruco(cv_image, crop["x1"], crop["y1"], crop["x2"], crop["y2"], detect_kts2._aruco_dict)
            self.get_logger().info(f'Kts2  - Tray data: {self._kts2_tray_data}')
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
    def _colour_identification_right(self):
        bin_crops_right = {
                1.9: (0, 70, 115, 178),
                1.8: (0, 70, 178, 236),
                1.7: (0, 70, 236, 306),
                1.6: (70, 145, 115, 178),
                1.5: (70, 145, 178, 236),
                1.4: (70, 145, 236, 306),
                1.3: (145, 211, 115, 178),
                1.2: (145, 211, 178, 236),
                1.1: (145, 211, 236, 306),

                4.9: (263, 325, 354, 417),
                4.8: (263, 325, 178, 236),
                4.7: (263, 325, 236, 306),
                4.6: (325, 389, 354, 417),
                4.5: (325, 389, 178, 236),
                4.4: (325, 389, 236, 306),
                4.3: (389, 452, 354, 417),
                4.2: (389, 452, 178, 236),
                4.1: (389, 452, 236, 306),

                2.9: (0, 70, 354, 417),
                2.8: (0, 70, 417, 481),
                2.7: (0, 70, 481, 546),
                2.6: (70, 145, 354, 417),
                2.5: (70, 145, 417, 481),
                2.4: (70, 145, 481, 546),
                2.3: (145, 211, 354, 417),
                2.2: (145, 211, 417, 481),
                2.1: (145, 211, 481, 546),

                3.9: (263, 325, 354, 417),
                3.8: (263, 325, 417, 481),
                3.7: (263, 325, 481, 546),
                3.6: (325, 389, 354, 417),
                3.5: (325, 389, 417, 481),
                3.4: (325, 389, 481, 546),
                3.3: (389, 452, 354, 417),
                3.2: (389, 452, 417, 481),
                3.1: (389, 452, 481, 546)
        }
        color_ranges = {
                'red':    ([0, 50, 50], [10, 255, 255]),
                'green':  ([45, 50, 50], [75, 255, 255]),
                'orange': ([11, 50, 50], [20, 255, 255]),
                'blue':   ([100, 50, 50], [130, 255, 255]),
                'purple': ([125, 50, 50], [150, 255, 255])
        }
        

        img_right = cv2.imread('right_bin_image.png')
        if img_right is None:
            self.get_logger().error('Right bin image did not load.')
            return
        else:
            self.get_logger().info('Right bin image loaded successfully.')

        component_names = ['Battery', 'Sensor']
        components = {name: cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) for name in component_names if cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) is not None}

        results = {}
        sift = cv2.SIFT_create()

        for bin, coords in bin_crops_right.items():
            cropped_image = img_right[coords[0]:coords[1], coords[2]:coords[3]]
            hsv_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
            mask_total = np.zeros(cropped_image.shape[:2], dtype="uint8")
            max_pixels = 0
            dominant_color = 'Unknown'

            for color, (lower, upper) in color_ranges.items():
                lower_np = np.array(lower, dtype="uint8")
                upper_np = np.array(upper, dtype="uint8")
                mask = cv2.inRange(hsv_cropped_image, lower_np, upper_np)
                num_pixels = cv2.countNonZero(mask)
                
                if num_pixels > max_pixels:
                    max_pixels = num_pixels
                    dominant_color = color
            
            segmented_image = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)

            kp1, des1 = sift.detectAndCompute(cropped_image, None)
            match_scores = {}
            max_matches = 0
            best_component = 'None'
            
            for name, component_img in components.items():
                kp2, des2 = sift.detectAndCompute(component_img, None)
                matches = self.match_keypoints(des1, des2)
                match_scores[name] = len(matches)
                if len(matches) > max_matches:
                    max_matches = len(matches)
                    best_component = name

            results[bin] = {
                'Dominant Color': dominant_color,
                'Best Component': best_component,
                'Max Matches': max_matches
            }

        for bin, data in results.items():
            if data['Max Matches'] > 0:  
                self.get_logger().info(f'Bin {bin}: Dominant Color - {data["Dominant Color"]}, Best Match - {data["Best Component"]} ({data["Max Matches"]} matches)')

    
    
    def _colour_identification_left(self):
        bin_crops_left ={
            6.9: (22, 83, 141, 206),
            6.8: (22, 83, 206, 263),
            6.7: (22, 83, 263, 331),
            6.6: (83, 143, 141, 206),
            6.5: (83, 143, 206, 263),
            6.4: (83, 143, 263, 331),
            6.3: (143, 208, 141, 206),
            6.2: (143, 208, 206, 263),
            6.1: (143, 208, 263, 331),

            5.9: (22, 83, 380, 450),
            5.8: (22, 83, 450, 511),
            5.7: (22, 83, 511, 571),
            5.6: (83, 143, 380, 450),
            5.5: (83, 143, 450, 511),
            5.4: (83, 143, 511, 571),
            5.3: (143, 208, 380, 450),
            5.2: (143, 208, 450, 511),
            5.1: (143, 208, 511, 571),

            7.9: (264, 330, 141, 206),
            7.8: (264, 330, 206, 263),
            7.7: (264, 330, 263, 331),
            7.6: (330, 387, 141, 206),
            7.5: (330, 387, 206, 263),
            7.4: (330, 387, 263, 331),
            7.3: (387, 452, 141, 206),
            7.2: (387, 452, 206, 263),
            7.1: (387, 452, 263, 331),

            8.9: (264, 330, 380, 450),
            8.8: (264, 330, 450, 511),
            8.7: (264, 330, 511, 571),
            8.6: (330, 387, 380, 450),
            8.5: (330, 387, 450, 511),
            8.4: (330, 387, 511, 571),
            8.3: (387, 452, 380, 450),
            8.2: (387, 452, 450, 511),
            8.1: (387, 452, 511, 571)

        }
        color_ranges = {
                'red':    ([0, 50, 50], [10, 255, 255]),
                'green':  ([45, 50, 50], [75, 255, 255]),
                'orange': ([11, 50, 50], [20, 255, 255]),
                'blue':   ([100, 50, 50], [130, 255, 255]),
                'purple': ([125, 50, 50], [150, 255, 255])
        }
        

        img_left = cv2.imread('left_bin_image.png')
        if img_left is None:
            self.get_logger().error('Left bin image did not load.')
            return
        else:
            self.get_logger().info('Left bin image loaded successfully.')

        component_names = ['Battery', 'Sensor']
        components = {name: cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) for name in component_names if cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) is not None}

        results = {}
        sift = cv2.SIFT_create()

        for bin, coords in bin_crops_left.items():
            cropped_image = img_left[coords[0]:coords[1], coords[2]:coords[3]]
            hsv_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
            mask_total = np.zeros(cropped_image.shape[:2], dtype="uint8")
            max_pixels = 0
            dominant_color = 'Unknown'

            for color, (lower, upper) in color_ranges.items():
                lower_np = np.array(lower, dtype="uint8")
                upper_np = np.array(upper, dtype="uint8")
                mask = cv2.inRange(hsv_cropped_image, lower_np, upper_np)
                num_pixels = cv2.countNonZero(mask)
                
                if num_pixels > max_pixels:
                    max_pixels = num_pixels
                    dominant_color = color
            
            segmented_image = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)

            kp1, des1 = sift.detectAndCompute(cropped_image, None)
            match_scores = {}
            max_matches = 0
            best_component = 'None'
            
            for name, component_img in components.items():
                kp2, des2 = sift.detectAndCompute(component_img, None)
                matches = self.match_keypoints(des1, des2)
                match_scores[name] = len(matches)
                if len(matches) > max_matches:
                    max_matches = len(matches)
                    best_component = name

            results[bin] = {
                'Dominant Color': dominant_color,
                'Best Component': best_component,
                'Max Matches': max_matches
            }

        for bin, data in results.items():
            if data['Max Matches'] > 0:  
                self.get_logger().info(f'Bin {bin}: Dominant Color - {data["Dominant Color"]}, Best Match - {data["Best Component"]} ({data["Max Matches"]} matches)')

    def match_keypoints(self, descriptors1, descriptors2):
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors1, descriptors2, k=2)
        good = []
        for m, n in matches:
            if m.distance < 0.8 * n.distance:
                good.append(m)
        return good

    def get_logger(self):
        import logging
        logging.basicConfig(level=logging.INFO)
        return logging.getLogger('rwa4_group_2.sensor_sub_interface')
    
