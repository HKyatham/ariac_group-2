import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ariac_msgs.msg import BasicLogicalCameraImage
from .aruco_detector_interface import ArucoDetector
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self._left_part_list=[]
        self._right_part_list=[]
        self._part_dic = {}
        colors = ['green', 'orange', 'blue', 'red', 'yellow', 'purple']
        components = ['sensor', 'battery', 'regulator', 'pump']
        for color in colors:
            for component in components:
                attr_name = f"_{color}_{component}"
                setattr(self, attr_name, [])
        self.right_bins_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/right_bins_camera/image",self._right_bins_camera_cb, qos_profile_sensor_data)
        self.right_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/right_bins_rgb_camera/rgb_image",self._right_bins_rgb_camera_cb,10)
        self.left_bins_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/left_bins_camera/image",self._left_bins_camera_cb, qos_profile_sensor_data)
        self.left_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/left_bins_rgb_camera/rgb_image",self._left_bins_rgb_camera_cb,10)
        # self.kts1_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts1_rgb_camera/rgb_image",self._kts1_rgb_camera_cb,10)
        # self.kts1_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/kts1_camera/image",self._kts1_camera_cb,qos_profile_sensor_data)
        # self.kts2_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts2_rgb_camera/rgb_image",self._kts2_rgb_camera_cb,10)
        # self.kts2_camera_sub = self.create_subscription(BasicLogicalCameraImage,"/ariac/sensors/kts2_camera/image",self._kts2_camera_cb,qos_profile_sensor_data)
        self._bridge = CvBridge()
    def _right_bins_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            self._colour_identification_right(cv_image)
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))
    
    def _right_bins_camera_cb(self,msg):
        try:
            if len(msg._part_poses) == 0:
                self.get_logger().info('NO Part DETECTED')
            for i, part_pose in enumerate(msg._part_poses):
                # self.get_logger().info(f'DETECTED part {i+1} ')
                X=part_pose.position.x
                Y=part_pose.position.y
                Z=part_pose.position.z
                oX=part_pose.orientation.x
                oY=part_pose.orientation.y
                oZ=part_pose.orientation.z
                oW=part_pose.orientation.w

                _Y_axis=[-0.595,-0.415,-0.235,0.155,0.335,0.515]
                _Z_axis=[-0.566,-0.386,-0.206,0.184,0.364,0.544]
                _slot_val={1:1.9,2:1.8,3:1.7,4:2.9,5:2.8,6:2.7,
                           7:1.6,8:1.5,9:1.4,10:2.6,11:2.5,12:2.4,
                           13:1.3,14:1.2,15:1.1,16:2.3,17:2.2,18:2.1,
                           19:4.9,20:4.8,21:4.7,22:3.9,23:3.8,24:3.7,
                           25:4.6,26:4.5,27:4.4,28:3.6,29:3.5,30:3.4,
                           31:4.3,32:4.2,33:4.1,34:3.3,35:3.2,36:3.1}

                counter=0                
                for i, z_val in enumerate(_Z_axis):
                    for j , y_val in enumerate(_Y_axis):
                        counter+=1
                        if abs(y_val-Y)<0.001 and abs(z_val-Z)<0.001:
                            slot=_slot_val.get(counter)
                            if slot not in self._right_part_list:
                                self.get_logger().info(f'Found Item in bin {i},{j},{counter},{slot}')
                                self._right_part_list.append(slot)
                                self._part_dic.update({(slot):(X,Y,Z, oX,oY,oZ,oW)})

            
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))


    def _left_bins_rgb_camera_cb(self,msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            self._colour_identification_left(cv_image)
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))

    def _left_bins_camera_cb(self,msg):
        try:
            if len(msg._part_poses) == 0:
                self.get_logger().info('NO Part DETECTED')
            for i, part_pose in enumerate(msg._part_poses):
                # self.get_logger().info(f'DETECTED part {i+1} in LEFT BIN')
                X=part_pose.position.x
                Y=part_pose.position.y
                Z=part_pose.position.z
                oX=part_pose.orientation.x
                oY=part_pose.orientation.y
                oZ=part_pose.orientation.z
                oW=part_pose.orientation.w
                _Y_axis=[-0.515,-0.335,-0.155,0.235,0.415,0.595]
                _Z_axis=[-0.566,-0.386,-0.206,0.184,0.364,0.544]
                _slot_val={1:6.9,2:6.8,3:6.7,4:5.9,5:5.8,6:5.7,
                        7:6.6,8:6.5,9:6.4,10:5.6,11:5.5,12:5.4,
                        13:6.3,14:6.2,15:6.1,16:5.3,17:5.2,18:5.1,
                        19:7.9,20:7.8,21:7.7,22:8.9,23:8.8,24:8.7,
                        25:7.6,26:7.5,27:7.4,28:8.6,29:8.5,30:8.4,
                        31:7.3,32:7.2,33:7.1,34:8.3,35:8.2,36:8.1}

                counter=0                
                for i, z_val in enumerate(_Z_axis):
                    for j , y_val in enumerate(_Y_axis):
                        counter+=1
                        if abs(y_val-Y)<0.001 and abs(z_val-Z)<0.001:
                            slot=_slot_val.get(counter)
                            if slot not in self._left_part_list:
                                self.get_logger().info(f'Found Item in bin {i},{j},{counter},{slot}')
                                self._left_part_list.append(slot)
                                self._part_dic.update({(slot):(X,Y,Z, oX,oY,oZ,oW)})
                
            
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
    
    def _colour_identification_right(self,image):
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
                2.6: (65, 145, 354, 417),
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
        

        img_right = image
        # if img_right is None:
        #     self.get_logger().error('Right bin image did not load.')
        #     return
        # else:
        #     self.get_logger().info('Right bin image loaded successfully.')

        component_names = ['Battery', 'Sensor']
        components = {name: cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) for name in component_names if cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) is not None}

        results = {}
        sift = cv2.SIFT_create()
        for detected_parts in self._right_part_list:
            coords = bin_crops_right.get(detected_parts)
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
            cropped_image_gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            kp1, des1 = sift.detectAndCompute(cropped_image_gray, None)
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

            results[detected_parts] = {
                'Bin': detected_parts,
                'Color': dominant_color,
                'Component': best_component,
                'Max Matches': max_matches
            }
            
        for detected_parts, data in results.items():
            ort = self._part_dic.get(detected_parts)
            if ort is None:
                self.get_logger().error(f"Orientation data not found for {detected_parts}")
                continue

            ort_var = {
                'X': ort[0], 'Y': ort[1], 'Z': ort[2],
                'roll': ort[3], 'pitch': ort[4], 'yaw': ort[5], 'w': ort[6]
            }

            list_color = ['green', 'orange', 'blue', 'red', 'purple']
            list_component = ['Sensor', 'Battery', 'Regulator', 'Pump']

            dominant_color = data["Color"].lower()
            best_component = data["Component"]

            matched = False  # Flag to indicate a successful match

            for color in list_color:
                for component in list_component:
                    attr_name = f"_{color.lower()}_{component.lower()}"
                    if dominant_color == color.lower() and best_component == component:
                        if hasattr(self, attr_name):
                            target_list = getattr(self, attr_name)
                            if ort_var not in target_list:  
                                target_list.append(ort_var)
                                self.get_logger().info(f"Updated {attr_name}: {ort_var}")  
                                matched = True
                        else:
                            self.get_logger().error(f"Attribute {attr_name} not found on {self}")

            if matched and data['Max Matches'] > 0:  
                self.get_logger().info(
                    f'Bin {detected_parts}: Color - {dominant_color.title()}, Best Match - {best_component} '
                    f'({data["Max Matches"]} matches), Orientation X - {ort_var["X"]}, Y - {ort_var["Y"]}, Z - {ort_var["Z"]}, '
                    f'roll - {ort_var["roll"]}, pitch - {ort_var["pitch"]}, yaw - {ort_var["yaw"]}, w - {ort_var["w"]}'
                )

    
    def _colour_identification_left(self, image):
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
        

        img_left = image
        # if img_left is None:
        #     self.get_logger().error('Left bin image did not load.')
        #     return
        # else:
        #     self.get_logger().info('Left bin image loaded successfully.')

        component_names = ['Battery', 'Sensor']
        components = {name: cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) for name in component_names if cv2.imread(f'{name}.png', cv2.IMREAD_GRAYSCALE) is not None}

        results = {}
        sift = cv2.SIFT_create()

        for detected_parts in self._left_part_list:
            coords = bin_crops_left.get(detected_parts)
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
            cropped_image_gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            kp1, des1 = sift.detectAndCompute(cropped_image_gray, None)
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

            results[detected_parts] = {
                'Bin': detected_parts,
                'Color': dominant_color,
                'Component': best_component,
                'Max Matches': max_matches
            }
        
        for detected_parts, data in results.items():
            ort = self._part_dic.get(detected_parts)
            if ort is None:
                self.get_logger().error(f"Orientation data not found for {detected_parts}")
                continue

            ort_var = {
                'X': ort[0], 'Y': ort[1], 'Z': ort[2],
                'roll': ort[3], 'pitch': ort[4], 'yaw': ort[5], 'w': ort[6]
            }

            list_color = ['green', 'orange', 'blue', 'yellow','red', 'purple']
            list_component = ['Sensor', 'Battery', 'Regulator', 'Pump']

            dominant_color = data["Color"].lower()
            best_component = data["Component"]

            matched = False  

            for color in list_color:
                for component in list_component:
                    attr_name = f"_{color.lower()}_{component.lower()}"
                    if dominant_color == color.lower() and best_component == component:
                        if hasattr(self, attr_name):
                            target_list = getattr(self, attr_name)
                            if ort_var not in target_list:  
                                target_list.append(ort_var)
                                self.get_logger().info(f"Updated {attr_name}: {ort_var}")  
                                matched = True
                        else:
                            self.get_logger().error(f"Attribute {attr_name} not found on {self}")

            if matched and data['Max Matches'] > 0: 
                self.get_logger().info(
                    f'Bin {detected_parts}: Color - {dominant_color.title()}, Best Match - {best_component} '
                    f'({data["Max Matches"]} matches), Orientation X - {ort_var["X"]}, Y - {ort_var["Y"]}, Z - {ort_var["Z"]}, '
                    f'roll - {ort_var["roll"]}, pitch - {ort_var["pitch"]}, yaw - {ort_var["yaw"]}, w - {ort_var["w"]}'
                )


    def match_keypoints(self, descriptors1, descriptors2):
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors1, descriptors2, k=2)
        good = []
        for m, n in matches:
            if m.distance < 0.98 * n.distance:
                    good.append(m)
            return good

    def get_logger(self):
        import logging
        logging.basicConfig(level=logging.INFO)
        return logging.getLogger('rwa4_group_2.sensor_sub_interface')
    
