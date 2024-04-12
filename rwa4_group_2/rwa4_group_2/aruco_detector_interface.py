import cv2
import numpy as np

class ArucoDetector():
    
    def __init__(self,kts):
        self._tray_data={}
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if kts == "KTS2":
            self._slot_counter = 4
        elif kts == "KTS1":
            self._slot_counter = 1
        
    def add_tray(self,tray_id,slot):
        if tray_id in self._tray_data:
            self._tray_data[tray_id].append(slot)
        else:
            self._tray_data[tray_id] =[slot]
            
    def get_slots(self,tray_id):
        return self._tray_data.get(tray_id, [])
    

    def aruco_detection(self,image, dictionary):
        # Initialising the Aruco parameters
        parameters = cv2.aruco.DetectorParameters()
        # Detecting the markers
        _, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
        if marker_ids is not None:
            for i in range(len(marker_ids)):
                # Decode the marker ID to determine the tray
                self.add_tray(int(marker_ids[i][0]),self._slot_counter)
                self._slot_counter += 1  
            return self._tray_data
        else:
            print("No markers detected")
            return None

    def crop_aruco(self,image, x1, y1, x2, y2, dictionary):
        img_frame_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cropped_region = img_frame_gray[y1:y2, x1:x2]
        white_padding_size = 20
        white_padding_width = x2 - x1 + 2 * white_padding_size
        white_padding_height = y2 - y1 + 2 * white_padding_size
        aruco_image_padded = np.ones((white_padding_height, white_padding_width), dtype=np.uint8) * 255
        x_offset = (white_padding_width - (x2 - x1)) // 2
        y_offset = (white_padding_height - (y2 - y1)) // 2
        aruco_image_padded[y_offset:y_offset + (y2 - y1), x_offset:x_offset + (x2 - x1)] = cropped_region
        return self.aruco_detection(aruco_image_padded, dictionary)
            
# if __name__ == "__main__":
    
#     kits_2_aruco_crops = [
#     {"x1": 104, "y1": 199, "x2": 149, "y2": 244},
#     {"x1": 297, "y1": 199, "x2": 342, "y2": 244},
#     {"x1": 490, "y1": 199, "x2": 535, "y2": 244},
#     ]
    
#     detect =ArucoDetector()
#     kit_2_img = "/home/gautam/ariac_ws/src/rwa4_group_2/opencv/kts2_img.png"
#     img_frame = cv2.imread(kit_2_img)
#     for crop in kits_2_aruco_crops:
#         detect.crop_aruco(img_frame, crop["x1"], crop["y1"], crop["x2"], crop["y2"], detect._aruco_dict)
    
    
    