import cv2
import numpy as np

class ArucoDetector():
    """A Class to handle aruco marker detection.

    This class handles aruco marker detection to identify the kit id using RGB Cameras placed over the kit tables.
    
    Attributes:
        _tray_data(dictionary): A dictionary to store tray IDs and their corresponding slots.
        _aruco_dict(dictionary): Aruco marker standard dictionary.
        _slot_counter(int): Counter to keep track of slot numbers
        
    Args:
        kts (str): Kit position ("KTS1" or "KTS2").
    """    
    
    def __init__(self,kts):
        self._tray_data={}
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if kts == "KTS2":
            self._slot_counter = 4
        elif kts == "KTS1":
            self._slot_counter = 1
        
    def add_tray(self,tray_id,slot):
        """ Method to add tray id and slot number to "_tray_data" dictionary.

        Args:
            tray_id (int): ID of the tray.
            slot (int): Slot number.
            
        """
        if tray_id in self._tray_data:
            self._tray_data[tray_id].append(slot)
        else:
            self._tray_data[tray_id] =[slot]
            
    def get_slots(self,tray_id):
        """Method to get slot position.

        Args:
            tray_id (int): ID of the tray.
        """
        return self._tray_data.get(tray_id, [])
    
    def aruco_detection(self,image, dictionary):
        """ Handle aruco marker detection.

        Args:
            image (numpy.ndarray): Input image containing cropped aruco markers.
            dictionary(_aruco_dict): Standard Aruco dictionary
        """
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
            self._slot_counter += 1  
            print("No markers detected")
            return self._tray_data

    def crop_aruco(self,image, x1, y1, x2, y2, dictionary):
        """Handle cropping of input image to get just  the aruco marker positions.

        Args:
            image (numpy.ndarray): Input image from rgb cameras.
            x1, y1, x2, y2 (int): Coordinates for cropping region to get just aruco markers.
            
        """
        img_frame_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cropped_region = img_frame_gray[y1:y2, x1:x2]
        # To give white background around aruco markers for detection.
        white_padding_size = 20
        white_padding_width = x2 - x1 + 2 * white_padding_size
        white_padding_height = y2 - y1 + 2 * white_padding_size
        aruco_image_padded = np.ones((white_padding_height, white_padding_width), dtype=np.uint8) * 255
        x_offset = (white_padding_width - (x2 - x1)) // 2
        y_offset = (white_padding_height - (y2 - y1)) // 2
        aruco_image_padded[y_offset:y_offset + (y2 - y1), x_offset:x_offset + (x2 - x1)] = cropped_region
        return self.aruco_detection(aruco_image_padded, dictionary)
            
    