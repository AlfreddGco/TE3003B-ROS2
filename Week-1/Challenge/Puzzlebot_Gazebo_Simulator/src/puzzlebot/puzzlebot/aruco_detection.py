import cv2, time
import numpy as np
from threading import Thread
from cv2 import aruco

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# ????
aruco_points_3D = np.array([
    (0.0, 0.0, 0.0),        # top left corner
    (0.0, 0.0, 0.0),        # top right corner
    (0.0, 0.0, 0.0),        # bottom left corner
    (0.0, 0.0, 0.0),        # bottom right corner
])

CAMERA_MATRIX = np.array([
    [385.172671, 0, 319.493898],
    [0, 516.679629, 231.445904],
    [0, 0, 1]
], dtype = "double")

DIST_COEFF = np.array([-0.311362, 0.082442, 0.000836, 0.001488])

markerSizeInCM = 4

class ArucoDetection:
    def __init__(self, camera):
        self.camera = camera
        self.arucos = []


    def detect(self):
        frame = self.camera.frame
        if(frame is None):
            return
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            frame, marker_dict, parameters=param_markers
        )
        print(marker_corners, marker_IDs)
        self.get_aruco_position(marker_corners, marker_IDs)


    def get_aruco_position(self, marker_corners, marker_IDs):
        for corners in marker_corners:
            success, vector_rotations, vector_translations = cv2.solvePnP(
                aruco_points_3D, corners, CAMERA_MATRIX, DIST_COEFF, flags=0)