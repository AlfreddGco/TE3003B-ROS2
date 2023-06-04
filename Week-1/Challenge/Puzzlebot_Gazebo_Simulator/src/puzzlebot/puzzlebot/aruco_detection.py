import cv2, time
import numpy as np
from threading import Thread
from cv2 import aruco

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

MARKER_SIZE = 10

# ????
aruco_points_3D = MARKER_SIZE*np.array([
    (-0.5, 0.5, 0.0),        # top left corner
    (0.5, 0.5, 0.0),        # top right corner
    (0.5, -0.5, 0.0),        # bottom right corner
    (-0.5, -0.5, 0.0),        # bottom left corner
])

CAMERA_MATRIX = np.array([
    [385.172671, 0, 319.493898],
    [0, 516.679629, 231.445904],
    [0, 0, 1]
], dtype = "double")

DIST_COEFF = np.array([-0.311362, 0.082442, 0.000836, 0.001488])

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
        print('Markers:', len(marker_corners))
        if(len(marker_corners) > 0):
            print(marker_corners[0])
        self.get_aruco_position(marker_corners, marker_IDs)


    def get_aruco_position(self, marker_corners, marker_IDs):
        for corners in marker_corners:
            retval, vector_rotations, vector_translations = cv2.solveP3P(
                aruco_points_3D, corners, CAMERA_MATRIX, DIST_COEFF, flags=2)
            if(retval > 0):
                rot = vector_rotations[0]
                rot, _ = cv2.Rodrigues(rot)
                trans = vector_translations[0].flatten()
                #trans = np.matmul(-rot, trans)
                screen_point = np.array([corners[0][0][0], corners[0][0][1], 0])
                world = np.matmul(np.transpose(rot), screen_point) - trans
                print(world)
