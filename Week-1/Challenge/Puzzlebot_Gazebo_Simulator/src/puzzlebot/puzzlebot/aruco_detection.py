import cv2, time
import numpy as np
from threading import Thread
import cv2
from cv2 import aruco

if(cv2.__version__ == '4.7.0'):
    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    param_markers =  aruco.DetectorParameters()
else:
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    param_markers = aruco.DetectorParameters_create()

MARKER_SIZE = 5.5

aruco_points_3D = MARKER_SIZE*np.array([
    (-0.5, 0.5, 0.0),        # top left corner
    (0.5, 0.5, 0.0),        # top right corner
    (0.5, -0.5, 0.0),        # bottom right corner
    (-0.5, -0.5, 0.0),        # bottom left corner
])

# Alfredo's Camera
CAMERA_MATRIX = np.array([
    [1443.9, 0, 975.432],
    [0, 1434.18, 525.587],
    [0, 0, 1],
], dtype = "double")

# Alfredo's Camera
DIST_COEFF = np.array([1.10863557e-01, -1.23471957e+00, 2.27342121e-04, 6.40102480e-03,
    1.68010720e+00])

# Puzzlebot Camera
CAMERA_MATRIX = np.array([
    [688.7137085, 0, 315.48663694],
    [0, 900.03895134, 256.8402004],
    [0, 0, 1]
], dtype = "double")

# Puzzlebot Camera
DIST_COEFF = np.array([-0.01032327, 4.012904, -0.003140463, -0.00174154])

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
        for corners, marker_id in zip(marker_corners, marker_IDs):
            retval, vector_rotations, vector_translations = cv2.solveP3P(
                aruco_points_3D, corners, CAMERA_MATRIX, DIST_COEFF, flags=2)
            if(retval > 0):
                rot = vector_rotations[0]
                rot, _ = cv2.Rodrigues(rot)
                trans = vector_translations[0].flatten()


