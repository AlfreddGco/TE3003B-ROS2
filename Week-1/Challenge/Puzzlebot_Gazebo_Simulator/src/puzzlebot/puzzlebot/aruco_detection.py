import cv2, time
from cv2 import aruco
import numpy as np
from threading import Thread
from utils import implement_node, rotate_vec


DICT = aruco.DICT_4X4_250
if(cv2.__version__ == '4.7.0'):
    marker_dict = aruco.getPredefinedDictionary(DICT)
    param_markers =  aruco.DetectorParameters()
else:
    marker_dict = aruco.Dictionary_get(DICT)
    param_markers = aruco.DetectorParameters_create()

MARKER_SIZE = 4.0 / 100

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

class Aruco:
    def __init__(self, _id, position = (0, 0)):
        self.id = _id
        x, y = position
        self.position = np.array([x, y])
        self.rotation = np.array([0, 0, 0])
        self.collected = False 
        self.detections = []
    
    def add_detection(self, rot, trans, odom):
        if(len(self.detections) == 0):
            self.rotation = rot
            self.position = odom.position + trans
            self.detections.append(self.position)
            # self.position = np.mean(np.array(self.detections), axis=0)
        # Model: aruco doesnt move
        predicted_odom = self.position - trans
        self.estimated_odom = predicted_odom + 7/10*(odom.position - predicted_odom)
        odom.filtered_position = self.estimated_odom
        #print("m p e", odom.position[1], predicted_odom[1], self.estimated_odom[1])
        #print('Estimated:', estimated_odom, odom.position, predicted_odom)


class ArucoDetection:
    def __init__(self, nh, camera, arucos, odom):
        implement_node(self, nh)
        self.camera = camera
        self.arucos = arucos
        self.odom = odom
        # self.create_timer(1/60, self.detect)


    def detect(self):
        frame = self.camera.frame
        if (frame is None):
            return
        marker_corners, marker_ids, _ = aruco.detectMarkers(
                frame, marker_dict, parameters=param_markers)
        if(marker_ids is not None):
            marker_ids = marker_ids.flatten()
            # print('Markers:', len(marker_corners), marker_ids)
            self.get_aruco_position(marker_corners, marker_ids)


    def correct_odometry(self):
        pass


    def get_aruco_position(self, marker_corners, marker_ids):
        for corners, markerId in zip(marker_corners, marker_ids):
            retval, vectorRotations, vectorTranslations = cv2.solveP3P(
                aruco_points_3D, corners, 
                CAMERA_MATRIX, DIST_COEFF, 
                flags=2)
            if(retval > 0):
                markerRotation, _ = cv2.Rodrigues(vectorRotations[0])
                markerTranslation = vectorTranslations[0].flatten()
                self.update_aruco_position(
                    markerId, markerRotation, markerTranslation)
    
    
    def update_aruco_position(self, marker_id, rot, trans):
        for aruco in self.arucos:
            if(aruco.id == marker_id):
                # x, y
                relative_pos = np.array([trans[2], trans[0]])
                relative_pos = rotate_vec(
                    relative_pos, np.rad2deg(self.odom.theta))
                aruco.add_detection(
                    rot, relative_pos, self.odom)
                break

