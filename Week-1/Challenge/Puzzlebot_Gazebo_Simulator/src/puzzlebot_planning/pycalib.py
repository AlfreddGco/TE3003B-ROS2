#!/usr/bin/python3
import numpy as np
import cv2 as cv
import glob

pattern_size = (7, 5)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
objp[:,:2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print('Cannot open camera')
    exit()


while True:
    # img = cv.imread(fname)
    ret, img = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    found, corners = cv.findChessboardCorners(gray, pattern_size, None)
    # If found, add object points, image points (after refining them)
    if found:
        # corners = np.array([[corner for [corner] in corners]])
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, pattern_size, corners2, found)
        # cv.imshow('img', img)
    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        break

cv.destroyAllWindows()
objp = np.array([objp])
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print('K', mtx)
print('D', dist)


