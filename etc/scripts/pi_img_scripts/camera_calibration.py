#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  gaussian_mixture_fit.py: fit gaussian mixture distribution to 1-D data
#
#  Copyright © OPENCV-ORG Documentation
#  Copyright © 2024 Abhirup Das<abhirup.das@rwth-aachen.de>
#
##########################################################################
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.
import glob

import cv2 as cv
import numpy as np

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) * gridsize like 2.5cm
objp = np.zeros((6 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * 2.5

print(objp)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob("put_calibration_images_here/*.jpeg")

for fname in images:
    img = cv.imread(fname)
    img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (8, 6), None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    cv.drawChessboardCorners(img, (8, 6), corners2, ret)
    cv.imshow("img", img)
    cv.waitKey(50)

cv.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
img = cv.imread("put_calibration_images_here/cali10.jpeg")
img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
print("------------------")
print("roi:")
print(roi)
print("------------------")
print("old intrinsic camera parameters")
print("old ppx: ", mtx[0, 2])
print("old ppy: ", mtx[1, 2])
print("old f_x: ", mtx[0, 0])
print("old f_y: ", mtx[1, 1])
print("------------------")
print("new intrinsic camera parameters")
print("new ppx: ", newcameramtx[0, 2])
print("new ppy: ", newcameramtx[1, 2])
print("new f_x: ", newcameramtx[0, 0])
print("new f_y: ", newcameramtx[1, 1])
print("------------------")
print("distortion: ", dist)

old_ppx = mtx[0, 2]
old_ppy = mtx[1, 2]
old_f_x = mtx[0, 0]
old_f_y = mtx[1, 1]

new_ppx = newcameramtx[0, 2]
new_ppy = newcameramtx[1, 2]
new_f_x = newcameramtx[0, 0]
new_f_y = newcameramtx[1, 1]

k1 = dist[0, 0]
k2 = dist[0, 1]
k3 = dist[0, 2]
k4 = dist[0, 3]
k5 = dist[0, 4]

old_camera_matrix = cv.UMat(np.array([[old_f_x, 0.0, old_ppx], [0.0, old_f_y, old_ppy], [0.0, 0.0, 1.0]]))
new_camera_matrix = cv.UMat(np.array([[new_f_x, 0.0, new_ppx], [0.0, new_f_y, new_ppy], [0.0, 0.0, 1.0]]))
distortion = cv.UMat(np.array([[k1, k2, k3, k4, k5]]))

# undistort
dst = cv.undistort(img, old_camera_matrix, distortion, None, new_camera_matrix)

# cv.imwrite('calibresult_new.png', dst)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    mean_error += error

print("total error: {}".format(mean_error / len(objpoints)))
