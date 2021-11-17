#!/usr/bin/env python

from PyQt5 import QtCore, QtGui, QtWidgets
from python_qt_binding import loadUi

import cv2
import sys
import numpy as np

def Check_Homography(img, driving_feed):
    	gray_frame = cv2.cvtColor(driving_feed, cv2.COLOR_BGR2GRAY)

    	sift = cv2.xfeatures2d.SIFT_create()

    	kp_image, desc_image = sift.detectAndCompute(img, None)
    	# img = cv2.drawKeyPoints(img, kp_image, img)

    	kp_grayframe, desc_grayframe = sift.detectAndCompute(gray_frame, None)
    	# gray_frame = cv2.drawKeyPoints(gray_frame, kp_grayframe, gray_frame)

    	index_params = dict(algorithm=0, trees=5)
    	search_params = dict()
    	flann = cv2.FlannBasedMatcher(index_params, search_params)

    	matches = flann.knnMatch(desc_image, desc_grayframe, k=2)
    	good_points = []
    	for m, n in matches:
    		if m.distance < 0.6 * n.distance:
    			good_points.append(m)

    	if len(good_points) > 8:
    		query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
    		train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
    		matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
    		matches_mask = mask.ravel().tolist()

    		h, w = img.shape
    		pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
    		dst = cv2.perspectiveTransform(pts, matrix)

    		homography = cv2.polylines(driving_feed, [np.int32(dst)], True, (255, 0, 0), 3)

    		return homography

    	else:
    		searching = cv2.drawMatches(img, kp_image, gray_frame, kp_grayframe, good_points, gray_frame)
    		return searching