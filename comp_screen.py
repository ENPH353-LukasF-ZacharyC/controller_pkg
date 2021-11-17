#!/usr/bin/env python

from PyQt5 import QtCore, QtGui, QtWidgets
from python_qt_binding import loadUi

import cv2
import sys
import rospy
import matplotlib.pyplot as plt

from PIL import Image, ImageFont, ImageDraw
from sensor_msgs.msg import Image as I
from cv_bridge import CvBridge, CvBridgeError

# img_PATH="/home/fizzer/ros_ws/src/controller_pkg/3232.png"
# img_name="3232.png"
# empty_license_img=Image.open(img_PATH)

class comp_App(QtWidgets.QMainWindow):

    def __init__(self):
        super(comp_App, self).__init__()
        loadUi("./competition_screen.ui", self)

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw", I, self.pub_image)

        # self._timer = QtCore.QTimer(self)
        # self._timer.timeout.connect(self.SLOT_query_camera)
        # self._timer.setInterval(1000 / 10)


    def pub_image(self, img):
    	img = self.bridge.imgmsg_to_cv2(img, "bgr8")
    	# img=Check_Homography(empty_license_img, img)
    	pixmap = self.convert_cv_to_pixmap(img)
        self.car_view_label.setPixmap(pixmap)

    def convert_cv_to_pixmap(self, cv_img):
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = cv_img.shape
        bytesPerLine = channel * width
        q_img = QtGui.QImage(cv_img.data, width, height, 
                    bytesPerLine, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(q_img)

def Check_Homography(img, driving_feed):
	img=cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGRss)
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


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    myApp = comp_App()
    rospy.init_node('not_driver', anonymous=True)
    myApp.show()
    sys.exit(app.exec_())


