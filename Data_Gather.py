##! /usr/bin/env python

import rospy
from time import sleep
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError
#roslib.load_manifest('2020_competition')
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import os
import pickle

class data_collection():
    def __init__(self):
        print(os.getcwd())
        self.current_twist = None
        self.twist_sub = rospy.Subscriber("R1/cmd_vel", Twist, self.get_twist)
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.record_img)
        self.current_twist = None
        self.data_dir = "/home/fizzer/ros_ws/src/2020_competition/enph353/enph353_gazebo/Data"
        self.img_num = self.get_img_num()
        self.bridge = CvBridge()

    def record_img(self, img):
        i = self.bridge.imgmsg_to_cv2(img, "bgr8")
        img = self.imgProcessing(i)
        d = (img, self.current_twist)
        if self.current_twist is None:
            print("Not Moving")
            return None
        with open(self.data_dir + "/data_" + str(self.img_num + 1) + ".p", "w") as file:
            self.img_num += 1
            pickle.dump(d, file)
            cv2.imwrite(str(self.img_num) + ".png", i)
            print(self.current_twist)
        print("image " + str(self.img_num + 1) + " recorded")

    def get_twist(self, t):
        if t.linear.x > 0 and t.angular.z == 0:
            self.current_twist = [0,1,0]
        elif t.linear.x == 0:
            if t.angular.z > 0:
                self.current_twist = [1,0,0]
            elif t.angular.z < 0:
                self.current_twist = [0,0,1]
        else:
            self.current_twist = None
        

    def imgProcessing(self,img):
        cropped = img[375:]
        shape = cropped.shape
        compressed = cv2.resize(cropped, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
        #compressed[:, :, 0] = compressed[:, :, 2] # removes the green and blue channel
        #compressed[:, :, 1] = compressed[:, :, 2]
        #color_img = cv2.cvtColor(compressed, cv2.COLOR_RGB2GRAY)
        #gray_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2BGR)
        #ret,bin_img = cv2.threshold(gray_img,100,255,cv2.THRESH_BINARY)
        return compressed

    def get_img_num(self):
        max_num = 0
        for data in os.listdir(self.data_dir):
            max_num = max(max_num, int(data[5:-2]))
        return max_num

if __name__ == '__main__':
    collect = data_collection()
    rospy.init_node('data_collection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stopping line_following")
