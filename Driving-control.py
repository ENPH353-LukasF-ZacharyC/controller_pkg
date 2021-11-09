import cv2
import rospy
import os
from time import sleep
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError
#roslib.load_manifest('2020_competition')
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

def fprint(s):
    print("Driving Module: " + str(s))

class drivingController():
    MOVEMENT_THRESHOLD = 2500000
    STOP_LINE_THRESHOLD = 2500000

    def __init__(self):
        self.twist_pub = rospy.Publisher("R1/cmd_vel", Twist, queue_size=1)
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.followPath)
        self.img_num = 0
        self.bridge = CvBridge()
        self.twist = Twist()
        self.previous_img = None
        self.clear_count =0
        self.waited = False
    
    def processImg(self, img):
        # os.chdir("/home/fizzer/ros_ws/src/controller_pkg/imgs/")
        # # cv2.imwrite("img_" + str(self.img_num) + ".png", img)
        # self.img_num += 1
        img[:, :, 0] = img[:, :, 1] # removes the Red 
        img[:, :, 2] = img[:, :, 1] # and Blue Channels
        img = img[400:, 200:1200]
        shape = img.shape
        img = cv2.resize(img, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
        img = cv2.GaussianBlur(img,(5,5),0)
        color_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        gray_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2BGR)
        # ret, img = cv2.threshold(gray_img,120,255,cv2.THRESH_BINARY_INV)
        img = cv2.inRange(gray_img, (80, 80, 80), (90, 90, 90))
        kernel = np.ones((9,9),np.uint8)
        img = cv2.erode(img,kernel,iterations = 1)
        self.img = img
        return img

    def getOffset(self, img):
        img = self.processImg(img)
        M = cv2.moments(img)
        shape = img.shape
        try:
            cX = int(M["m10"] / M["m00"])
            # cY = int(M["m01"] / M["m00"])
        except:
            fprint("Error finding Moment, turning right")
            return shape[1]/2
        
        return cX - shape[1]/2
    
    def checkCrosswalk(self, img):
        # shape = img.shape
        # img = cv2.resize(img, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
        # img = cv2.GaussianBlur(img, (55,55), cv2.BORDER_DEFAULT)
        return np.sum(cv2.inRange(img[550:], (0,0,200), (100,100,255))) > self.STOP_LINE_THRESHOLD


    def doneCrossing(self, img):
        if self.previous_img is None:
            shape = img.shape
            img = cv2.resize(img, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
            self.previous_img = cv2.GaussianBlur(img, (55,55), cv2.BORDER_DEFAULT)
            return False
        else:
            shape = img.shape
            img = cv2.resize(img, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
            img = cv2.GaussianBlur(img, (55,55), cv2.BORDER_DEFAULT)
            diff_img = img - self.previous_img
            kernel = np.ones((3,5),np.uint8)
            diff = cv2.erode(diff_img, kernel,iterations = 1)
            fprint("movement: " + str(np.sum(diff_img)))
            self.previous_img = img
            return np.sum(diff_img) < self.MOVEMENT_THRESHOLD
    
    def twist_(self, x, z):
        self.twist.linear.x = x
        self.twist.linear.z = z
        self.twist_pub.publish(self.twist)

    def crosswalkHandler(self, img):
        if self.checkCrosswalk(img):
            self.twist_(0,0)
            if self.doneCrossing(img):
                fprint("All clear " + str(self.clear_count))
                self.clear_count += 1
                if self.waited and self.clear_count < 25:
                    fprint("Waiting")
                    self.clear_count += 1
                    return False
                else:
                    fprint("Crossing")
                    self.twist_(0.5,0)
                    sleep(0.75) # gives enough time for car to cross
                    self.clear_count = 0
                    self.waited = False
                    return True
            else:
                fprint("Waiting at Crosswalk")
                self.waited = True
                self.twist_(0,0)
                self.clear_count = 0
                sleep(0.01)
                return False
        else:
            return True

    def followPath(self, img):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        if not self.crosswalkHandler(img):
            return None

        offset = self.getOffset(img)
        fprint(offset)
        P = 0.02

        self.twist_(max(0.3 - P*np.abs(offset),0), -P*offset)
        # print("X: ", self.twist.linear.x)
        # print("Z: ", self.twist.angular.z)




#if __name__ == '__main__':
fprint("starting Script")

d = drivingController()
rospy.init_node('driver', anonymous=True)

try:
    rospy.spin()
except KeyboardInterrupt:
    fprint("Stopping line_following") 