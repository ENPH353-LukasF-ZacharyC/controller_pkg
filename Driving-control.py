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

class drivingController():
    def __init__(self):
        self.twist_pub = rospy.Publisher("R1/cmd_vel", Twist, queue_size=1)
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.follow_path)
        self.img_num = 0

        print("model loaded")
        self.bridge = CvBridge()
        self.twist = Twist()
    
    def processImg(self,img):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
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
            cY = int(M["m01"] / M["m00"])
        except:
            print("error")
            return shape[1]/2
        
        print(cX - shape[1]/2, cY - shape[0]/2)
        cv2.circle(img, (cX, cY), 5, (0), -1)
        cv2.putText(img, ".", (cX, cY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # cv2.imshow("Centroid", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.waitKey(1)
        return cX - shape[1]/2
    
    def follow_path(self, img):
        offset = self.getOffset(img)
        print(offset)
        P = 0.02

        self.twist.linear.x = max(0.3 - P*np.abs(offset),0)
        self.twist.angular.z = -P*offset
        print("X: ", self.twist.linear.x)
        print("Z: ", self.twist.angular.z)
        self.twist_pub.publish(self.twist)

#if __name__ == '__main__':
print("starting Script")
#os.chdir("/home/fizzer/Documents/ros_driving")
d = drivingController()
rospy.init_node('driver', anonymous=True)
try:
    # while True:
    #     cv2.imshow("Centroid", d.img)
    rospy.spin()
except KeyboardInterrupt:
    print("Stopping line_following") 
    #     cv2.ims