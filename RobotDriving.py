import cv2
import rospy
import os
from time import sleep
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError
from rospy.rostime import get_rostime
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

MODULE_NAME = "Driving Module"

def fprint(*args):
    """
    Custom print function that adds the module name to any print statements
    @return None
    @author Lukas
    """
    print(MODULE_NAME + ": " + " ".join(map(str,args)))

class drivingHandler():
    INTERSECTION_THRESHOLD = 2200000
    MOVEMENT_THRESHOLD = 50000
    STILL_THRESHOLD = 10000 # Summed number of pixel values where we can expect to see movement 
    STOP_LINE_THRESHOLD = 1500000 # Summed number of red pixel values where we can expect to see a red stop line
    
    CROSSWALK_TIME = 5
    INTERSECTION_TIME = 5


    def __init__(self):
        """
        Initializes a new drivingHandler that subscribes to the robots camera feed and publishes movement controls 
        @return None
        @author Lukas
        """
        
        self.twist_pub = rospy.Publisher("R1/cmd_vel", Twist, queue_size=1) # Handles car communication
        self.img_num = 0 # Stores how many images have been seen
        self.twist = Twist() # Car's velocity
        self.previous_img = None # Previously seen image (used for motion detection not always up to date)
        self.clear_time = 0 # Counts how many frames without movement have been seen   
        self.waited = False # Stores whether the car is waiting at a cross walk or not
        self.stop_time = 0 
        self.pause = True
        self.cross_time = 0 # Stores the time the car started to cross a crosswalk at
        
        self.intersection_time = 0 # Stores the time the car started going throught an intersection
        self.start_time = rospy.get_rostime().secs

        sleep(1) # imporant for initializing publishers

        self.start_time = rospy.get_rostime().secs # Stores how long the car has been going for
        
        
    def startHandler(self):
        start_time = rospy.get_rostime().secs
        self.twist_(0.4,1)
        self.pause = True
        fprint("Starting")
        
        while start_time + 1 > rospy.get_rostime().secs:
            
            pass
        self.pause = False
        fprint("Done Start Section")


    def processImg(self, img):
        """
        grayscales the image from the green channel, crops, shrinks, blurs, binary filters an image

        @return processed image
        @author Lukas
        """
        
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
        kernel = np.ones((20,20),np.uint8)
        Tkernel = np.ones((20,5),np.uint8)
        img = cv2.erode(img,kernel,iterations = 1)
        img = cv2.dilate(img, Tkernel, iterations = 1)
        self.img = img
        return img

    def getOffset(self, img):
        """
        Checks how far the center of the road is from the center of the image

        @return number of pixels that the center is off by 
                (- means road center is to the left)
                (+ means road center is to the right)
        @author Lukas
        """
        intersection = False
        img = self.processImg(img)
        M = cv2.moments(img)
        shape = img.shape
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except:
            fprint("Error finding Moment, turning right")
            return intersection, shape[1]/2
        cv2.circle(img, (cX, cY), 5, (0), -1)
        cv2.putText(img, ".", (cX, cY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow("Road Following", img)
        s = np.sum(img)
        fprint("s = ", s)
        if s > self.INTERSECTION_THRESHOLD:
            fprint("Upcoming intersection")
            intersection = True
        else: 
            fprint("On the road")
            intersection = False
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            cv2.destroyAllWindows()
        
        return intersection, cX - shape[1]/2
    
    def checkCrosswalk(self, img):
        """
        Checks if there is a crosswalk stopline ahead 

        @return True if there is one, False otherwise
        @author Lukas
        """
        # shape = img.shape
        # img = cv2.resize(img, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
        # img = cv2.GaussianBlur(img, (55,55), cv2.BORDER_DEFAULT)
        n =  np.sum(cv2.inRange(img[500:], (0,0,200), (100,100,255)))
        # fprint("Crosswalk handler: ", n)
        return n > self.STOP_LINE_THRESHOLD

    def checkMovement(self, img):
        """
        Checks if there is moment in an image
        
        @return True if there is movement, False otherwise
        @author Lukas
        """
        if self.previous_img is None:
            shape = img.shape
            img = cv2.resize(img, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
            self.previous_img = cv2.GaussianBlur(img, (55,55), cv2.BORDER_DEFAULT)
            return True
        else:
            shape = img.shape
            img = cv2.resize(img, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
            img = cv2.GaussianBlur(img, (55,55), cv2.BORDER_DEFAULT)
            diff_img = img - self.previous_img
            kernel = np.ones((7,7),np.uint8)
            diff_img = cv2.erode(diff_img, kernel,iterations = 1)
            fprint("movement: " + str(np.sum(diff_img)))
            self.previous_img = img

            s  = np.sum(diff_img)
            if s > self.MOVEMENT_THRESHOLD:
                fprint("movement: ", s)
        
            else:
                fprint("No movement: ", s)

            cv2.imshow("Movement", cv2.resize(diff_img, (int(shape[1]/2), int(shape[0]/2)), interpolation = cv2.INTER_CUBIC))
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
            if np.sum(diff_img) > self.MOVEMENT_THRESHOLD and self.stop_time + 2 < rospy.get_rostime().secs:
                self.waited = True
            fprint("Waited: ", self.waited)
            return np.sum(diff_img) > self.STILL_THRESHOLD
    
    def twist_(self, x, z):
        """
        Publishes the cars twist
        @return None
        @author Lukas
        """
        if not self.pause:
            self.twist.linear.x = x
            self.twist.angular.z = z
            self.twist_pub.publish(self.twist)

    def crosswalkHandler(self, img):
        """
        Checks an image to see if there is a crosswalk, and if so controls the robot throught
        without hitting a pedestrian 

        @return True if at cross walk 
        @author Lukas
        """
        if self.checkCrosswalk(img):
            self.twist_(0,0)
            if self.stop_time + 5 < rospy.get_rostime().secs:
                    self.stop_time = rospy.get_rostime().secs
            if not self.checkMovement(img):
                fprint("All clear " + str(self.clear_time))
                
                if self.waited and self.clear_time + 0.5 < rospy.get_rostime().secs:
                    fprint("Crossing")
                    self.cross_time = rospy.get_rostime().secs
                    self.clear_time = rospy.get_rostime().secs
                    self.pause = False
                    return False

                else:
                    self.pause = True
                    fprint("Waiting")                
                    return True

            else:
                fprint("Waiting at Crosswalk")
                self.twist_(0,0)
                self.pause = True
                self.clear_time = rospy.get_rostime().secs
                sleep(0.01)
                return True
        else:
            self.waited = False
            return False


    def drive(self, img):
        """
        Takes the robots camera image and controls the robot to follow the road
        @return None
        @author Lukas
        """
        if self.cross_time + self.CROSSWALK_TIME < rospy.get_rostime().secs:
            if self.crosswalkHandler(img):
                return None
        
        intersection, offset = self.getOffset(img)

        fprint(offset)
        P = 0.01

        az = -2*P*offset
        lx = max(0.5 - P*np.abs(offset),0)

        if intersection:
            az -= 0.5
        
        if self.cross_time + 2 > rospy.get_rostime().secs:
            fprint("BIASED")
            lx = max(lx, 0.5)
    
        self.twist_(lx , az)



if __name__ == '__main__':
    fprint("starting Script")
    fprint("New Branch")
    rospy.init_node('driver', anonymous=True)
    d = drivingHandler()
    d.lp_pub.publish('TeamRed,multi21,0,AA00')
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        d.twist_(0,0)
        cv2.destroyAllWindows()
        fprint("Stopping line_following") 