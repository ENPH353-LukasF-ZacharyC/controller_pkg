from RobotDriving import drivingHandler
from LicensePlateRecognition import licensePlateHandler

import os
from time import sleep
import threading
import cv2

import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

MODULE_NAME = "Main"

def fprint(*args):
    """
    Print wrapper function that adds the module name to any print statements
    @return None
    @author Lukas
    """
    print(MODULE_NAME + ": " + " ".join(map(str,args)))

class main():
    RUN_TIME = 1000 # How many seconds the robot should be running for

    def __init__(self):
        self.bridge = CvBridge()
        self.drivingHandler = drivingHandler()
        self.licensePlateHandler = licensePlateHandler()
                
        self.lp_pub = rospy.Publisher("/license_plate", String, queue_size=1)
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.protocol) # Handles car video feed
        self.start_time = rospy.get_rostime().secs
        sleep(3)
        self.start_time = rospy.get_rostime().secs
        self.drivingHandler.startHandler()

    def protocol(self, img):
        if self.start_time + self.RUN_TIME  < rospy.get_rostime().secs:
            self.stop()
        else:
            img = self.bridge.imgmsg_to_cv2(img, "bgr8")
            img2 = img.copy()
            self.drivingHandler.drive(img)
            self.licensePlateHandler.reportLicensePlate(img2)



    def stop(self):
        self.lp_pub.publish('TeamRed,multi21,-1,0000')
        print("========================\n")
        print("Time's Up; Ending Script")
        print("\n========================")
        self.drivingHandler.twist_(0,0)
        cv2.destoryAllWindows()
        self.img_sub.unregister()

if __name__ == '__main__':
    fprint("starting Script")
    fprint("New Branch")
    rospy.init_node('driver', anonymous=True)
    m = main()
    m.lp_pub.publish('TeamRed,multi21,0,AA00')
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        fprint("Ending Script")
        m.stop()
        fprint("Script Ended") 