import os
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import *
import rospy
from time import sleep
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError
#roslib.load_manifest('2020_competition')
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

sess1 = tf.compat.v1.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)


class driver():
    def __init__(self):
        self.twist_pub = rospy.Publisher("R1/cmd_vel", Twist, queue_size=1)
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.follow_path)
        self.model = self.load_model()
        #tf.compat.v1.keras.models.load_model("/home/fizzer/ros_ws/src/2020_competition/enph353/enph353_gazebo/nodes/DM2")
        print("model loaded")
        self.bridge = CvBridge()
        self.twist = Twist()

    def load_model(self):
        """
        tmp = os.getcwd()
        os.chdir("/home/fizzer/ros_ws/src/2020_competition/enph353/enph353_gazebo/nodes/")
        with open('model_config.json') as json_file:
            json_config = json_file.read()
        new_model = tf.keras.models.model_from_json(json_config)

            # Load weights
        new_model.load_weights('weights_only.h5')
        os.chdir(tmp)
        """
        
        return keras.models.load_model("Driving_model.h5")

    def follow_path(self, img):
        processed = self.imgProcessing(img)
        global sess1
        global graph1
        prediction = None
        with graph1.as_default():
            set_session(sess1)
            prediction = self.model.predict(processed)[0]
            print(prediction)

        model_output = np.argmax(prediction)
        if model_output == 0:
            print("left")
            self.twist.linear.x = 0.02
            self.twist.angular.z = 0.2
        elif model_output == 1:
            print("straight")
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0
        elif model_output == 2:
            print("right")
            self.twist.linear.x = 0.02
            self.twist.angular.z = -0.2
    
        self.twist_pub.publish(self.twist)
        


    def imgProcessing(self, img):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        cropped = img[375:]
        shape = cropped.shape
        compressed = cv2.resize(cropped, (int(shape[1]/5), int(shape[0]/5)), interpolation = cv2.INTER_CUBIC)
        compressed[:, :, 0] = compressed[:, :, 2] # removes the green and blue channel i think
        compressed[:, :, 1] = compressed[:, :, 2]
        color_img = cv2.cvtColor(compressed, cv2.COLOR_RGB2GRAY)
        gray_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2BGR)
        ret,bin_img = cv2.threshold(gray_img,100,255,cv2.THRESH_BINARY)
        return np.expand_dims(bin_img, axis=0)

#if __name__ == '__main__':
print("starting Script")
#os.chdir("/home/fizzer/Documents/ros_driving")
d = driver()
rospy.init_node('driver', anonymous=True)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Stopping line_following")