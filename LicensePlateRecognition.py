import rospy
from std_msgs.msg import String
import roslib
import tensorflow as tf

import cv2
import numpy as np

from collections import Counter

from tensorflow import keras

from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

sess1 = tf.compat.v1.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)

MODULE_NAME = "Car Finding"

def fprint(*args):
    """
    Custom print function that adds the module name to any print statements
    @return None
    @author Lukas
    """
    print(MODULE_NAME + ": " + " ".join(map(str,args)))

def get_lp_letters(img):
    """
    Takes a image from the car and finds all the letters on parked cars in the image
    @return bool: if there are letters, list: list of letters from left to right (None if there are less than 4)
    @author Lukas
    """
    # fprint("Getting plates")
    lp = get_license_plates(img)
    filtered =[]
    for img in lp:
        img = cv2.cvtColor(img[:, 30:img.shape[1]-50], cv2.COLOR_BGR2HSV)
        i = cv2.inRange(img, (117,117, 0), (255,255,255))
        Wkernel = np.ones((10,1),np.uint8)
        Tkernel = np.ones((1,5),np.uint8)
        kernel = np.ones((5,5), np.uint8)
        i = cv2.erode(i, Tkernel, iterations = 1)
        i = cv2.dilate(i,kernel, iterations= 1)
        filtered.append(i)
    letters = []
    for img in filtered:    
        _, contours, _hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = list(filter(lambda x : cv2.arcLength(x, True) > 200, contours))
    
        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
        for i, c in enumerate(contours):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            boundRect[i] = cv2.boundingRect(contours_poly[i])

        boundRect = list(filter(lambda x : x[2]*x[3]>5000, boundRect))
        boundRect = sorted(boundRect, key = lambda x : x[2]*x[3], reverse=True)[:4]
        boundRect = sorted(boundRect, key = lambda x : x[0])
        
        if len(boundRect) < 4:
            return False, []

        drawing = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        for i in range(len(boundRect)):
            color = (255,255,255)
            cv2.drawContours(drawing, contours_poly, i, color)
            cv2.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
                (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)

        for i in range(len(boundRect)):
            x,y,w,h = boundRect[i]
            # fprint(w*h)
            im = img[y:y+h , x:x+w]
            
            Lkernel = np.ones((5,5), np.uint8)
            Skernel = np.ones((3,3), np.uint8) 
            im = cv2.dilate(im,Lkernel, iterations= 1)
            im = cv2.erode(im,Skernel, iterations= 1)
            letters.append(im)
            cv2.imshow("Letter: " + str(i), im)

    if len(letters) < 4:
        return False, []
    else:
        return True, letters

def get_license_plates(img):
    """
    gets all the license plates in a given image
    @returns a list of all license plates
    @author Lukas
    """
    locb = get_car_back(img)
    lp = []
    for img in locb:
        i = img
        img = cv2.GaussianBlur(img, (11, 11), 0)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        img = cv2.inRange(img, (100,0,30), (110,69,255))
        
        Tkernel = np.ones((15,1),np.uint8)
        Wkernel = np.ones((1,15),np.uint8)
        kernel = np.ones((10,10),np.uint8)

        img = cv2.dilate(img, kernel, iterations= 1)
        img = cv2.erode(img, Wkernel, iterations = 2)
        img = cv2.dilate(img, kernel, iterations= 1)
        img = cv2.erode(img, Wkernel, iterations = 1)
        img = cv2.dilate(img, kernel, iterations = 1)
        img = cv2.dilate(img, Wkernel, iterations = 5)
        img = cv2.erode(img, Tkernel, iterations = 1)

        _, contours, _hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        squares = []

        for cnt in contours:
            cnt_len = cv2.arcLength(cnt, True)
            cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
            if len(cnt) == 4 and cv2.isContourConvex(cnt) and cv2.contourArea(cnt) > 1500: 
                # fprint(cv2.contourArea(cnt))
                squares.append(cnt)

        for s in squares:
            pts1 = corner_sorter(s)
            pts2 = np.float32([[0,0],[200,0],[0,100],[200,100]])
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(i,M,(200,100))
            kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
            dst = cv2.filter2D(dst, -1, kernel)  
            dst = cv2.resize(dst, (600, 300))
            lp.append(dst)
            cv2.imshow("License plate", dst)
    return lp
    
def get_car_back(img):
    """
    Gets the image of a car back from an image
    @returns an image of a car back 
    @author Lukas
    """
    carBacks = []
    i = img[300:]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img = cv2.GaussianBlur(img[300:], (5, 5), 0)

    img = cv2.inRange(img, (0,0,86), (129,30,230)) #- cv2.inRange(img, (0,0,225), (255,255,255)) #+ cv2.inRange(img, (0,0,0), (0,0,70)) 

    Tkernel = np.ones((12,5),np.uint8)
    Wkernel = np.ones((5,12),np.uint8)
    kernel = np.ones((10,10),np.uint8)

    img = cv2.erode(img, Tkernel, iterations = 1)
    img = cv2.dilate(img, Tkernel, iterations= 1)
    img = cv2.dilate(img, kernel, iterations= 1)
    img = cv2.erode(img, kernel, iterations = 1)
    img = cv2.dilate(img, kernel, iterations= 2)

    img = cv2.erode(img, Tkernel, iterations = 2)
    img = cv2.dilate(img, kernel, iterations = 2)
    img = cv2.erode(img, Tkernel, iterations = 3)

    _, contours, _hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    squares = []
    for cnt in contours:
        cnt_len = cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
        if len(cnt) == 4 and cv2.isContourConvex(cnt) and cv2.contourArea(cnt) > 7500: # and cv2.contourArea(cnt) > 1000, 
            # fprint(cv2.contourArea(cnt))
            squares.append(cnt)
    
    for s in squares:
        x = 100
        y = 200
        pts1 = corner_sorter(s)
        pts2 = np.float32([[0,0],[x,0],[0,y],[x,y]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(i,M,(x,y))
        carBacks.append(dst)
        cv2.imshow("Car Back", dst)
    return carBacks

def corner_sorter(lop):
    """
    Sorts a set of points in form [[[x1,y1]], [[x2,y2]], [[x3,y3]], [[x4,y4]]]]
    @returns the list of points in order top left, top right, bottom left, bottom right
    @author Lukas
    """
    lop = list(lop)
    s = sorted(lop, key = lambda x : x[0][0])
    left = sorted(s[:2], key = lambda x : x[0][1])
    right = sorted(s[2:], key = lambda x : x[0][1])
    return np.array([left[0][0], right[0][0], left[1][0], right[1][0]], np.float32)

class licensePlateHandler():

    def __init__(self):
        self.letter_num = 0
        self.lp_pub = rospy.Publisher("/license_plate", String, queue_size=1)
        self.model = self.load_model()
        self.ps_plates = {"1": [], "2": [], "3": [], "4": [], "5": [], "6": [], "7": [], "8": []}
        self.ps_order = ["2","3","4","5","6","1","7","8"]
        self.current_ps_index = 0
        self.time_looking_for_lp = 0

    def load_model(self):
        return keras.models.load_model("Letter_Identification_NN.h5")

    def vector_to_str(self, vector):
        m = np.argmax(vector)
        if m >= 10:
            return chr(m+55)
        else:
            return chr(m+48)
    
    def reportLicensePlate(self, img):
        status, letters = get_lp_letters(img)
        spot = self.ps_order[self.current_ps_index]
        # status = False
        # for letter in letters:
        #     cv2.imwrite("letter_" + str(self.letter_num) + ".png", letter)
        #     self.letter_num += 1
        if status is True and len(letters) == 4:
            resized=[]
            for i in letters:
                dim = (95, 125)
                r = cv2.resize(i, dim, interpolation = cv2.INTER_AREA)
                r = np.expand_dims(r, axis = 2)
                r = np.expand_dims(r, axis = 0)
                resized.append(r)

            prediction=""
            global sess1
            global graph1
            with graph1.as_default():
                set_session(sess1)
                for letter_img in resized:
                    p=self.vector_to_str(self.model.predict(letter_img)[0])
                    prediction += p

                # print(prediction)
                self.ps_plates[spot].append(prediction)

        if len(self.ps_plates[spot]) != 0 and self.time_looking_for_lp == 0:
            self.time_looking_for_lp = rospy.get_rostime().secs

        if rospy.get_rostime().secs - self.time_looking_for_lp > 1.75 and self.time_looking_for_lp != 0:
            potential_winners = self.ps_plates[spot]
            plate_count = Counter(potential_winners)
            top = plate_count.most_common(1)
            print("//////////////////////////////////////////////////")
            fprint("The most frequent license plate is: "+ top[0][0])
            fprint("At spot: "+ str(spot))
            print("//////////////////////////////////////////////////")
            # fprint('TeamRed,multi21,{},{}'.format(str(spot), top[0][0]))
            self.lp_pub.publish('TeamRed,multi21,{},{}'.format(str(spot), top[0][0]))
            self.current_ps_index += 1
            self.time_looking_for_lp = 0


        # else:
        #     if len(self.potential_winner) != 0:
        #         possible_plates = Counter(self.potential_winner)
        #         top = possible_plates.most_common(1)
        #         print("The most frequent license plate is: ")
        #         print(top[0][0])
        #         self.potential_winner=[]




        # TODO ZACH LETTER RECOGNITION
