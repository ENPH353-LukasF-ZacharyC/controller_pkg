import cv2
import numpy as np

def get_license_plates(locb):
    lolp = []
    for img in locb:
        i = img
        img = cv2.GaussianBlur(img, (5, 5), 0)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img = cv2.inRange(img, (14,0,143), (137,88,195))
        kernel = np.ones((7,7),np.uint8)
        img = cv2.erode(img, kernel, iterations = 1)
        img = cv2.dilate(img,kernel, iterations= 3)
        canny = (cv2.Canny(img, 0, 50, apertureSize=5))
        contours, _hierarchy = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        squares = []
        for cnt in contours:
                    cnt_len = cv2.arcLength(cnt, True)
                    cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                    if len(cnt) == 4 and cv2.isContourConvex(cnt) and cv2.contourArea(cnt) > 5000:
                        print(cv2.contourArea(cnt))
                        squares.append(cnt)
        for s in squares:
            pts1 = corner_sorter(s)
            pts2 = np.float32([[0,0],[100,0],[0,30],[100,30]])
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(i,M,(100,30)) 
            cv2.imshow("LP", dst)
            lolp.append(dst)
    return lolp
    

def get_car_back(img):
    carBacks = []
    i = img[300:]
    img = cv2.GaussianBlur(img[300:], (5, 5), 0)
    img = cv2.inRange(img, (110,110,110), (225,225,225))
    kernel = np.ones((20,20),np.uint8)
    img = cv2.erode(img, kernel, iterations = 1)
    img = cv2.dilate(img,kernel, iterations= 2)
    canny = (cv2.Canny(img, 0, 50, apertureSize=5))
    contours, _hierarchy = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for cnt in contours:
        cnt_len = cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
        if len(cnt) == 4 and cv2.isContourConvex(cnt) and cv2.contourArea(cnt) > 10000:
            squares.append(cnt)
    for s in squares:
        pts1 = corner_sorter(s)
        pts2 = np.float32([[0,0],[150,0],[0,150],[150,150]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(i,M,(150,150)) 
        carBacks.append(dst)
    return carBacks

def corner_sorter(lop):
    lop = list(lop)
    print(lop[0][0])
    s = sorted(lop, key = lambda x : x[0][0])
    left = sorted(s[:2], key = lambda x : x[0][1])
    right = sorted(s[2:], key = lambda x : x[0][1])
    return np.array([left[0][0], right[0][0], left[1][0], right[1][0]], np.float32)