#!/usr/bin/env python
#coding=utf-8
from re import T
import rospy
import numpy as np
from Python_API import Sendmessage                  #PythonAPI
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time
import math
MULT = 4
LENTH = 216/MULT
WIDTH = 240/MULT
send = Sendmessage()
#             Orange   Yellow      Blue        Green     Black   Red         White
COLOR_MASK = [[0,0,0],[128,128,0],[128,0,128],[0,0,128],[0,0,0],[255,255,0],[0,0,0]]
class deep_calculate:
    def __init__(self,color):
        self.bridge = CvBridge()
        # 訂閱攝像頭資訊 #"/kidsize/camera/image_raw" #"compress_image" #"/usb_cam/image_raw"
        #colormodel_image  orign_image
        self.Image_compress_sub = rospy.Subscriber("colormodel_image",Image, self.convert)			
        self.init()
        self.first = True
        self.line_ = True
        self.color = color
        self.output =[]
        self.chage_cnt = 0
        self.reset_cnt = 0

    def init(self):
        self.now_line_coordinate    = [-9999,-9999,-9999,-9999]
        self.last_line_coordinate   = [-9999,-9999,-9999,-9999]

    # 讀取圖片
    def convert(self, imgmsg):
        try:                             #影像通訊
            img = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        except CvBridgeError as e:
            print(e)
        img = cv2.resize(img, (int(LENTH),int(WIDTH)))
        self.edge(img,self.color)
        # cv2.imshow("Image_show",self.output)
        # cv2.waitKey(1)
        send.drawImageFunction(14,0,self.now_line_coordinate[0],self.now_line_coordinate[2],self.now_line_coordinate[1],self.now_line_coordinate[3],255,255,0)
        # print(self.now_line_coordinate)
        if abs(self.now_line_coordinate[1] - self.now_line_coordinate[3]) == 0:
            # print("0")
            self.slope = 0
        else:
            self.slope = (self.now_line_coordinate[1] - self.now_line_coordinate[3]) / abs(self.now_line_coordinate[0] - self.now_line_coordinate[2])*10
            # print((self.now_line_coordinate[1] - self.now_line_coordinate[3]) / abs(self.now_line_coordinate[0] - self.now_line_coordinate[2]))
        # return self.now_line_coordinate

    def edge(self, img, color):
        size = 0
        for i in range(0, send.color_mask_subject_cnts[color]):
            if send.color_mask_subject_size[color][i] > 5000:
                size = send.color_mask_subject_size[color][i]
                break

        for i in range(28, int(LENTH), 1):
            for j in range(0, int(WIDTH), 1):
                if img.item(j, i, 0) != COLOR_MASK[color][0] or img.item(j, i, 1) != COLOR_MASK[color][1] or img.item(j, i, 2) != COLOR_MASK[color][2]:
                    img[j, i] = (0, 0, 0)

        output = cv2.medianBlur(img, 15)
        edges = cv2.Canny(output, 30, 50)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 20, minLineLength=15, maxLineGap=10)

        if lines is not None:
            closest_line = None
            closest_distance = sys.maxsize
            for line in lines:
                x1, y1, x2, y2 = line[0]

                if abs(y1 - y2) / abs(x1 - x2) > 3:
                    continue

                distance_to_bottom = abs(y1 - int(WIDTH))
                if distance_to_bottom < closest_distance:
                    closest_line = line[0]
                    closest_distance = distance_to_bottom

            if closest_line is not None:
                x1, y1, x2, y2 = closest_line
                self.now_line_coordinate[0] = x1 * MULT
                self.now_line_coordinate[2] = x2 * MULT
                self.now_line_coordinate[1] = y1 * MULT
                self.now_line_coordinate[3] = y2 * MULT
                self.reset_cnt = 0
                self.first = False
        else:
            self.now_line_coordinate = self.last_line_coordinate

        if size < 5000:
            self.now_line_coordinate = [-9999, -9999, -9999, -9999]
            self.reset_cnt = 0

        self.last_line_coordinate = self.now_line_coordinate
        # cv2.line(output, (self.now_line_coordinate[0] // 2, self.now_line_coordinate[1] // 2),
        #         (self.now_line_coordinate[2] // 2, self.now_line_coordinate[3] // 2), (0, 0, 255), 5, lineType=cv2.LINE_AA)

        self.output = output

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            # send = Sendmessage()
            if send.Web == True:
                pass
            if send.Web == False:
                deep_calculate(5)
                rospy.spin()
    except rospy.ROSInterruptException:
        pass