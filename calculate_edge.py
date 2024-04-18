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
MULT = 1
LENTH = 320/MULT
WIDTH = 240/MULT
send = Sendmessage()
#             Orange   Yellow      Blue        Green     Black   Red         White
COLOR_MASK = [[0,0,0],[128,128,0],[128,0,128],[0,0,128],[0,0,0],[255,255,0],[0,0,0]]
class deep_calculate:
    def __init__(self,color,layer):
        self.bridge = CvBridge()
        # 訂閱攝像頭資訊 #"/kidsize/camera/image_raw" #"compress_image" #"/usb_cam/image_raw"
        #colormodel_image  orign_image
        self.Image_compress_sub = rospy.Subscriber("colormodel_image",Image, self.convert)			
        self.init()
        self.color = color
        self.layer = layer
        self.output =[]


    def init(self):
        self.now_line_coordinate    = [-9999,-9999,-9999,-9999]
        self.last_line_coordinate   = [-9999,-9999,-9999,-9999]

    # 影像判斷更新
    def convert(self, imgmsg):
        try:                             #影像通訊
            img = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        except CvBridgeError as e:
            print(e)
        img  = img[115:215,120:220]
        
        # img = cv2.resize(img, (int(LENTH),int(WIDTH)))
        self.edge(img,self.color)

        ##----測試用---##
        # cv2.imshow("Image_show",self.output)
        # cv2.waitKey(1)
        ##------------##

        send.drawImageFunction(999,0,self.now_line_coordinate[0],self.now_line_coordinate[2],self.now_line_coordinate[1],self.now_line_coordinate[3],255,255,0)
        #計算斜率
        if abs(self.now_line_coordinate[1] - self.now_line_coordinate[3]) == 0:
            self.slope = 0
        else:
            self.slope = (self.now_line_coordinate[1] - self.now_line_coordinate[3]) / abs(self.now_line_coordinate[0] - self.now_line_coordinate[2])*10

    def calc_slope(self, x0, y0, x1, y1):
        if x0 == x1:
            #無限e
            return float("inf")
        return (y1 - y0) / (x1 - x0)

    def edge(self, img, color):
        size = 0
        #限制物件大小
        for i in range(0, send.color_mask_subject_cnts[color]):
            if send.color_mask_subject_size[color][i] > 5000:
                # size = send.color_mask_subject_size[color][i]
                break
            else:
                self.now_line_coordinate  = [-9999, -9999, -9999, -9999]
                self.last_line_coordinate = [-9999,-9999,-9999,-9999]
                return
        #遮罩
        for i in range(0, 100, 1):
            for j in range(0, 100, 1):
                if img.item(j, i, 0) != COLOR_MASK[color][0] or img.item(j, i, 1) != COLOR_MASK[color][1] or img.item(j, i, 2) != COLOR_MASK[color][2]:
                    img[j, i] = (0, 0, 0)
        #中值濾波
        output = cv2.medianBlur(img, 15)
        #邊緣偵測
        edges = cv2.Canny(output, 30, 50)
        #霍夫曼機率找線(圖片,像素單位,角度精度(弧度),threshod:線段偵測閥值(越大偵測的線越少),minLineLength:線段最小偵測長度,maxLineGap:兩個線段允許最大間隔)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=15, minLineLength=10, maxLineGap=10)

        if lines is not None:
            if self.layer < 4:
                lines_y = [max([line[0][1], line[0][3]]) for line in lines]
                min_y_idx = np.argmax(lines_y)
            else:
                lines_slope = [abs(self.calc_slope(line[0][0], line[0][1], line[0][2], line[0][3])) for line in lines]
                min_y_idx = np.argmin(lines_slope)

            closest_line = None
            
            # for line in lines:
            if True:
                line = lines[min_y_idx]
                x1, y1, x2, y2 = line[0]

                # if abs((y1 - y2 )/ (x1 - x2)) > 1.5 or abs(y1-y2) > 40:
                #     continue
                if self.layer < 4:
                    closest_distance = sys.maxsize
                    distance_to_bottom = abs(max(y1,y2) - int(WIDTH))
                    if distance_to_bottom < closest_distance:
                        closest_line = line[0]
                        closest_distance = distance_to_bottom
                else:
                    closest_distance = -sys.maxsize
                    distance_to_upper = abs(min(y1,y2) - int(WIDTH))
                    if distance_to_upper > closest_distance:
                        closest_line = line[0]
                        closest_distance = distance_to_upper
            

            if closest_line is not None:
                x1, y1, x2, y2 = closest_line
                self.now_line_coordinate[0] = x1+115
                self.now_line_coordinate[2] = x2+115
                self.now_line_coordinate[1] = y1+120
                self.now_line_coordinate[3] = y2+120
        else:
            self.now_line_coordinate = self.last_line_coordinate

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