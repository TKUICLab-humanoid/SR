#!/usr/bin/env python
#coding=utf-8

# 校正變數
# 第幾層
# 改線
# 上板空間不夠
# 下板空間不夠
#revision
import rospy
import numpy as np
from rospy import Publisher
# from tku_msgs.msg import Interface,HeadPackage,SandHandforward,DrawImage,SingleMotorData,SensorSet,ObjectList,LabelModelObjectList,RobotPos,SetGoalPoint,SoccerDataList,SensorPackage
from std_msgs.msg import Int16,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import cv2
import time
from Python_API import Sendmessage

send = Sendmessage()#要放在class外面,不然不能宣告
BLUE   = 2
RED    = 5
YELLOW = 1
GREEN  = 3

#--校正量--#
#校正變數       #revision
#前進量校正
FORWARD_CORRECTION        = 0
#平移校正
TRANSLATION_CORRECTION    = 500
#旋轉校正
THETA_CORRECTION          = 0
#---微調站姿開關---#
STAND_CORRECT             = True
UPBOARD_CORRECT           = True
DOWNBOARD_CORRECT         = True
START_LAYER               = 2
#-----------------#
#----------#
FOOTBOARD_LINE            = 220                   #上板基準線
#----------#                [ 左, 中, 右]
LEFT_FOOT                 = [120,130,140]
RIGHT_FOOT                = [180,190,200]
HEAD_HORIZONTAL           = 2055                  #頭水平
HEAD_VERTICAL             = 2690                  #頭垂直 1472#
#----------#
WARNING_DISTANCE          = 4                     #危險距離
GO_UP_DISTANCE            = 8                     #上板距離
FIRST_FORWORD_CHANGE_LINE = GO_UP_DISTANCE + 15   #
SECOND_FORWORD_CHANGE_LINE = GO_UP_DISTANCE + 45  #
THIRD_FORWORD_CHANGE_LINE = GO_UP_DISTANCE + 60   #
ANGLE_REVISION_DISTANCE   = 100                   #角度修正距離
UP_BOARD_DISTANCE         = 30                    #最低上板需求距離
#----------#
BACK_MIN                  = -500                  #小退後
BACK_NORMAL               = -1000                 #退後
FORWARD_MIN               = 200                   #小前進
FORWARD_NORMAL            = 800                   #前進
FORWARD_BIG               = 1300                  #大前進
FORWARD_SUPER             = 2500                  #超大前進
#----------#
TRANSLATION_MIN           = 500                   #小平移
TRANSLATION_NORMAL        = 1000                  #平移
TRANSLATION_BIG           = 1500                  #大平移
#----------#
THETA_MIN                 = 1                     #小旋轉
THETA_NORMAL              = 3                     #旋轉
THETA_BIG                 = 5                     #大旋轉
#----------#
BASE_CHANGE               = 100                   #基礎變化量(前進&平移)
#----------#
LCUP                      = 16000                 #上板
LCDOWN                    = 20000                 #下板

class Send_distance():
    def __init__(self):#初始化
        #
        self.board_in_right        = False
        self.board_in_left         = False
        #
        self.walkinggait_stop      = True  
        self.walkinggait_LC        = False
        #上板延遲
        self.upboard_start         = 0
        self.upboard_end           = 0
        #層數       
        self.layer                 = START_LAYER
        self.layer_model           = [   GREEN,   BLUE,   RED,   YELLOW,   RED,   BLUE,   GREEN]
        self.layer_parameter       = [2**GREEN,2**BLUE,2**RED,2**YELLOW,2**RED,2**BLUE,2**GREEN]
        #設定頭部馬達
        self.head_Horizontal       = HEAD_HORIZONTAL
        self.head_Vertical         = HEAD_VERTICAL
        #距離矩陣                     [左左,左中,左右 ,右左,右中,右右 ]
        self.distance              = [9999,9999,9999,9999,9999,9999]
        self.next_distance         = [9999,9999,9999,9999,9999,9999]
        #板子資訊矩陣                  [X   ,   Y] 
        self.board_left_point      = [9999,9999]
        self.board_right_point     = [9999,9999]
        self.board_top_point       = [9999,9999]
        self.board_bottom_point    = [9999,9999]
        self.board_size            = 0
        #步態參數
        self.forward               = 0 + FORWARD_CORRECTION
        self.translation           = 0 + TRANSLATION_CORRECTION
        self.theta                 = 0 + THETA_CORRECTION
        self.last_translation      = 0
        self.last_theta            = 0
        self.LCup_x                = LCUP      #Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = 3.2
        self.LCdown_x              = LCDOWN    #Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = -1.5
        #左基礎參數
        self.left_theta            = 1
        #右基礎參數
        self.right_theta           = -1
        #前進基礎參數
        self.left_theta            = 1
        #後退基礎參數
        self.right_theta           = -1
        #左右決定
        self.decide_theta          = 0
        #危險斜率
        self.slope_min             = 5      #有點斜
        self.slope_normal          = 10      #斜
        self.slope_big             = 15      #過斜
    def find_board(self,state):
        #確認物件為板子#
        Object              = 0
        success_find_board  = False 
        Object              = send.color_mask_subject_cnts[self.layer_model[state]]
        self.distance       = [9999,9999,9999,9999,9999,9999]
        self.next_distance  = [9999,9999,9999,9999,9999,9999]
        if Object != 0:
            Object_size = 0
            print("find board")
            for i in range(Object):
                Object_size = send.color_mask_subject_size[self.layer_model[state]][i]
                if Object_size >10000:
                    self.board_left_point[0]    = send.color_mask_subject_XMin[self.layer_model[state]][i]
                    self.board_right_point[0]   = send.color_mask_subject_XMax[self.layer_model[state]][i]
                    self.board_top_point[1]     = send.color_mask_subject_YMin[self.layer_model[state]][i]
                    self.board_bottom_point[1]  = send.color_mask_subject_YMax[self.layer_model[state]][i]
                    self.board_size             = send.color_mask_subject_size[self.layer_model[state]][i]
                    success_find_board          = True
                    break
                else:#初始化
                    self.board_left_point[0]    = 9999
                    self.board_left_point[1]    = 9999
                    self.board_right_point[0]   = 9999
                    self.board_right_point[1]   = 9999
                    self.board_top_point[0]     = 9999
                    self.board_top_point[1]     = 9999
                    self.board_bottom_point[0]  = 9999
                    self.board_bottom_point[1]  = 9999
                    self.board_size             = 0
                    success_find_board = False           
        else:
            pass
        #-------------#
        #-------------#
        if success_find_board == True:
            #left_point
            for i in range (0,240):
                if send.Label_Model[320*i+self.board_left_point[0]]   == self.layer_parameter [state]:
                    self.board_left_point[1]    = i
                    break
            #right_point
            for i in range (0,240):
                if send.Label_Model[320*i+self.board_right_point[0]]  == self.layer_parameter [state]:
                    self.board_right_point[1]   = i
                    break
            #top_point
            for i in range (0,320):
                if send.Label_Model[320*self.board_top_point[1]+i]    == self.layer_parameter [state]:
                    self.board_top_point[0]     = i
                    break
            #bottom_point
            for i in range (0,320):
                if send.Label_Model[320*self.board_bottom_point[1]+i] == self.layer_parameter [state]:
                    self.board_bottom_point[0]  = i
                    break
        else:
            pass
        #----Leftfoot----# 
        for LL in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(LL,LEFT_FOOT[0],state):
                    self.distance[0] = FOOTBOARD_LINE - LL
                    break
        for LM in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(LM,LEFT_FOOT[1],state):
                    self.distance[1] = FOOTBOARD_LINE - LM
                    break
        for LR in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(LR,LEFT_FOOT[2],state):
                    self.distance[2] = FOOTBOARD_LINE - LR
                    break
        #----Rightfoot----#
        for RL in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(RL,RIGHT_FOOT[0],state):
                    self.distance[3] = FOOTBOARD_LINE - RL
                    break
        for RM in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(RM,RIGHT_FOOT[1],state):
                    self.distance[4] = FOOTBOARD_LINE - RM
                    break
        for RR in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(RR,RIGHT_FOOT[2],state):
                    self.distance[5] = FOOTBOARD_LINE - RR
                    break
        #-----------------#
        if self.layer == 6:
            pass
        elif self.layer != 3 :
            #----Leftfoot_next----# 
            for LL_2 in range(LL,10,-1):
                    if self.return_real_board(LL_2,LEFT_FOOT[0],state+1):
                        self.next_distance[0] = LL - LL_2
                        break
            for LM_2 in range(LM,10,-1):
                    if self.return_real_board(LM_2,LEFT_FOOT[1],state+1):
                        self.next_distance[1] = LM - LM_2
                        break
            for LR_2 in range(LR,10,-1):
                    if self.return_real_board(LR_2,LEFT_FOOT[2],state+1):
                        self.next_distance[2] = LR - LR_2
                        break
            #----Rightfoot_next----#
            for RL_2 in range(RR,10,-1):
                    if self.return_real_board(RL_2,RIGHT_FOOT[0],state+1):
                        self.next_distance[3] = RL - RL_2
                        break
            for RM_2 in range(RM,10,-1):
                    if self.return_real_board(RM_2,RIGHT_FOOT[1],state+1):
                        self.next_distance[4] = RM - RM_2
                        break
            for RR_2 in range(RR,10,-1):
                    if self.return_real_board(RR_2,RIGHT_FOOT[2],state+1):
                        self.next_distance[5] = RR - RR_2
                        break
            #上板空間
            send.drawImageFunction(14,1,LEFT_FOOT[0]-5,LEFT_FOOT[0]+5,LL_2-5,LL_2+5,128,255,128)
            send.drawImageFunction(15,1,LEFT_FOOT[1]-5,LEFT_FOOT[1]+5,LM_2-5,LM_2+5,128,255,128)
            send.drawImageFunction(16,1,LEFT_FOOT[2]-5,LEFT_FOOT[2]+5,LR_2-5,LR_2+5,128,255,128)
            send.drawImageFunction(17,1,RIGHT_FOOT[0]-5,RIGHT_FOOT[0]+5,RL_2-5,RL_2+5,128,255,128)
            send.drawImageFunction(18,1,RIGHT_FOOT[1]-5,RIGHT_FOOT[1]+5,RM_2-5,RM_2+5,128,255,128)
            send.drawImageFunction(19,1,RIGHT_FOOT[2]-5,RIGHT_FOOT[2]+5,RR_2-5,RR_2+5,128,255,128)
            send.drawImageFunction(20,1,self.board_bottom_point[0]-5,self.board_bottom_point[0]+5,self.board_bottom_point[1]-5,self.board_bottom_point[1]+5,128,255,128)
        else:
            pass

    def walkinggait(self,layer_now):
        if (self.distance[0] < GO_UP_DISTANCE)   and (self.distance[1] < GO_UP_DISTANCE+3) and\
           (self.distance[2] < GO_UP_DISTANCE+5) and (self.distance[3] < GO_UP_DISTANCE+5) and\
           (self.distance[4] < GO_UP_DISTANCE+3) and (self.distance[5] < GO_UP_DISTANCE):
            print("對正板子")
            self.upboard_start=time.time()
            while(self.upboard_end-self.upboard_start > 2):
                self.upboard_end=time.time()
                send.sendContinuousValue(-500,self.translation,0,self.theta,0)
                print("delay:",self.upboard_end-self.upboard_start)
            self.forward        = 0
            self.translation    = 0
            self.theta          = 0
            send.sendBodyAuto(0,0,0,0,1,0)                  #停止步態
            time.sleep(3)
            send.sendSensorReset()                          #IMU reset 避免機器人步態修正錯誤
            self.layer                 += 1                 #層數加一
            self.distance              = [9999,9999,9999,9999,9999,9999]
            self.next_distance         = [9999,9999,9999,9999,9999,9999]
            self.walkinggait_stop      = True
            self.walkinggait_LC        = True
            if layer_now < 3:
                if UPBOARD_CORRECT   == True:
                    print("準備上板")
                    send.sendBodySector(31)                 #上板前站姿調整
                    time.sleep(3)                           #微調站姿延遲
                send.sendBodyAuto(self.LCup_x,0,0,0,2,0)    #上板步態
            else:
                if DOWNBOARD_CORRECT == True:
                    print("準備下板")
                    send.sendBodySector(32)                 #下板前站姿調整
                    time.sleep(3)                           #微調站姿延遲
                send.sendBodyAuto(self.LCdown_x,0,0,0,3,0)  #上板步態
            time.sleep(5)
            send.sendBodySector(29)         #這是基本站姿的磁區
            time.sleep(1)
            if STAND_CORRECT == True:
                send.sendBodySector(30)     #基礎站姿調整
            time.sleep(1)
            self.checkout_board()
            time.sleep(1)
        else:
            self.edge_judge(layer_now)
            self.last_theta       = self.theta
            self.last_translation = self.translation
            self.speed_limit()
            send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)

    def edge_judge(self,state):
        #太靠板子
        if (self.distance[0] <= WARNING_DISTANCE) or (self.distance[1] <= WARNING_DISTANCE) or (self.distance[2] <= WARNING_DISTANCE) or (self.distance[3] <= WARNING_DISTANCE) or (self.distance[4] <= WARNING_DISTANCE) or (self.distance[5] <= WARNING_DISTANCE): 
            #即將踩板
            if  ((self.distance[0] <= WARNING_DISTANCE) and (self.distance[1] <= WARNING_DISTANCE) and (self.distance[4] > WARNING_DISTANCE) and (self.distance[5] > WARNING_DISTANCE)) or \
                ((self.distance[0] > WARNING_DISTANCE) and (self.distance[1] > WARNING_DISTANCE) and (self.distance[4] <= WARNING_DISTANCE) and (self.distance[5] <= WARNING_DISTANCE)):
                self.forward = BACK_NORMAL + FORWARD_CORRECTION
                self.theta_change(state)
                if self.theta>THETA_CORRECTION:
                    self.translation = self.right_theta * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                elif self.theta<THETA_CORRECTION:
                    self.translation = self.left_theta * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                else:
                    self.translation = TRANSLATION_CORRECTION
                print("!!!快踩板,後退!!!")
            #
            elif (self.next_distance[0] < WARNING_DISTANCE) or (self.next_distance[5] < WARNING_DISTANCE): 
                self.forward = BACK_MIN + FORWARD_CORRECTION
                self.theta_change(state)
                if self.theta>THETA_CORRECTION:
                    self.translation = self.right_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
                elif self.theta<THETA_CORRECTION:
                    self.translation = self.left_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
                else:
                    self.translation = TRANSLATION_CORRECTION
                print("!!!小心踩板,後退!!!")
            #90度板位在中間
            elif (self.board_bottom_point[0] > LEFT_FOOT[0]) and (self.board_bottom_point[0] < RIGHT_FOOT[2]):
                if  self.board_bottom_point[0] < ((LEFT_FOOT[2]+RIGHT_FOOT[0])/2):
                    self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                    self.theta       =  THETA_BIG*self.right_theta
                    self.translation = self.left_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
                    print("90板怎麼在中間,快左修")
                elif self.board_bottom_point[0] > ((LEFT_FOOT[2]+RIGHT_FOOT[0])/2):
                    self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                    self.theta       =  THETA_BIG*self.left_theta
                    self.translation = self.right_theta*  TRANSLATION_MIN + TRANSLATION_CORRECTION
                    print("90板怎麼在中間,快右修")
            #90度板位在雙腳前
            elif (self.board_bottom_point[0] < LEFT_FOOT[2]) or (self.board_bottom_point[0] > RIGHT_FOOT[0]):
                if  (self.board_bottom_point[0] > RIGHT_FOOT[0]):
                    self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                    self.theta       =  THETA_BIG*self.right_theta
                    self.translation = self.left_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
                    print("90度板在右,快左修")
                elif (self.board_bottom_point[0] < LEFT_FOOT[2]):
                    self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                    self.theta       =  THETA_BIG*self.left_theta
                    self.translation = self.right_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
                    print("90度板在左,快右修")
            else:
                self.forward        = 0
                self.translation    = 0
                self.theta          = 0
                print("🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷🚷")
        #正常距離
        else:
            #上板空間不足
            if  self.layer != 3 and (self.next_distance[0] < UP_BOARD_DISTANCE or self.next_distance[1] < UP_BOARD_DISTANCE or self.next_distance[2] < UP_BOARD_DISTANCE or self.next_distance[3] < UP_BOARD_DISTANCE or self.next_distance[4] < UP_BOARD_DISTANCE or self.next_distance[5] < UP_BOARD_DISTANCE):
                #左邊空間較大
                if (self.next_distance[0] + self.next_distance[1]) > (self.next_distance[4] + self.next_distance[5]):
                    self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                    self.theta       =  THETA_MIN * self.right_theta + THETA_CORRECTION
                    self.translation = self.left_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
                    print("空間不足,往左移")
                #右邊空間較大
                elif (self.next_distance[0] + self.next_distance[1]) > (self.next_distance[4] + self.next_distance[5]):
                    self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                    self.theta       =  THETA_MIN * self.left_theta + THETA_CORRECTION
                    self.translation = self.right_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
                    print("空間不足,往右移")
            #90度板位在中間
            # elif (self.board_bottom_point[0] > LEFT_FOOT[0]) and (self.board_bottom_point[0] < RIGHT_FOOT[2]):
            #     if  self.board_bottom_point[0] < ((LEFT_FOOT[2]+RIGHT_FOOT[0])/2):
            #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
            #         self.theta       =  THETA_BIG*self.right_theta + THETA_CORRECTION
            #         self.translation = self.left_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
            #         print("90板怎麼在中間,快左修")
            #     elif self.board_bottom_point[0] > ((LEFT_FOOT[2]+RIGHT_FOOT[0])/2):
            #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
            #         self.theta       =  THETA_BIG*self.left_theta + THETA_CORRECTION
            #         self.translation = self.right_theta*  TRANSLATION_MIN + TRANSLATION_CORRECTION
            #         print("90板怎麼在中間,快右修")
            #90度板位在雙腳前
            # elif (self.board_bottom_point[0] < LEFT_FOOT[2]) or (self.board_bottom_point[0] > RIGHT_FOOT[0]):
            #     if  (self.board_bottom_point[0] > RIGHT_FOOT[0]):
            #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
            #         self.theta       =  THETA_BIG*self.right_theta + THETA_CORRECTION
            #         self.translation = self.left_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
            #         print("90度板在右,快左修")
            #     elif (self.board_bottom_point[0] < LEFT_FOOT[2]):
            #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
            #         self.theta       =  THETA_BIG*self.left_theta + THETA_CORRECTION
            #         self.translation = self.right_theta * TRANSLATION_MIN + TRANSLATION_CORRECTION
            #         print("90度板在左,快右修")    
            elif self.layer > 1 and self.distance[0] > 240 and self.distance[5] > 240:
                print("前方沒有要上的板子")
                self.no_up_board(state)
            else:
                self.translation = TRANSLATION_CORRECTION 
                if self.distance[0] < FIRST_FORWORD_CHANGE_LINE and self.distance[5] < FIRST_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_MIN + FORWARD_CORRECTION
                    self.theta_change(state)
                    print('小前進')
                elif self.distance[0] < SECOND_FORWORD_CHANGE_LINE and self.distance[5] < SECOND_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.theta_change(state)
                    print('前進')
                elif self.distance[0] < THIRD_FORWORD_CHANGE_LINE and self.distance[5] < THIRD_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                    self.theta_change(state)
                    print('大前進')
                else:
                    self.theta = THETA_CORRECTION
                    if state > 1:
                        self.forward     = FORWARD_SUPER + FORWARD_CORRECTION
                        print('超大前進') 
                    else:
                        self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                        print('大前進')

    def theta_change(self,state):
        slope_long  = self.distance[0]-self.distance[5]
        slope_short = self.distance[3]-self.distance[4]
        if   (slope_long<0):
            self.decide_theta = self.left_theta
            print('左旋')
        elif (slope_long>0):
            self.decide_theta = self.right_theta
            print('右旋')
        else:
            print('直走')

        if  (abs(slope_long))>self.slope_big:                    #斜率過大,角度給最大
            self.theta =  THETA_BIG*self.decide_theta + THETA_CORRECTION
            self.translation = TRANSLATION_NORMAL*self.decide_theta*-1
            print("大旋")
        elif(abs(slope_long))>self.slope_normal:                 #斜率較大,修正值較大
            self.theta = THETA_NORMAL*self.decide_theta + THETA_CORRECTION
            print("旋")
        elif(abs(slope_long))>self.slope_min:                    #斜率較小,修正值較小
            self.theta = THETA_MIN*self.decide_theta + THETA_CORRECTION
            print("小旋")
        else:
            self.theta = 0+THETA_CORRECTION

    def speed_limit(self): 
        #避免修正過大
        if (self.last_translation > self.translation and self.translation < 0) or self.last_translation < self.translation and self.translation > 0:
            self.translation = 0
        elif (self.last_theta > self.theta and self.theta < 0) or self.last_theta < self.theta and self.theta > 0:
            self.theta = 0
        #速度限制
        if self.forward > 2000:
            self.forward = 2000
        elif self.forward < -2000:
            self.forward = -2000
        #平移限制
        if self.translation > 1000:
            self.translation = 1000
        elif self.translation < -1000:
            self.translation = -1000
        #角度限制
        if self.theta > 5:
            self.theta = 5
        elif self.theta < -5:
            self.theta = -5

    # def check_board(self,state):
    #     send.sendHeadMotor(2,self.head_Vertical-100,100)#垂直
    #     print("start check board")
    #     cnt = send.color_mask_subject_cnts[self.layer_model[self.layer]]
    #     self.board_in_right  = False
    #     self.board_in_left   = False 
    #     no_find_board = False
    #     if cnt != 0:
           
    #         for i in range(cnt):
    #             if send.color_mask_subject_size[self.layer_model[state]][i] > 300:
    #                 x = send.color_mask_subject_X[self.layer_model[state]][i]
    #                 print(no_find_board)
    #                 time.sleep(2)
    #                 if x < 100:
    #                     self.theta =  THETA_MIN*self.left_theta + THETA_CORRECTION
    #                     break
    #                 elif x > 220:
    #                     self.theta =  THETA_MIN*self.right_theta + THETA_CORRECTION
    #                     break
    #                 else:
    #                     self.theta =  THETA_CORRECTION
    #                     break
    #             else:
    #                 self.theta =  THETA_CORRECTION
    #                 no_find_board = True
    #                 print(no_find_board)
    #                 time.sleep(2)
    #                 break
    #         if no_find_board == True:
    #             print('find board')
    #             for right in range(HEAD_HORIZONTAL,HEAD_HORIZONTAL-512,-2):
    #                 send.sendHeadMotor(1,right,50)#水平
    #                 time.sleep(0.05)
    #                 if max(send.color_mask_subject_size[self.layer_model[state]]) > 300: 
    #                     self.board_in_right  = True
    #                     break
    #             send.sendHeadMotor(1,self.head_Horizontal,70)#水平
    #             time.sleep(1.5)
    #             for left in range(HEAD_HORIZONTAL,HEAD_HORIZONTAL+512,2):
    #                 send.sendHeadMotor(1,left,50)#水平
    #                 time.sleep(0.05)
    #                 if max(send.color_mask_subject_size[self.layer_model[state]]) > 300: 
    #                     self.board_in_left  = True
    #                     break
    #             #-------#
    #             if self.board_in_right == True:
    #                 if right > HEAD_HORIZONTAL-128:
    #                     self.theta =  THETA_NORMAL*self.left_theta + THETA_CORRECTION
    #                 else:
    #                     self.theta =  THETA_BIG*self.left_theta + THETA_CORRECTION
    #     else:
    #         #轉頭找板
    #         print('find board')
    #         for right in range(HEAD_HORIZONTAL,HEAD_HORIZONTAL-512,-2):
    #             send.sendHeadMotor(1,right,50)#水平
    #             time.sleep(0.05)
    #             if max(send.color_mask_subject_size[self.layer_model[state]]) > 300: 
    #                 self.board_in_right  = True
    #                 break
    #         send.sendHeadMotor(1,self.head_Horizontal,70)#水平
    #         time.sleep(1.5)
    #         for left in range(HEAD_HORIZONTAL,HEAD_HORIZONTAL+512,2):
    #             send.sendHeadMotor(1,left,50)#水平
    #             time.sleep(0.05)
    #             if max(send.color_mask_subject_size[self.layer_model[state]]) > 300: 
    #                 self.board_in_left  = True
    #                 break
    #         #-------#
    #         if self.board_in_right == True:
    #             if right > HEAD_HORIZONTAL-128:
    #                 self.theta =  THETA_NORMAL*self.left_theta + THETA_CORRECTION
    #             else:
    #                 self.theta =  THETA_BIG*self.left_theta + THETA_CORRECTION
    #     send.sendHeadMotor(1,self.head_Horizontal,100)#水平

            
    def checkout_board(self): 
        self.find_board(self.layer)
        if (self.distance[0] >250 and self.distance[1] >250 and self.distance[2] >250 and self.distance[3] >250) or (self.distance[0] >250 and self.distance[1] >250 and self.distance[2] >250 and self.distance[3] >250):
            self.find_board_right = False
            self.find_board_left  = False
            board_right           = 0
            board_left            = 0
            send.sendHeadMotor(2,self.head_Vertical-100,100)
            for i in range(1800,1000,-10):
                send.sendHeadMotor(1,i,100)
                board_right= send.color_mask_subject_cnts[self.layer_model[self.layer]]
                print(self.layer)
                if board_right!=0 :
                    for i in range(board_right):                    
                        board_size = send.color_mask_subject_size[self.layer_model[self.layer]][i]
                        if board_size > 2500 and send.color_mask_subject_XMax[self.layer_model[self.layer]][i]>160:
                            self.find_board_right = True
                            print("board in right",self.find_board_left)
                            time.sleep(0.5)
                            break
                        else:
                            pass
                if self.find_board_right:
                    print(i)
                    if i < 1200:
                        self.theta = THETA_BIG*self.right_theta
                    elif i <1500:
                        self.theta = THETA_NORMAL*self.right_theta
                    else:
                        self.theta = THETA_MIN*self.right_theta
                    break
                time.sleep(0.05)
            time.sleep(2)
            send.sendHeadMotor(1,self.head_Horizontal,100)#水平
            time.sleep(1)
            if self.find_board_right == False:
                for k in range(2200,3000,10):
                    send.sendHeadMotor(1,k,100)
                    board_left = send.color_mask_subject_cnts[self.layer_model[self.layer]]
                    if board_left!=0 :
                        for i in range(board_left):                    
                            board_size = send.color_mask_subject_size[self.layer_model[self.layer]][i]
                            if board_size > 2500 and send.color_mask_subject_XMin[self.layer_model[self.layer]][i]<160:
                                self.find_board_left = True
                                print("board in left",self.find_board_left)
                                time.sleep(0.5)
                                break
                            else:
                                pass
                    if self.find_board_left:
                        print(k)
                        if k > 2650:
                            self.theta = THETA_BIG*self.left_theta
                        elif k >2350:
                            self.theta = THETA_NORMAL*self.left_theta
                        else:
                            self.theta = THETA_MIN*self.left_theta
                        break
                    time.sleep(0.05)
            time.sleep(2)
            send.sendHeadMotor(1,self.head_Horizontal,100)#水平
            send.sendHeadMotor(2,self.head_Vertical,100)#垂直

    def no_up_board(self,state):
        last_board = send.color_mask_subject_cnts[self.layer_model[state-1]]
        if last_board !=0:
            self.forward = BACK_NORMAL
            self.theta = self.theta
            self.translation = self.translation
        else:
            pass

        
    def draw_function(self):
        send.drawImageFunction(1,0,0,320,FOOTBOARD_LINE,FOOTBOARD_LINE,0,128,255)#膝蓋的橫線
        send.drawImageFunction(2,0,LEFT_FOOT[0],LEFT_FOOT[0],0,240,255,128,128)#lr的線
        send.drawImageFunction(3,0,LEFT_FOOT[1],LEFT_FOOT[1],0,240,255,128,128)#lm的線
        send.drawImageFunction(4,0,LEFT_FOOT[2],LEFT_FOOT[2],0,240,255,128,128)#ll的線
        send.drawImageFunction(5,0,RIGHT_FOOT[0],RIGHT_FOOT[0],0,240,255,128,128)#rl的線
        send.drawImageFunction(6,0,RIGHT_FOOT[1],RIGHT_FOOT[1],0,240,255,128,128)#rm的線
        send.drawImageFunction(7,0,RIGHT_FOOT[2],RIGHT_FOOT[2],0,240,255,128,128)#rr的線
        #邊緣線
        send.drawImageFunction(8,1,LEFT_FOOT[0]-5,LEFT_FOOT[0]+5,FOOTBOARD_LINE-self.distance[0]-5,FOOTBOARD_LINE-self.distance[0]+5,255,0,128)
        send.drawImageFunction(9,1,LEFT_FOOT[1]-5,LEFT_FOOT[1]+5,FOOTBOARD_LINE-self.distance[1]-5,FOOTBOARD_LINE-self.distance[1]+5,255,0,128)
        send.drawImageFunction(10,1,LEFT_FOOT[2]-5,LEFT_FOOT[2]+5,FOOTBOARD_LINE-self.distance[2]-5,FOOTBOARD_LINE-self.distance[2]+5,255,0,128)
        send.drawImageFunction(11,1,RIGHT_FOOT[0]-5,RIGHT_FOOT[0]+5,FOOTBOARD_LINE-self.distance[3]-5,FOOTBOARD_LINE-self.distance[3]+5,255,0,128)
        send.drawImageFunction(12,1,RIGHT_FOOT[1]-5,RIGHT_FOOT[1]+5,FOOTBOARD_LINE-self.distance[4]-5,FOOTBOARD_LINE-self.distance[4]+5,255,0,128)
        send.drawImageFunction(13,1,RIGHT_FOOT[2]-5,RIGHT_FOOT[2]+5,FOOTBOARD_LINE-self.distance[5]-5,FOOTBOARD_LINE-self.distance[5]+5,255,0,128)
        
    def return_real_board(self,y,x,layer):
        real_distance_flag=0
        real_distance_flag= (send.Label_Model[320*y+x] == self.layer_parameter[layer])
        if real_distance_flag==1:
            for i in range(1,11):
                real_distance_flag=real_distance_flag and send.Label_Model[320*(y-i)+x] == self.layer_parameter[layer]
                if real_distance_flag==0:
                    break
        return real_distance_flag

    def printf(self):
        print('_______________________________________')
        print('x:'    ,self.forward)
        print('y:'    ,self.translation)
        print('theta:',self.theta)
        print('💢💢💢💢💢💢💢💢💢💢💢💢💢💢💢💢')
        print("層數"    ,self.layer)
        print("距離板:" ,self.distance)
        print("上板空間",self.next_distance)
        print("板最左點",self.board_left_point)
        print("板最右點",self.board_right_point)
        print("板最上點",self.board_top_point)
        print("板最下點",self.board_bottom_point)
        print("板大小"  ,self.board_size)
        print('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')