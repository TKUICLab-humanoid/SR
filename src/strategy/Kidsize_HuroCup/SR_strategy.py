#!/usr/bin/env python
#coding=utf-8
import sys
import rospy
import numpy as np
import math
import time
from Python_API import Sendmessage

send = Sendmessage()

#--校正量--#
#校正變數
#前進量校正
FORWARD_CORRECTION         = 0
#平移校正
TRANSLATION_CORRECTION     = 0
#旋轉校正
THETA_CORRECTION           = 0
#上下板前進量
LCUP                       = 16000                 #上板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = 3.2
LCDOWN                     = 20000                 #下板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = -1.5
#---微調站姿開關---#
STAND_CORRECT_LC           = True                  #sector(30) LC_stand微調站姿
STAND_CORRECT_CW           = False                 #sector(33) CW_stand微調站姿
UPBOARD_CORRECT            = True                  #sector(31) 上板微調站姿
DOWNBOARD_CORRECT          = True                  #sector(32) 下板微調站姿
DRAW_FUNCTION_FLAG         = True                  #影像繪圖開關
START_LAYER                = 1
BOARD_COLOR                = ["Green"  ,           #板子顏色(根據比賽現場調整)
                              "Red"    ,
                              "Blue"   , 
                              "Yellow" , 
                              "Blue"   , 
                              "Red"    , 
                              "Green"]
LADDER_COLOAR              = "Red"                     
#----------#
FOOTBOARD_LINE             = 220                   #上板基準線
#----------#                 [ 左, 中, 右]
LEFT_FOOT                  = [115,134,153]
RIGHT_FOOT                 = [176,194,213]
HEAD_HORIZONTAL            = 2055                  #頭水平
HEAD_VERTICAL              = 2705                  #頭垂直 #down 2750
#----------#
WARNING_DISTANCE           = 4                     #危險距離
GO_UP_DISTANCE             = 10                    #上板距離
DISTANCE_DISPARITY         = 5                     #距離點差距
FIRST_FORWORD_CHANGE_LINE  = 50                    #小前進判斷線
SECOND_FORWORD_CHANGE_LINE = 90                    #前進判斷線
THIRD_FORWORD_CHANGE_LINE  = 150                   #大前進判斷線
ANGLE_REVISION_DISTANCE    = 100                   #角度修正距離
UP_BOARD_DISTANCE          = 60                    #最低上板需求距離
#----------#
BACK_MIN                   = -500                  #小退後
BACK_NORMAL                = -1000                 #退後
FORWARD_MIN                = 1000                  #小前進
FORWARD_NORMAL             = 2000                  #前進
FORWARD_BIG                = 3000                  #大前進
FORWARD_SUPER              = 5000                  #超大前進
#----------#
TRANSLATION_MIN            = 500                   #小平移
TRANSLATION_NORMAL         = 1000                  #平移
TRANSLATION_BIG            = 1500                  #大平移
#----------#
THETA_MIN                  = 1                     #小旋轉
THETA_NORMAL               = 3                     #旋轉
THETA_BIG                  = 5                     #大旋轉
SLOPE_MIN                  = 5                     #有點斜
SLOPE_NORMAL               = 8                     #斜
SLOPE_BIG                  = 15                    #過斜
#左基礎參數
LEFT_THETA                 = 1
#右基礎參數
RIGHT_THETA                = -1
#前進基礎參數
FORWARD_PARAM              = 1
#後退基礎參數
BACK_PARAM                 = -1
#----------#
BASE_CHANGE                = 100                   #基礎變化量(前進&平移)
 
class LiftandCarry:
#上下板
    def __init__(self):
        self.init()
        self.start      = time.time()
        
    def main(self):
        send.sendHeadMotor(1,self.head_Horizontal,100)#水平
        send.sendHeadMotor(2,self.head_Vertical,100)#垂直
        if DRAW_FUNCTION_FLAG:
            self.draw_function()
        self.end = time.time()
        if (self.end-self.start >0.5):
            self.printf()
            self.start = time.time()
        if stategy == "Lift_and_Carry_off":
        #初始化設定
            if not self.walkinggait_stop:
                rospy.loginfo("🔊LC parameter reset")
                send.sendHeadMotor(1,self.head_Horizontal,100)  #水平
                send.sendHeadMotor(2,self.head_Vertical,100)    #垂直
                send.sendBodyAuto(0,0,0,0,1,0)
                send.sendSensorReset()              #IMUreset
                time.sleep(2)
                self.init()
                send.sendBodySector(29)             #基礎站姿磁區
                time.sleep(1.5)
                if STAND_CORRECT_LC:
                    send.sendBodySector(30)             #基礎站姿調整磁區
                time.sleep(1)
                rospy.loginfo("reset🆗🆗🆗")
            rospy.loginfo("LC turn off")
        
            time.sleep(1)
        else:
        #策略
            if self.layer ==7:
                pass
            elif self.walkinggait_stop and self.first_in:
                sys.stdout.write("\033[H")
                sys.stdout.write("\033[J")
                send.sendBodyAuto(self.forward,0,0,0,1,0)
                self.walkinggait_stop = False
                self.first_in         = False
            elif self.walkinggait_stop and not self.first_in:
                send.sendBodyAuto(0,0,0,0,1,0)
                self.walkinggait_stop = False
            elif not self.walkinggait_stop:
                self.find_board()
                self.walkinggait(self.edge_judge())

    def init(self):
        #狀態
        self.state                 ='停止'
        #轉頭找板旗標
        self.find_board_in_right   = False
        self.find_board_in_left    = False
        #步態啟動旗標
        self.walkinggait_stop      = True
        self.first_in              = True  
        #上板延遲
        self.upboard_start         = 0          #初始化開始時間
        self.upboard_end           = -999999    #初始化結束時間
        self.updelay               = 1
        #層數       
        self.layer                 = START_LAYER
        #設定頭部馬達
        self.head_Horizontal       = HEAD_HORIZONTAL
        self.head_Vertical         = HEAD_VERTICAL
        #距離矩陣                     [左左,左中,左右 ,右左,右中,右右 ]
        self.distance              = [9999,9999,9999,9999,9999,9999]
        self.next_distance         = [9999,9999,9999,9999,9999,9999]
        #危險板資訊                   [左       右]
        self.danger_board          = [False,False]
        #板子資訊矩陣                  [X   ,   Y] 
        self.board_left_point      = [9999,9999]
        self.board_right_point     = [9999,9999]
        self.board_top_point       = [9999,9999]
        self.board_bottom_point    = [9999,9999]
        #上板基準線
        self.footboard_line        = FOOTBOARD_LINE
        #步態參數
        self.forward               = 2000 + FORWARD_CORRECTION
        self.translation           = 0    + TRANSLATION_CORRECTION
        self.theta                 = 0    + THETA_CORRECTION
        self.now_forward           = 0 
        self.now_translation       = 0
        self.now_theta             = 0  
        #左右決定
        self.decide_theta          = 0
        #建立板子資訊
        self.next_board            = ObjectInfo(BOARD_COLOR[self.layer+1],'Board')
        self.now_board             = ObjectInfo(BOARD_COLOR[self.layer],'Board')   #設定當前尋找的板子
        self.last_board            = ObjectInfo(BOARD_COLOR[self.layer-1],'Board') #設定前一個板子
        # self.next_board.update()
        # self.now_board.update()
        # self.last_board.update()

    def find_board(self):
    #獲取板子資訊、距離資訊
        self.next_board.update()
        self.now_board.update()
        self.last_board.update()
        self.distance              = [9999,9999,9999,9999,9999,9999]
        self.next_distance         = [9999,9999,9999,9999,9999,9999]
        #-------獲取板子資訊------#
        if self.now_board.get_target == True:
            self.board_left_point[0]    = self.now_board.edge_min.x
            self.board_right_point[0]   = self.now_board.edge_max.x
            self.board_top_point[1]     = self.now_board.edge_min.y
            self.board_bottom_point[1]  = self.now_board.edge_max.y
            #left_point
            for i in range (self.board_bottom_point[1],self.board_top_point[1],-1):
                if send.Label_Model[320*i+self.board_left_point[0]]  == self.now_board.color_parameter:
                    # if not self.nearest_search(self.board_left_point[0],i):
                    #     continue
                    # else:
                    self.board_left_point[1]    = i
                    break
            #right_point
            for i in range (self.board_bottom_point[1],self.board_top_point[1],-1):
                if send.Label_Model[320*i+self.board_right_point[0]]  == self.now_board.color_parameter:
                    # if not self.nearest_search(self.board_right_point[0],i):
                    #     continue
                    # else:
                    self.board_right_point[1]   = i
                    break
            #top_point
            for i in range (self.board_right_point[0],self.now_board.edge_min.x,-1):
                if send.Label_Model[320*self.board_top_point[1]+i]  == self.now_board.color_parameter:
                    # if not self.nearest_search(i,self.board_top_point[1]):
                    #     continue
                    # else:
                    self.board_top_point[0]     = i
                    break
            #bottom_point
            for i in range (self.board_right_point[0],self.board_left_point[0]):
                if send.Label_Model[320*self.board_bottom_point[1]+i]  == self.now_board.color_parameter:
                    # if not self.nearest_search(i,self.board_bottom_point[1]):
                    #     continue
                    # else:
                    self.board_bottom_point[0]  = i
                    break
        else:
        #初始化
            self.board_left_point    = [9999,9999]
            self.board_right_point   = [9999,9999]
            self.board_top_point     = [9999,9999]
            self.board_bottom_point  = [9999,9999]
        #-------距離判斷-------#
        #危險區域判斷
        count = 0
        for i in range (220,150,-1):
            if send.Label_Model[320*i+LEFT_FOOT[0]-10]  == self.last_board.color_parameter:
                count = count +1
            if count > 45:
                self.danger_board[0] = True
                count = 0
                break
            else:
                count = 0
        for i in range (220,150,-1):
            if send.Label_Model[320*i+RIGHT_FOOT[2]+10]  == self.last_board.color_parameter:
                count = count +1
            if count > 45:
                self.danger_board[1] = True
                count = 0
                break
            else:
                count = 0
        #----Leftfoot----# 
        for LL in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(LL,LEFT_FOOT[0],self.now_board.color_parameter):
                    self.distance[0] = self.footboard_line - LL
                    break
        for LM in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(LM,LEFT_FOOT[1],self.now_board.color_parameter):
                    self.distance[1] = self.footboard_line - LM
                    break
        for LR in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(LR,LEFT_FOOT[2],self.now_board.color_parameter):
                    self.distance[2] = self.footboard_line - LR
                    break
        #----Rightfoot----#
        for RL in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(RL,RIGHT_FOOT[0],self.now_board.color_parameter):
                    self.distance[3] = self.footboard_line - RL
                    break
        for RM in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(RM,RIGHT_FOOT[1],self.now_board.color_parameter):
                    self.distance[4] = self.footboard_line - RM
                    break
        for RR in range(FOOTBOARD_LINE,10,-1):
                if self.return_real_board(RR,RIGHT_FOOT[2],self.now_board.color_parameter):
                    self.distance[5] = self.footboard_line - RR
                    break
        #-----------------#
        if self.layer == 6 or self.layer == 3:
        #要下最後一層,不用偵測下板空間
            pass
        else:
        #除了上最頂層以外,偵測上板空間
            #----Leftfoot_next----# 
            for LL_2 in range(LL,10,-1):
                    if self.return_real_board(LL_2,LEFT_FOOT[0],self.next_board.color_parameter):
                        self.next_distance[0] = LL - LL_2
                        break
            for LM_2 in range(LM,10,-1):
                    if self.return_real_board(LM_2,LEFT_FOOT[1],self.next_board.color_parameter):
                        self.next_distance[1] = LM - LM_2
                        break
            for LR_2 in range(LR,10,-1):
                    if self.return_real_board(LR_2,LEFT_FOOT[2],self.next_board.color_parameter):
                        self.next_distance[2] = LR - LR_2
                        break
            #----Rightfoot_next----#
            for RL_2 in range(RR,10,-1):
                    if self.return_real_board(RL_2,RIGHT_FOOT[0],self.next_board.color_parameter):
                        self.next_distance[3] = RL - RL_2
                        break
            for RM_2 in range(RM,10,-1):
                    if self.return_real_board(RM_2,RIGHT_FOOT[1],self.next_board.color_parameter):
                        self.next_distance[4] = RM - RM_2
                        break
            for RR_2 in range(RR,10,-1):
                    if self.return_real_board(RR_2,RIGHT_FOOT[2],self.next_board.color_parameter):
                        self.next_distance[5] = RR - RR_2
                        break

    def walkinggait(self,ready_to_lc):
    #步態函數,用於切換countiue 或 LC 步態
        if ready_to_lc:
            rospy.loginfo("對正板子")
            #--延遲上板--#
            # self.upboard_start=time.time()
            # self.upboard_end = -999999
            # while(self.upboard_end-self.upboard_start) < self.updelay:
            #     self.upboard_end=time.time()
            #     send.sendContinuousValue(-500,self.translation,0,self.theta,0)
            #     print("=======")
            #     print("∥delay:",self.upboard_end-self.upboard_start,'∥')
            #     print("=======")
            self.forward        = 0
            self.translation    = 0
            self.theta          = 0
            send.sendBodyAuto(0,0,0,0,1,0)           #停止步態
            time.sleep(3)
            if self.layer == 3:
                self.footboard_line = FOOTBOARD_LINE
                self.updelay = 0.5
            else:
                self.updelay = 1
            send.sendSensorReset()                   #IMU reset 避免機器人步態修正錯誤
            self.distance              = [9999,9999,9999,9999,9999,9999]
            self.next_distance         = [9999,9999,9999,9999,9999,9999]
            self.walkinggait_stop      = True
            send.sendBodySector(29)                  #這是基本站姿的磁區
            time.sleep(1)
            if self.layer < 4:
                if UPBOARD_CORRECT   == True:
                    rospy.loginfo("準備上板")
                    send.sendBodySector(31)          #上板前站姿調整
                    time.sleep(3)                    #微調站姿延遲
                send.sendBodyAuto(LCUP,0,0,0,2,0)    #上板步態
            else:
                if DOWNBOARD_CORRECT == True:
                    rospy.loginfo("準備下板")
                    send.sendBodySector(32)          #下板前站姿調整
                    time.sleep(3)                    #微調站姿延遲
                send.sendBodyAuto(LCDOWN,0,0,0,3,0)  #下板步態
            self.layer += 1                          #層數加一
            time.sleep(5)
            send.sendBodySector(29)                  #這是基本站姿的磁區
            time.sleep(1)
            if STAND_CORRECT_LC == True:
                send.sendBodySector(30)              #基礎站姿調整
            time.sleep(1)
            if self.layer == 7:
                pass
            else:
                self.next_board = ObjectInfo(BOARD_COLOR[self.layer+1],'Board') #設定下一個尋找的板子
                self.now_board  = ObjectInfo(BOARD_COLOR[self.layer],'Board')   #設定當前尋找的板子
                self.last_board = ObjectInfo(BOARD_COLOR[self.layer-1],'Board') #設定前一個板子
                self.checkout_board()
            time.sleep(1)
        else:
            #前進變化量
            if self.now_forward > self.forward:
                self.now_forward -= BASE_CHANGE
            elif self.now_forward < self.forward:
                self.now_forward += BASE_CHANGE
            else:
                self.now_forward = self.forward
            #平移變化量
            if self.now_translation > self.translation:
                self.now_translation -= BASE_CHANGE
            elif self.now_translation < self.translation:
                self.now_translation += BASE_CHANGE
            else:
                self.now_translation = self.translation
            #旋轉變化量
            if self.now_theta > self.theta:
                self.now_theta -= 1
            elif self.now_theta < self.theta:
                self.now_theta += 1
            else:
                self.now_theta = self.theta
            #速度調整
            send.sendContinuousValue(self.now_forward,self.now_translation,0,self.now_theta,0)

    def edge_judge(self):
        if ((self.distance[0] < GO_UP_DISTANCE+10) and (self.distance[1] < GO_UP_DISTANCE+8) and\
           (self.distance[2] < GO_UP_DISTANCE+5) and (self.distance[3] < GO_UP_DISTANCE+3) and\
           (self.distance[4] < GO_UP_DISTANCE) and (self.distance[5] < GO_UP_DISTANCE)):
           #上板
           return True
        else:
            if (self.distance[0] <= WARNING_DISTANCE) or (self.distance[1] <= WARNING_DISTANCE) or (self.distance[2] <= WARNING_DISTANCE) or (self.distance[3] <= WARNING_DISTANCE) or (self.distance[4] <= WARNING_DISTANCE) or (self.distance[5] <= WARNING_DISTANCE): 
            #即將踩板
                #90度板位在中間
                # if (self.board_bottom_point[0] > LEFT_FOOT[2]) and (self.board_bottom_point[0] < RIGHT_FOOT[0]) and abs(self.distance[0] - self.distance[2]) < 15 and abs(self.distance[3] - self.distance[5]) < 15:
                #     if  self.board_bottom_point[0] > ((LEFT_FOOT[2]+RIGHT_FOOT[0])/2):
                #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                #         self.theta       = THETA_BIG*RIGHT_THETA
                #         self.translation = LEFT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                #         self.state = "90板怎麼在中間,快左修"
                #     elif self.board_bottom_point[0] < ((LEFT_FOOT[2]+RIGHT_FOOT[0])/2):
                #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                #         self.theta       = THETA_BIG*LEFT_THETA
                #         self.translation = RIGHT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                #         self.state = "90板怎麼在中間,快右修"
                # #90度板位在雙腳前
                # elif (self.board_bottom_point[0] < LEFT_FOOT[2]) or (self.board_bottom_point[0] > RIGHT_FOOT[0]):
                #     if  (self.board_bottom_point[0] > RIGHT_FOOT[0]):
                #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                #         self.theta       = THETA_BIG*RIGHT_THETA
                #         self.translation = LEFT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                #         self.state = "90度板在右,快左修"
                #     elif (self.board_bottom_point[0] < LEFT_FOOT[2]):
                #         self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                #         self.theta       = THETA_BIG*LEFT_THETA
                #         self.translation = RIGHT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                #         self.state = "90度板在左,快右修"
                # else:
                self.forward = BACK_MIN + FORWARD_CORRECTION
                self.theta_change()
                self.state = "!!!小心踩板,後退!!!"
            else:
                if  self.layer != 3 and (self.next_distance[0] < UP_BOARD_DISTANCE or self.next_distance[1] < UP_BOARD_DISTANCE or self.next_distance[2] < UP_BOARD_DISTANCE or self.next_distance[3] < UP_BOARD_DISTANCE or self.next_distance[4] < UP_BOARD_DISTANCE or self.next_distance[5] < UP_BOARD_DISTANCE):
                    #左邊空間較大
                    if (self.next_distance[0] + self.next_distance[1]) > (self.next_distance[4] + self.next_distance[5]):
                        self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                        self.theta       =  0 #THETA_MIN * RIGHT_THETA + THETA_CORRECTION
                        self.translation = LEFT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                        self.state = "空間不足,往左移"
                    #右邊空間較大
                    elif (self.next_distance[0] + self.next_distance[1]) < (self.next_distance[4] + self.next_distance[5]):
                        self.forward     = BACK_NORMAL + FORWARD_CORRECTION
                        self.theta       =  0 #THETA_MIN * LEFT_THETA + THETA_CORRECTION
                        self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                        self.state = "空間不足,往右移"
                elif self.layer > 1 and self.distance[0] > 240  and self.distance[1] > 240 and self.distance[4] > 240 and self.distance[5] > 240:
                    self.state = "前方沒有要上的板子"
                    self.no_up_board()
                else:
                    if self.layer == 1:
                        self.translation = TRANSLATION_CORRECTION 
                        if self.distance[0] < FIRST_FORWORD_CHANGE_LINE or self.distance[1] < FIRST_FORWORD_CHANGE_LINE or self.distance[2] < FIRST_FORWORD_CHANGE_LINE or self.distance[3] < FIRST_FORWORD_CHANGE_LINE or self.distance[4] < FIRST_FORWORD_CHANGE_LINE or self.distance[5] < FIRST_FORWORD_CHANGE_LINE:
                            self.forward     = FORWARD_MIN + FORWARD_CORRECTION
                            self.theta_change()
                            self.state = '小前進'
                        elif self.distance[0] < SECOND_FORWORD_CHANGE_LINE or self.distance[1] < SECOND_FORWORD_CHANGE_LINE or self.distance[2] < SECOND_FORWORD_CHANGE_LINE or self.distance[3] < SECOND_FORWORD_CHANGE_LINE or self.distance[4] < SECOND_FORWORD_CHANGE_LINE or self.distance[5] < SECOND_FORWORD_CHANGE_LINE:
                            self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                            self.theta_change()
                            self.state = '前進'
                        elif self.distance[0] < THIRD_FORWORD_CHANGE_LINE or self.distance[5] < THIRD_FORWORD_CHANGE_LINE:
                            self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                            self.theta_change()
                            self.state = '大前進'
                        else:
                            self.theta = 0+THETA_CORRECTION
                            if self.layer == 1:
                                self.forward     = FORWARD_SUPER + FORWARD_CORRECTION
                                self.state = '超大前進' 
                            else:
                                self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                                self.state = '大前進'
                    else:
                        self.translation = 0+TRANSLATION_CORRECTION 
                        if self.distance[0] < FIRST_FORWORD_CHANGE_LINE or self.distance[1] < FIRST_FORWORD_CHANGE_LINE or self.distance[2] < FIRST_FORWORD_CHANGE_LINE or self.distance[3] < FIRST_FORWORD_CHANGE_LINE or self.distance[4] < FIRST_FORWORD_CHANGE_LINE or self.distance[5] < FIRST_FORWORD_CHANGE_LINE:
                            self.forward     = FORWARD_MIN + FORWARD_CORRECTION
                            self.theta_change()
                            self.state = '小前進'
                        elif self.distance[0] < SECOND_FORWORD_CHANGE_LINE or self.distance[1] < SECOND_FORWORD_CHANGE_LINE or self.distance[2] < SECOND_FORWORD_CHANGE_LINE or self.distance[3] < SECOND_FORWORD_CHANGE_LINE or self.distance[4] < SECOND_FORWORD_CHANGE_LINE or self.distance[5] < SECOND_FORWORD_CHANGE_LINE:
                            self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                            self.theta_change()
                            self.state = '前進'
                        elif self.distance[0] < THIRD_FORWORD_CHANGE_LINE or self.distance[5] < THIRD_FORWORD_CHANGE_LINE:
                            self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                            self.theta_change()
                            self.state = '大前進'
                        else:
                            self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                            self.theta       = 0 + THETA_CORRECTION
                            self.state = '大前進'
            return False

    def theta_change(self):
    #旋轉修正
        if self.distance[2] < 240 and self.distance[3] < 240:
            slope = self.distance[2] - self.distance[3]             #計算斜率(使用LR-RL)
        else:
            slope = 0

        if self.now_board.edge_min.x > self.distance[1] and slope > 5:
            self.theta = THETA_NORMAL*RIGHT_THETA + THETA_CORRECTION
            self.state = "板子太右,右旋"
        elif self.now_board.edge_max.x < self.distance[4] and slope < -5:
            self.theta = THETA_NORMAL*LEFT_THETA + THETA_CORRECTION
            self.state = "板子太左,左旋"
        else:
            #---決定左或右轉---#
            if   (slope < -1*(SLOPE_MIN)):
                self.decide_theta = LEFT_THETA
                self.state = "左旋"
            elif (slope > SLOPE_MIN):
                self.decide_theta = RIGHT_THETA
                self.state = "右旋"
            else:
                self.state = "直走"
            #-----------------#
            if  (abs(slope)) > SLOPE_BIG:                    #斜率過大,角度給最大
                self.theta       =  THETA_BIG*self.decide_theta + THETA_CORRECTION
                self.translation = TRANSLATION_NORMAL*self.decide_theta*-1
            elif(abs(slope)) > SLOPE_NORMAL:                 #斜率較大,修正值較大
                self.theta       = THETA_NORMAL*self.decide_theta + THETA_CORRECTION
                self.translation = TRANSLATION_MIN*self.decide_theta*-1
            elif(abs(slope)) > SLOPE_MIN:                    #斜率較小,修正值較小
                self.theta       = THETA_MIN*self.decide_theta + THETA_CORRECTION
                self.translation = 0+THETA_CORRECTION
            else:
                self.translation = 0+TRANSLATION_CORRECTION
                self.theta       = 0+THETA_CORRECTION

    def no_up_board(self):
    #上板或下板後影像上無下一層板
        last_board = send.color_mask_subject_cnts[self.last_board.color]
        if last_board !=0:
            self.forward = FORWARD_CORRECTION
            self.theta = self.theta
            self.translation = self.translation
        else:
            pass

    def checkout_board(self):
    #上板完後尋找下一板
        self.find_board()
        if (self.distance[0] >250 and self.distance[1] >250 and self.distance[2] >250 and self.distance[3] >250) or (self.distance[0] >250 and self.distance[1] >250 and self.distance[2] >250 and self.distance[3] >250):
            self.find_board_in_right = False
            self.find_board_in_left  = False
            board_right           = 0
            board_left            = 0
            send.sendHeadMotor(2,self.head_Vertical-100,100)
            #找右邊
            for i in range(1800,1000,-10):
                send.sendHeadMotor(1,i,100)
                board_right= send.color_mask_subject_cnts[self.now_board.color]
                if board_right!=0 :
                    for i in range(board_right):                    
                        board_size = send.color_mask_subject_size[self.now_board.color][i]
                        if board_size > 2500 and send.color_mask_subject_XMax[self.now_board.color][i]>160:
                            self.find_board_in_right = True
                            rospy.loginfo(f"板子在右邊")
                            time.sleep(0.5)
                            break
                        else:
                            pass
                if self.find_board_in_right:
                    if i < 1200:
                        self.theta = THETA_BIG*RIGHT_THETA
                    elif i <1500:
                        self.theta = THETA_NORMAL*RIGHT_THETA
                    else:
                        self.theta = THETA_MIN*RIGHT_THETA
                    break
                time.sleep(0.05)
            time.sleep(2)
            send.sendHeadMotor(1,self.head_Horizontal,100)#水平
            time.sleep(1)
            if self.find_board_in_right == False:
            #找左邊
                for k in range(2200,3000,10):
                    send.sendHeadMotor(1,k,100)
                    board_left = send.color_mask_subject_cnts[self.now_board.color]
                    if board_left!=0 :
                        for i in range(board_left):                    
                            board_size = send.color_mask_subject_size[self.now_board.color][i]
                            if board_size > 2500 and send.color_mask_subject_XMin[self.now_board.color][i]<160:
                                self.find_board_in_left = True
                                rospy.loginfo("板子在左邊")
                                time.sleep(0.5)
                                break
                            else:
                                pass
                    if self.find_board_in_left:
                        if k > 2650:
                            self.theta = THETA_BIG*LEFT_THETA
                        elif k >2350:
                            self.theta = THETA_NORMAL*LEFT_THETA
                        else:
                            self.theta = THETA_MIN*LEFT_THETA
                        break
                    time.sleep(0.05)
            time.sleep(2)
            send.sendHeadMotor(1,self.head_Horizontal,100)#水平
            if self.layer <4:
                send.sendHeadMotor(2,self.head_Vertical,100)#垂直
            else:
                send.sendHeadMotor(2,self.head_Vertical+45,100)#垂直
                pass

    def draw_function(self):
    #畫面顯示繪畫資訊    
        #腳的距離判斷線
        send.drawImageFunction(1,0,0,320,self.footboard_line,FOOTBOARD_LINE,0,128,255)#膝蓋的橫線
        send.drawImageFunction(2,0,LEFT_FOOT[0],LEFT_FOOT[0],0,240,255,128,128)#lr的線
        send.drawImageFunction(3,0,LEFT_FOOT[1],LEFT_FOOT[1],0,240,255,128,128)#lm的線
        send.drawImageFunction(4,0,LEFT_FOOT[2],LEFT_FOOT[2],0,240,255,128,128)#ll的線
        send.drawImageFunction(5,0,RIGHT_FOOT[0],RIGHT_FOOT[0],0,240,255,128,128)#rl的線
        send.drawImageFunction(6,0,RIGHT_FOOT[1],RIGHT_FOOT[1],0,240,255,128,128)#rm的線
        send.drawImageFunction(7,0,RIGHT_FOOT[2],RIGHT_FOOT[2],0,240,255,128,128)#rr的線
        #邊緣點
        send.drawImageFunction(8,1,LEFT_FOOT[0]-5,LEFT_FOOT[0]+5,FOOTBOARD_LINE-self.distance[0]-5,FOOTBOARD_LINE-self.distance[0]+5,255,0,128)
        send.drawImageFunction(9,1,LEFT_FOOT[1]-5,LEFT_FOOT[1]+5,FOOTBOARD_LINE-self.distance[1]-5,FOOTBOARD_LINE-self.distance[1]+5,255,0,128)
        send.drawImageFunction(10,1,LEFT_FOOT[2]-5,LEFT_FOOT[2]+5,FOOTBOARD_LINE-self.distance[2]-5,FOOTBOARD_LINE-self.distance[2]+5,255,0,128)
        send.drawImageFunction(11,1,RIGHT_FOOT[0]-5,RIGHT_FOOT[0]+5,FOOTBOARD_LINE-self.distance[3]-5,FOOTBOARD_LINE-self.distance[3]+5,255,0,128)
        send.drawImageFunction(12,1,RIGHT_FOOT[1]-5,RIGHT_FOOT[1]+5,FOOTBOARD_LINE-self.distance[4]-5,FOOTBOARD_LINE-self.distance[4]+5,255,0,128)
        send.drawImageFunction(13,1,RIGHT_FOOT[2]-5,RIGHT_FOOT[2]+5,FOOTBOARD_LINE-self.distance[5]-5,FOOTBOARD_LINE-self.distance[5]+5,255,0,128)

    def return_real_board(self,y,x,board):
    #檢查回傳的物件是否為板子,確認連續10個點為同一色模
        real_distance_flag=0
        real_distance_flag= (send.Label_Model[320*y+x] == board)
        if real_distance_flag==1:
            for i in range(1,11):
                real_distance_flag=real_distance_flag and send.Label_Model[320*(y-i)+x] == board
                if real_distance_flag==0:
                    break
        return real_distance_flag

    def nearest_search(self,x,y):
    #確認點為物件的點
        count = 0
        for i in range (x-1,x+1):
            for j in range (y-1,y+1):
                if send.Label_Model[320*j+i]   == self.now_board.color_parameter:
                    count+=1
        if count<5:
            return False
        else:
            return True

    def printf(self):
        sys.stdout.write("\033[H")
        # sys.stdout.write("\033[K")
        sys.stdout.write("\033[J")
        rospy.loginfo('_______________________________________')
        rospy.loginfo(f'x: {self.now_forward} ,y: {self.now_translation} ,theta: {self.now_theta}')
        rospy.loginfo(f'Goal_x: {self.forward} ,Goal_y: {self.translation} ,Goal_theta: {self.theta}')
        rospy.loginfo(f"機器人狀態: {self.state}")
        rospy.loginfo(f"IMU: {send.imu_value_Yaw}")
        rospy.loginfo(f"層數: {self.layer}")
        rospy.loginfo(f"距離板: {self.distance}")
        rospy.loginfo(f"上板空間: {self.next_distance}")
        rospy.loginfo(f"板最左點: {self.board_left_point},板最右點: {self.board_right_point}")
        rospy.loginfo(f"板最上點: {self.board_top_point},板最下點: {self.board_bottom_point}")
        rospy.loginfo(f"板大小: {self.now_board.target_size}")
        rospy.loginfo('_______________________________________')
        

class WallClimb:
#爬梯
    def __init__(self):
        self.ladder = ObjectInfo(LADDER_COLOAR,'Board')
        self.init()

    def main(self):
        self.printf()

    def init(self):
        #步態參數
        self.forward               = 2000 + FORWARD_CORRECTION
        self.translation           = 0    + TRANSLATION_CORRECTION
        self.theta                 = 0    + THETA_CORRECTION
        self.now_forward           = 0 
        self.now_translation       = 0
        self.now_theta             = 0  

    def find_ladder(self):
        pass
    
    def walkinggait(self):
        pass

    def edge_judge(self):
        pass


    def draw_function(self):
        pass

    def printf(self):
        sys.stdout.write("\033[H")
        sys.stdout.write("\033[J")
        rospy.loginfo('_______________________________________')
        rospy.loginfo(f'x: {self.forward} ,y: {self.translation} ,theta: {self.theta}')
        rospy.loginfo('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')  

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
    color_dict = {  'Orange':  0,
                    'Yellow':  1,
                    'Blue'  :  2,
                    'Green' :  3,
                    'Black' :  4,
                    'Red'   :  5,
                    'White' :  6 }
    parameter  = {  'Orange':  2**0,
                    'Yellow':  2**1,
                    'Blue'  :  2**2,
                    'Green' :  2**3,
                    'Black' :  2**4,
                    'Red'   :  2**5,
                    'White' :  2**6 }

    def __init__(self, color, object_type):
        self.color            = self.color_dict[color]
        self.color_parameter  = self.parameter[color]
        self.edge_max         = Coordinate(0, 0)
        self.edge_min         = Coordinate(0, 0)
        self.center           = Coordinate(0, 0)
        self.get_target       = False
        self.target_size      = 0

        update_strategy = { 'Board': self.get_board_object,
                            'Ball' : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_board_object(self):
        max_object_size = max(send.color_mask_subject_size[self.color])
        max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)
        return max_object_idx if max_object_size > 10000 else None

    def get_ball_object(self):
        object_idx = None
        for i in range(send.color_mask_subject_cnts[self.color]):
            length_width_diff = abs(abs(send.color_mask_subject_XMax[self.color][i] - send.color_mask_subject_XMin[self.color][i]) - abs(send.color_mask_subject_YMax[self.color][i] - send.color_mask_subject_YMin[self.color][i]))
            if 100 < send.color_mask_subject_size[self.color][i] < 2500 and length_width_diff < 8:
                object_idx = i
        
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target  = True
            self.edge_max.x  = send.color_mask_subject_XMax[self.color][object_idx]
            self.edge_min.x  = send.color_mask_subject_XMin[self.color][object_idx]
            self.edge_max.y  = send.color_mask_subject_YMax[self.color][object_idx]
            self.edge_min.y  = send.color_mask_subject_YMin[self.color][object_idx]
            self.center.x    = send.color_mask_subject_X[self.color][object_idx]
            self.center.y    = send.color_mask_subject_Y[self.color][object_idx]
            self.target_size = send.color_mask_subject_size[self.color][object_idx]

            # rospy.loginfo(self.target_size)
            # rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
            # send.drawImageFunction(1, 1, self.edge_min.x, self.edge_max.x, self.edge_min.y, self.edge_max.y, 0, 0, 255)
        else:
            self.get_target = False

def Strategy_select():
    if send.DIOValue == 1 or send.DIOValue == 2 or send.DIOValue == 3 or  send.DIOValue == 9 or send.DIOValue == 10 or send.DIOValue == 11:
        return "Lift_and_Carry_off"
    elif send.DIOValue == 17 or send.DIOValue == 18 or  send.DIOValue == 19 or  send.DIOValue == 25 or  send.DIOValue == 26 or  send.DIOValue == 27:
        return "Lift_and_Carry_on"
    elif send.DIOValue == 4 or send.DIOValue == 12:
        return "Wall_Climb_off"
    elif send.DIOValue == 20 or send.DIOValue == 28:
        return "Wall_Climb_on"

if __name__ == '__main__':
    try:
        LC = LiftandCarry()
        CW = WallClimb()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            stategy = Strategy_select()
            # start=time.time()
            if stategy == "Lift_and_Carry_off" or stategy == "Lift_and_Carry_on":
                LC.main()
            elif stategy == "Wall_Climb_off" or stategy == "Wall_Climb_on":
                CW.main()
            # end=time.time()
            # rospy.loginfo(f'策略計算總時間: {end-start}')
            r.sleep()
    except rospy.ROSInterruptException:
        pass

# ░░░░░░░░░▄░░░░░░░░░░░░░░▄░░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀████▀▄▄⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▄█ 
# ░░░░░░░░▌▒█░░░░░░░░░░░▄▀▒▌░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂█▀░░░░▀▀▄▄▄▄▄⠂⠂⠂⠂▄▄▀▀█⠂⠂
# ░░░░░░░░▌▒▒█░░░░░░░░▄▀▒▒▒▐░░░░░░   ⠂⠂⠂▄⠂⠂⠂⠂⠂⠂⠂█░░░░░░░░░░░▀▀▀▀▄░░▄▀ ⠂⠂
# ░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐░░░░░░  ⠂▄▀░▀▄⠂⠂⠂⠂⠂⠂▀▄░░░░░░░░░░░░░░▀▄▀⠂⠂⠂⠂⠂
# ░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░░░░░░   ▄▀░░░░█⠂⠂⠂⠂⠂⠂█▀░░░▄█▀▄░░░░░░▄█⠂⠂⠂⠂⠂
# ░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░░░░░░   ▀▄░░░░░▀▄⠂⠂⠂█░░░░░▀██▀░░░░░██▄█ ⠂⠂⠂⠂
# ░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌░░░░░  ⠂⠂▀▄░░░░▄▀⠂█░░░▄██▄░░░▄░░▄░░▀▀░█ ⠂⠂⠂⠂
# ░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐░░░░░  ⠂⠂⠂█░░▄▀⠂⠂█░░░░▀██▀░░░░▀▀░▀▀░░▄▀⠂⠂⠂⠂
# ░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌░░░░  ⠂⠂█░░░█⠂⠂█░░░░░░▄▄░░░░░░░░░░░▄▀⠂⠂⠂⠂⠂
# ░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌░░░░  ⠂█░░░█⠂⠂█▄▄░░░░░░░▀▀▄░░░░░░▄░█ ⠂⠂⠂⠂⠂
# ▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐░░░░  ⠂⠂▀▄░▄█▄█▀██▄░░▄▄░░░▄▀░░▄▀▀░░░█ ⠂⠂⠂⠂⠂
# ▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌░░░  ⠂⠂⠂⠂▀███░░░░░░░░░▀▀▀░░░░▀▄░░░▄▀⠂⠂⠂⠂⠂
# ▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐░░░░  ⠂⠂⠂⠂⠂⠂▀▀█░░░░░░░░░▄░░░░░░▄▀█▀ ⠂⠂⠂⠂⠂
# ░▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀█░░░░░▄▄▄▀░░▄▄▀▀░▄▀ ⠂⠂⠂⠂⠂
# ░▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▀▀▄▄▄▄▀⠂▀▀▀⠂▀▀▄▄▄▀⠂⠂⠂⠂⠂
# ░░▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌░░░░░
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++==--::::--==+++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++=-...........:-=+++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++=:..............:=++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++-......:--:......-++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++=......:++++:.....:++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++===----=++=-......-++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++++-:......:=++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++=:......:-++++++++++++++++++++++++++++++++++++++++++
# +++++++++*%@@%*++++++++++++++++++++++++++++++++=.......-=+++++++++++++++++++++++++++++++++++++++++++
# ++++++++*@#+**%@#++++++++++++++++++++++++++++++:.....:=+++++++++++++++++++++++++++++++++++++++++++++
# ++++++++@*+==++*#@%*+++++++++++++++++++++++++++::::::-++++++++++++++++++++++++++++++++++++++++++++++
# +++++++#%------+++#@%++++++++++++++++++++++++++=======++++++++++++++++++++++++++++++++++++++++++++++
# +++++++@*-------=+++#@#++++++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# +++++++@=---------+++*%@#++++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# ++++++*@-----------=+++*%%*++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# ++++++*@--------=*%%#*+++*%%*++++++++++++++++++------=+++++++++++++++++++++++++++++++**#%#++++++++++
# ++++++*@--+%%##%%*-:=#@#*++#@%++++++++++++++++++++++++++++++++++++++++++++++++++++*%@%#**@*+++++++++
# +++++++@+**+=-.  .:-.  -+*++*#@%+++++++++++++++++++++++++++++++++++++++++++++++*%@%*+++++%%+++++++++
# +++++++@%+----:.          .:+++#%#++*##########*++++++++++++++++++++++++++++*#@%#+++++===%%+++++++++
# +++++++-..                 .+++++*#####*****####%%@%##*++++++++++++++++++*#%%#*++++=-----%#+++++++++
# +++++=*%%*=.      .-=+***++++++++++++++++++++++++++**#%%%%*+++++++++++*#@%#*+++==-------=@++++++++++
# ++++#@+.      -+#%%#*+++++++++++++++++++++++++++++++++++**#%%#*++++*%@%#*+++=-----------%%++++++++++
# +++%%==-  .=#%%#*+++++++++++++++++++++++++++++++++++++++++++**#%%%%#*++++++****+=------+@+++++++++++
# +++##%=.=#@#*++++++++++++++++++++++++++++++++++++++++++++++++++*****+--=++++======----=@#+++++++++++
# +++@#-*%#*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=:.      :--:.:--%%++++=+++++++
# +++@@%#*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-.    .-*%%**#+=+++=+++++++
# +#@%*+++++++++++++++-:-++++++++++++++++++++++++++++++++++++++++++++++++++++=:      :+%%*+-+++-++++++
# %%*++++++++++++++=: .=++++++++++++++*%*+++++++++++++++++++++++++++++++++++++++=.   .  -%#+-++*-***++
# *++++++++++++++-.  -**++++++++++++*#@@*+++++++++++++++++++++++++++++++++++++++**=  .+#-:#++=*++=***+
# +++++++++++++:   .*#+++++++++++++#@*+@++++++++++++++++++++++++++++++++++++++++++##:  -%#+++-+++=+**+
# +++++++++++-    -%*++++++++++++#@*: +%++++++++++++++++++++++++*%*++++++++++++++++*%+  =%@++=:*=***+*
# ++++++++++:    *@*++++++++++*#@*:   +@++++++++++++++++++++++++#@%+++++++++++++++++*%#.=@%*+-+++=*+++
# +++++++++:   .#%*++++++++*#%%+.     -@*+++++++++++++++++++++++%*%%++++++++++++++++++%%.*%+*-*+++*+++
# ++++++++:   .%%+++++++*#%%*-         #%++++++++++++++++++++++*@::@#+++++++**+++++++++%%:%#*--+++++++
# +++++++:   .#%++++*##%%+-.           :@#++*++++++++++++++++++%+  -@#++++++*%#+++++++++%%*%+++++*++++
# ++++++-    #%*##%@@%%*===--..         =@#+*+++++++++++++++++#%.   -@#++++++*@#+++++++++%@*++++++++++
# +++++=    +@##@@@@@@@@@@@@@@@%*=-.     =@#*#+++++++++++++++*@-     -@#++++++#@*++++++++*%%++++++++++
# +++++.   -@-.++#%#*+======+*#%@@@@%+-.  -%%*#++**+++++++++*@=       :%%*+++++#@*++++++++*@#+++++++++
# ++++-   .%*       :=*#%%%%%%%#+==+#@@@*- .+%*=-#%@@%####*#@+         .+@#*++++%%+++++++++#@+++++++++
# ++++.   +@.    :*%%#*+=======+*#@*- :=+*=.  :.   ..=+++*##+==++****+==::*@%#*+*@#+++++++++%%++++++++
# +++-   .%*   .*@*+===============*%#:                   =@@@@@@@@@@@@@@@%#%@@%#%@+++++++++*@#+++++++
# +++:   =@:  .%%+===================*%=                  .:::::::--=+=+*#@@@@@%#*@#+++++++++#@+++++++
# +++    #%   +@+=====================+%=                   .:+#@@%%%%%%*=:.-=*%@@@%+++++++++*@#++++++
# ++=    @+   +@+======================#%:                :*@%#*+=======+*%%*:  :=#@#+++++++++#%++++++
# ++-   .@-   .%#======================*@:               =@#+==============+*%#:  .@%*++++++++*@*+++++
# ++:   =@:    :@*=====================%%.              =@#===================*@-  #%++++++++++%#+++++
# ++:   =@.     -%#+=================+%%:              .@%=====================#@. +%++++++++++%%+++++
# ++-   =%       .+%#*=============+#@+.               :@#======================@= +%++++++++++#@+++++
# ++=   +%         .-*%%##*+++**#%@#=.                  %@+=====================@= +%++++++++++#@+++++
# **+   +%             .:=++***+=-.                     -%%====================*@: +%++++++++++*@+++++
# **+:  =@.                                              .*%*+===============+#@=  +%++++++++++#@+++++
# +%*=  :@:                                                :+%%#++========+*%@*:   *%++++++++++#@+++++
# +*@*-  %+                                 .-*#+             :=*%%%%##%%@%*-.     %#++++++++++%%+++++
# ++#%*. *@#=.                            =#@#=-%#:                .:-::..        .@#++++++**+*@*+++++
# +++%@+ .@%%@#=:                         .:.   .+%#.                             =@*++++++#%+#@++++++
# %*++#@+.=@**%@@%+-.                             .:.                             #%++++++*@#*@#++++++
# #%@%##%%+*%%+*%##%@%+-:                                                        :@#++++++%@+%%+++++++
# ++*##%##%@%+:%+#%**##@@%#*=-.                                                 :*%++++++#@*#@++++++++
# ++++++*%#.:--@:.#@#**#@#:-=*#%#*+=-:.                                  .:-=+#%@@*+++++#@#*@@*+++++++
# +++++*@*.   =@.  +@%***%@=    .:-=+*#%%#**+===--::::::::::---===++**#%%%%%##**%%+++++#@#*%*%%+++++++
# ++++*@+     +#    :%%#**#@%-           .:%#=+@#****#####**#@@@%@%+%#*++++++++#%*+++*%%*+*++#@+++++++
# +++*@#      #*     .%@#***%@+            -@*#@.         -#@@%#*#@:=@#++++++++%*+++*@%++++++#@+++++++                                                                  
#                                                           ...                                                                      
#      =+=:        .-==.                              .-=++++++++++++++=+=-:                                                          
#     -=.:=++    ++=-:-+                           :=++==++***++*+++++*++++*#*-:                                                      
#     #.::-.#    #.-::.#.                       .=+==+*+++***++++++++##++++*+*%##+:                                                   
#    :+::::+-    -=::::=-                   =+:++-+*++++++*=#+++++++#=#+++*##*****##+..                                               
#    *::::.#     .#.-:-:*                   -#*+++++******-+#++****#-=#++##**#*******#**:                                             
#    #.:::==      +:-:-.#.                 .*+=+****=----+=+*#+=---=-+####*##**#**#**#=+-                                             
#   .*.:::+*+-    -=:--:+:        .%*=-.  -%=*#-=+=-=+*####*+--+*****##***#*##*****#**+*-                                             
#   :*:--.=.:=*=  .*:--:==         :*=*#*#+-=*++==+#*+====--=*#==++**+#****##**#****#**-#.                                            
#   :*:--:.---.#. .#.---:+          :#-=#++*+-=-**++**+*-=#*++**++-**##*#***#*#*#****#*-%%.                                           
#   :*:--:---:==   #.---:*           =*--==-=+**+++++++*=+*+++++*#-#++#***#**#*********=++#-:                                         
#   .#.------:*    #.---.#           +#+---+-#*+++++++++#*++++++*-**++*###*****#***#***==-==*+                                        
#    *:-----:#.    =+===++          +#+*%#**+#+-+++++++++*++++++*-==*+#***##*#*****#**++..:-:-*:                                      
#    :*:---:*:      ....         :+##++%*+++%+*-*=  =++++++++++++=  -**:##***#*******=.::..:.--*-                                     
#     -*--=*:                 .+%#+=+*##=+++*+*-*.  :+:=+++=++++++  ..#-+*#****++++-*::.::.::::++                                     
#      .:-:++++=-:.            .-=**%-#=+++*++#=*   == -++*.-+++++ ....==+*####%**+== ::... :-:=#.                                    
#         *--:::-==++=              +=*-+++*+**=#::-*+++++++++*+++     --*++++++#==--:.:...:.-:+=#                                    
#        ==:-----==-.#.             #*+=+++#+*==#:*+*+++++++++*+++    .--**+++++*=+--:::.....==+#=                                    
#        .+++=----=:+-             .#+=++++#+*-=*:++*+++++++++*++=    ..-+#++++#==+=-=:::---=-*-                                      
#     .=+++=+==--:::-*-        .    #=-+++**+=:-*-=++*++++++++*++-     .=++*++++#*====*+==---##+                                      
#     +-::--=========.#     .=#+#*. =#-+++*++:.=+=-++#++++++++*+*:.   . ====++++**#*#=+++=++#*+*%:                                    
#     +:=============-+    .#-=.--+  %-+==++*--+*+=+=**====+++#+*==----:==+ *+++++*+*###%%##*#%=-.                                    
#     *:-:--=.:====-=+.   .%= - ::%+.%-+===+*. .*-=-==#====++=*=+       ==+.*+++++++++**%#****%.                                      
#     :*+*%=-=====:-+=    .#:.: -.==+#-+===++:::-*--*=*+====+=+*=::::-: ===:*+*++*++++**%*##**%                                       
#      :+=-=========.#.-=-.%..  .:.-+*-===###****%*#++=*====+%#%#*******#=-.*+++++++++*##*#*%%-                                       
#    .++-=====--:===:==-:+:%. :::-.:*+=-+-*-**=:::*=:+=++=-=-=*+#-::--#-++: ++++*++++**%**#+%-                                        
#    #:=====-++#:+===+++-*:+=  .:. .%+*-*=* +*.   =-. .=.:=++=+**     #.*+. =+++***++*##***+%                                         
#    -*-==-++. *-=+++++=:#.:%.  .-++=#%:=#=::*:   +:        .::-*     % ++  =+*+-::+**%***++%                                         
#     .++=*:  .:+*====+++=  ++-==.**=*@==##+ :*   #       .   . -+   =+:+-  =*-.--. #%#**+++%                                         
#       ..   -*==++...      +#- .=#%#=*%-##-..-+++:  .        .  -++*+.=*.. =-.=- . **#*++++%                                         
#            +:++:#       .*= .+%#=-%-.#***.....    .         .   .....*-   . -:--  #*#+++++%                                         
#            *.++.*      .#: -##-   :%:.#*=.....  .               ....-= .  . =--: +#**+++++%                                         
#            #:++.*      +-:+%=      -%:.%+         .              . .-   . . .= .*%*+*+++++%                                 .       
#           .*-++:*      .#+%:        -%--@-       ..+=====++             .=*=#**#*%*+++++++%                                 .       
#           :+=+*:*       :@-:         =%%##*-      .--:::--=           :==::-=#**+%*+++++++%                                         
#           -=++*:*        ==::         *%#**##+-.           ..     :=+++::-::=*++*#*++++*++%                                         
#           =-++*:*         +-:.       :####*##*##%*++==----====++#+==::-----:*=++##+++++*++%                                         
#           +-*+*:*        =.*-:.      -*%%**#**#*%*#*---=+#=-:--=+**--------:*+++##+++++*++%.                                        
#           +:*+*.*        *  *::.      -#%##%*+*+%*#=+*++*****+*++*----------++++*#+*+++*++%:                                        
#           *:***.*       :=  .#::.      =#%##*+*+#*#:**:.+=+=-*:=*--=++-=-+-+-++#+*+*++*#*+#-                                        
#           *:***:*       :+   .#::.      +%##+++*#*#=*=:.-*+%-=-....  .=%#%+#-++%+*+*++#**+*+                                        
#           *---==+       -+    .*:::    .+#%%=++#*#*##. ..++: ......    :#**#-++%##**++#**++#                                        
#       =++-.-=--:  :-==. :*     .*-::. :=**-*=++%*#=*+ . =.+ . ... . . . =%%%-++#*#%+++#*#++%:                                       
#      :*-==+*.   =+===:* .#  -.   ++::. +.= ===*-*+*#-.  :=.  .-:-::- . .:%%%++=#*##++*%*#+=**                                       
#      *:***-*.   +:***=+: #  =.    :*-::.*.+=+=* :*++***+===+++++++#=.   .%%%%+++%#%==##%#+==%.                                      
#     .*-***-+    :*=***-+ +: =:      #+.::+--=++.#++++++++++++++++++**=.. %%%%%%%%%#==#***##*+*                                      
#     =-***-*.     *:#*#:# :+ --      *+=:::+==#-#*++++++++++++++++++%*+*=.#%%%%%%%%+==%****%*=#:                                     
#     *:***:+      ==***=+: +..*      #-%::--===#%+++++++++++++++++++%:=***#*%%%%%%%===%%%#**##=#                                     
#     #:#**=*+=.   .*=***-= .= *.    .#:*+.=--+*:%*+++++++*+++++++++*%- .-+*%%%%%%%%===%%%%#**#**-                                    
#    .#-#*+--=-++.  #-#*#:*    .+    .*:*::=--#*+-%*++++++#+++++++++#@=  -*=%%%#%%%@=-=%%#*****#=#                                    
#    .#-***.**#=+:  #:#*#:#     -:   :+-+ ----%:=:*%##++++*#++++***#%%+:*#####****#%=-=%%#****#%+#:                                   
#     #-#******-+   *:###-#.     .   -=*: =:-=* :*-%#%%***+*#*%##%**#%##***********#=-=#****##%%==+                                   
#     #:#*#*##:#.   +:###=*.         =-#  =--+: ==:%++####**#***++*##**************#+--%****#%%%=-#                                   
#     =-*#*##-*:    -*==++*          **-  =-++ :-.##*+**###*+++++*+*%***************#--%***#%%%@-:#.                                  
#     .#-*##-*-       ...           -#*#.:=-*+ =  *##**++++++++++*#%#***************%--#***##%%#+-*-                                  
#      :*===*-                      *+####=-#=:- .%**+++++++++*#%#*#%***************#=:+#**%%++*#-+=                                  
#        :-:                       -*=%=..+-%=*  +#++++++*####*+++*%%#%**#***********#:=*#%**++*%-==                                  
#                                  #-##:  *-#-+ .%########+*####***%%%%####**********#=-=+:-*++#%=+=                                  
#                                 +##*    *:+++ +#+++++++%#+++++***%%%##**###++%#*%%+#*:-+::#+=%#=+-                                  
#                                 .#=.    =-+++.#+++++=+*+*#+++++++###**#*=.. ..+:**++-+::*.#=+++-*.                                  