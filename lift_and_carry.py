#!/usr/bin/env python
#coding=utf-8
from re import T
import sys
import rospy
import numpy as np
import math
from Python_API import Sendmessage
from calculate_edge import deep_calculate
#--校正量--#
#前進量校正
FORWARD_CORRECTION         = -300
#平移校正
TRANSLATION_CORRECTION     = 200
#旋轉校正
THETA_CORRECTION           = 0
#基礎變化量(前進&平移)
BASE_CHANGE                = 100                   
#上下板前進量
LCUP                       = 20000                 #上板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = 3.2
LCDOWN                     = 18000                 #下板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = -1.5
#上下板後路徑規劃
ROUTE_PLAN_FORWARD         = [1000,   300,    0, 1500,    0,  600]
ROUTE_PLAN_TRANSLATION     = [2000, -1000, 1500,    0, 1000, 1000]
ROUTE_PLAN_THETA           = [   0,     0,    2,    0,    0,   -2]
ROUTE_PLAN_TIME            = [   0,     0,    0,    0,    0,    0]
#---微調站姿開關---#
STAND_CORRECT_LC           = False                 #sector(30) LC_stand微調站姿
UPBOARD_GROUND_CORRECT     = False                 #第一層上板站姿 (地板 -> 板子)  磁區暫定33記得改!!
UPBOARD_CORRECT            = False                  #sector(31) 上板微調站姿
DOWNBOARD_CORRECT          = True                  #sector(32) 下板微調站姿
DOWNBOARD_GROUND_CORRECT   = False                 #第6層下板站姿 (板子 -> 地板)  磁區暫定34記得改!!
DRAW_FUNCTION_FLAG         = True                  #影像繪圖開關
START_LAYER                = 1
BOARD_COLOR                = ["Green"  ,           #板子顏色(根據比賽現場調整)
                              "Red"   ,           #Blue Red Yellow Green
                              "Yellow" ,
                              "Blue"    ,  
                              "Yellow"    , 
                              "Red"   , 
                              "Green"]              
#----------#                       右腳           左腳
#                              左 ,  中,  右|  左,  中,   右
FOOT                       = [106 , 125, 144, 176, 194, 213]
HEAD_HORIZONTAL            = 2048                  #頭水平
HEAD_VERTICAL              = 1350                  #頭垂直 #down 
##判斷值
FOOTBOARD_LINE             = 230                   #上板基準線
WARNING_DISTANCE           = 4                     #危險距離
GO_UP_DISTANCE             = 10                    #上板距離
FIRST_FORWORD_CHANGE_LINE  = 50                    #小前進判斷線
SECOND_FORWORD_CHANGE_LINE = 90                    #前進判斷線
THIRD_FORWORD_CHANGE_LINE  = 150                   #大前進判斷線
UP_BOARD_DISTANCE          = 60                    #最低上板需求距離
##前後值
BACK_MIN                   = -500                  #小退後
BACK_NORMAL                = -700                 #退後
FORWARD_MIN                = 500                  #小前進
FORWARD_NORMAL             = 800                  #前進
FORWARD_BIG                = 1500                  #大前進
FORWARD_SUPER              = 2000                  #超大前進
##平移值
TRANSLATION_MIN            = 500                   #小平移
TRANSLATION_NORMAL         = 1000                  #平移
TRANSLATION_BIG            = 1500                  #大平移
##旋轉值
THETA_MIN                  = 2                     #小旋轉
THETA_NORMAL               = 4                     #旋轉
THETA_BIG                  = 6                     #大旋轉
SLOPE_MIN                  = 1                     #有點斜
SLOPE_NORMAL               = 3                     #斜
SLOPE_BIG                  = 7                     #過斜
#左基礎參數
LEFT_THETA                 = 1
#右基礎參數
RIGHT_THETA                = -1
#前進基礎參數
FORWARD_PARAM              = 1
#後退基礎參數
BACK_PARAM                 = -1

send       = Sendmessage()
edge       = deep_calculate(5)
class LiftandCarry:
#LC主策略
    def __init__(self):
        self.init()
        
    def main(self,strategy):
        send.sendHeadMotor(1,self.head_Horizontal,100)#水平
        send.sendHeadMotor(2,self.head_Vertical,100)#垂直
        if DRAW_FUNCTION_FLAG:
            self.draw_function()

        sys.stdout.write("\033[H")
        sys.stdout.write("\033[J")
        rospy.loginfo('________________________________________')
        rospy.loginfo(f'x: {self.now_forward} ,y: {self.now_translation} ,theta: {self.now_theta}')
        rospy.loginfo(f'Goal_x: {self.forward} ,Goal_y: {self.translation} ,Goal_theta: {self.theta}')
        rospy.loginfo(f"機器人狀態: {self.state}")
        rospy.loginfo(f"機器人角度: {self.angle}")
        rospy.loginfo(f"SLOPE: {edge.slope}")
        if self.layer < 7:
            rospy.loginfo(f"層數: {self.layer},{BOARD_COLOR[self.layer]}")
        rospy.loginfo(f"距離板: {self.distance}")
        rospy.loginfo(f"上板空間: {self.next_distance}")
        rospy.loginfo(f"板大小: {self.now_board.target_size}")
        rospy.loginfo('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')
        if strategy == "Lift_and_Carry_off":
        #關閉策略,初始化設定
            if not self.walkinggait_stop:
                rospy.loginfo("🔊LC parameter reset")
                send.sendHeadMotor(1,self.head_Horizontal,100)  #水平
                send.sendHeadMotor(2,self.head_Vertical,100)    #垂直
                send.sendBodyAuto(0,0,0,0,1,0)
                rospy.sleep(2)
                send.sendBodySector(29)             #基礎站姿磁區
                # while not send.execute:
                #     rospy.logdebug("站立姿勢")
                # send.execute = False
                # if STAND_CORRECT_LC:
                #     send.sendBodySector(30)             #LC基礎站姿調整磁區
                #     while not send.execute:
                #         rospy.logdebug("站立姿勢調整")
                #     send.execute = False
                rospy.loginfo("reset🆗🆗🆗")
            self.init()
            send.sendSensorReset(1,1,1)
            rospy.loginfo("turn off")
        elif strategy == "Lift_and_Carry_on":
        #開啟LC策略
            if self.layer < 7:
                if self.walkinggait_stop and self.first_in:
                    sys.stdout.write("\033[H")
                    sys.stdout.write("\033[J")
                    send.sendBodySector(29)             #基礎站姿磁區
                    while not send.execute:
                        rospy.logdebug("站立姿勢")
                    send.execute = False
                    if STAND_CORRECT_LC:
                        #send.sendBodySector(30)             #LC基礎站姿調整磁區
                        send.sendBodySector(2) 
                        while not send.execute:
                            rospy.logdebug("站立姿勢調整")
                        send.execute = False
                        rospy.sleep(1) 
                    send.sendBodyAuto(self.forward,0,0,0,1,0)
                    self.walkinggait_stop = False
                    self.first_in         = False
                    self.route_plan(self.layer)
                elif self.walkinggait_stop and not self.first_in:
                    send.sendBodyAuto(0,0,0,0,1,0)
                    self.walkinggait_stop = False
                    self.route_plan(self.layer)
                elif not self.walkinggait_stop:
                    self.find_board()
                    self.walkinggait(motion=self.edge_judge())
                    
    def init(self):
        #狀態
        self.state                 = '停止'
        self.angle                 = '直走'
        #步態啟動旗標
        self.walkinggait_stop      = True
        self.first_in              = True  
        #層數       
        self.layer                 = START_LAYER
        #設定頭部馬達
        self.head_Horizontal       = HEAD_HORIZONTAL
        self.head_Vertical         = HEAD_VERTICAL
        #距離矩陣                     [左左,左中,左右 ,右左,右中,右右 ]
        self.distance              = [9999,9999,9999,9999,9999,9999]
        self.next_distance         = [9999,9999,9999,9999,9999,9999]
        #步態參數
        self.forward               = FORWARD_NORMAL + FORWARD_CORRECTION
        self.translation           = 0              + TRANSLATION_CORRECTION
        self.theta                 = 0              + THETA_CORRECTION
        self.now_forward           = 0 
        self.now_translation       = 0
        self.now_theta             = 0  
        #建立板子資訊
        self.next_board            = ObjectInfo(BOARD_COLOR[self.layer+1],'Board') #設定下一個尋找的板子
        self.now_board             = ObjectInfo(BOARD_COLOR[self.layer],'Board')   #設定當前尋找的板子
        self.last_board            = None                                          #設定前一階板子
        edge.color                 = ObjectInfo.color_dict[BOARD_COLOR[self.layer]]
        
    def find_board(self):
    #獲取板子資訊、距離資訊
        self.next_board.update()
        self.now_board.update()
        if self.last_board is not None:
            self.last_board.update()
        #腳與邊緣點距離
        self.distance         = [9999,9999,9999,9999,9999,9999]
        self.next_distance    = [9999,9999,9999,9999,9999,9999]
        #邊緣點
        now_edge_point        = [9999,9999,9999,9999,9999,9999]
        next_edge_point       = [9999,9999,9999,9999,9999,9999]
        #-------距離判斷-------#
        for i in range(6):
            self.distance[i],now_edge_point[i] = self.return_real_board(outset=FOOTBOARD_LINE, end_range=10, x=FOOT[i],board=self.now_board.color_parameter, translate_bool=True)
        #-----------------#
        if self.layer != 6 or self.layer != 3:
        #除了上最頂層和下最底層以外,偵測上下板空間
            for i in range(6):
                if now_edge_point[i]>240:
                    continue
                else:
                    self.next_distance[i] ,next_edge_point[i]= self.return_real_board(outset=now_edge_point[i], end_range=10, x=FOOT[i],board=self.next_board.color_parameter, translate_bool=True)
    
    def walkinggait(self,motion):
    #步態函數,用於切換countiue 或 LC 步態
        if motion == 'ready_to_lc':
            rospy.loginfo("對正板子")
            send.sendBodyAuto(0,0,0,0,1,0)           #停止步態
            rospy.sleep(3)                           #穩定停止後的搖晃
            send.sendSensorReset(1,1,1)              #IMU reset 避免機器人步態修正錯誤
            send.sendBodySector(29)                  #這是基本站姿的磁區
            while not send.execute:
                rospy.logdebug("站立姿勢")
            send.execute = False
            if self.layer < 4:
                if UPBOARD_CORRECT:
                    rospy.loginfo("準備上板")
                    send.sendWalkParameter('send',\
                                            walk_mode = 2,\
                                            com_y_shift = 0,\
                                            y_swing = 5.5,\
                                            period_t = 600,\
                                            t_dsp = 0.3,\
                                            base_default_z = 5,\
                                            right_z_shift = 2.3,\
                                            base_lift_z = 2.8,\
                                            com_height = 27.5,\
                                            stand_height = 23.5,\
                                            back_flag = 0)
                    rospy.sleep(1.5)
                    send.sendBodySector(31)          #上板前站姿調整
                    while not send.execute:
                        rospy.logdebug("上板前姿勢")
                    rospy.sleep(1.5)
                    send.execute = False                   #微調站姿延遲
                send.sendBodyAuto(LCUP,0,0,0,2,0)    #上板步態
            else:
                if DOWNBOARD_CORRECT:
                    rospy.loginfo("準備下板")
                    send.sendWalkParameter('send',\
                                            walk_mode = 3,\
                                            com_y_shift = -3,\
                                            y_swing = 5.5,\
                                            period_t = 570,\
                                            t_dsp = 0.3,\
                                            base_default_z = 3.2,\
                                            right_z_shift = 1,\
                                            base_lift_z = -2,\
                                            com_height = 59.5,\
                                            stand_height = 21.5,\
                                            back_flag = 0)
                    rospy.sleep(1.5)
                    send.sendBodySector(32)          #下板前站姿調整
                    while not send.execute:
                        rospy.logdebug("下板前姿勢")
                    rospy.sleep(1.5)
                    send.execute = False               #微調站姿延遲
                if self.layer == 4:
                    send.sendBodyAuto(18000, 0, 0, 0, 3, 0)
                else:
                    send.sendBodyAuto(LCDOWN,0,0,0,3,0)  #下板步態
            rospy.sleep(5)  
            send.sendBodySector(39)               
            send.sendWalkParameter('send',\
                                    walk_mode = 1,\
                                    stand_height = 23.5)
            rospy.sleep(2)                           #剛下板,等待搖晃
            send.sendBodySector(29)                  #這是基本站姿的磁區
            while not send.execute:
                rospy.logdebug("站立姿勢")
            send.execute = False
            rospy.sleep(1.5)
            if STAND_CORRECT_LC:
                send.sendBodySector(30)              #基礎站姿調整
                while not send.execute:
                    rospy.logdebug("站立姿勢調整")
                send.execute = False
            rospy.sleep(1)
            #-初始化-#
            self.forward        = 0
            self.translation    = 0
            self.theta          = 0
            self.layer += 1                          #層數加一
            edge.color = ObjectInfo.color_dict[BOARD_COLOR[self.layer]]
            self.walkinggait_stop   = True
            if self.layer < 7:
                self.now_board  = ObjectInfo(BOARD_COLOR[self.layer],'Board')   #設定當前尋找的板子
                self.last_board = None 
                if self.layer != 4:
                    if self.layer != 6:
                        self.next_board = ObjectInfo(BOARD_COLOR[self.layer+1],'Board') #設定下一個尋找的板子
                    self.last_board = ObjectInfo(BOARD_COLOR[self.layer-2],'Board') #設定前一個板子
                else:
                    self.next_board = ObjectInfo(BOARD_COLOR[self.layer+1],'Board') #設定下一個尋找的板子
                # self.checkout_board()                 #轉頭找板
            #-------#
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
            
            if self.now_translation >1000 and self.now_forward >2000:
                self.now_forward = 2000
            #速度調整
            send.sendContinuousValue(self.now_forward,self.now_translation,0,self.now_theta,0)

    def edge_judge(self):
    #邊緣判斷,回傳機器人走路速度與走路模式
        if ((self.distance[0] < GO_UP_DISTANCE ) and (self.distance[1] < GO_UP_DISTANCE ) and\
           (self.distance[2] < GO_UP_DISTANCE ) and (self.distance[3] < GO_UP_DISTANCE) and\
           (self.distance[4] < GO_UP_DISTANCE) and (self.distance[5] < GO_UP_DISTANCE + 5)) or \
           ((self.distance[0] < GO_UP_DISTANCE + 5) and (self.distance[1] < GO_UP_DISTANCE ) and \
           (self.distance[2] < GO_UP_DISTANCE ) and (self.distance[3] < GO_UP_DISTANCE) and \
           (self.distance[4] < GO_UP_DISTANCE) and (self.distance[5] < GO_UP_DISTANCE)):
           #上板
           if self.layer <= 3:
                self.state = "上板"
                return 'ready_to_lc'
           else:
                if((self.distance[0] < GO_UP_DISTANCE - 7) and (self.distance[1] < GO_UP_DISTANCE - 7) and\
                    (self.distance[2] < GO_UP_DISTANCE - 7) and (self.distance[3] < GO_UP_DISTANCE - 7) and\
                    (self.distance[4] < GO_UP_DISTANCE - 7)) or\
                    ((self.distance[1] < GO_UP_DISTANCE - 7) and (self.distance[2] < GO_UP_DISTANCE - 7) and\
                    (self.distance[3] < GO_UP_DISTANCE - 7) and (self.distance[4] < GO_UP_DISTANCE - 7) and\
                    (self.distance[5] < GO_UP_DISTANCE - 7)):
                    self.state = "下板"
                    return 'ready_to_lc'
                else:
                    self.forward     = FORWARD_MIN + FORWARD_CORRECTION
                    self.translation = TRANSLATION_CORRECTION
                    self.theta       = THETA_CORRECTION
    #===============================================================================================
        else:
            #前後!!!
            #踩板
            if (self.distance[0] <= WARNING_DISTANCE ) or (self.distance[1] <= WARNING_DISTANCE ) or \
               (self.distance[2] <= WARNING_DISTANCE ) or (self.distance[3] <= WARNING_DISTANCE ) or \
               (self.distance[4] <= WARNING_DISTANCE ) or (self.distance[5] <= WARNING_DISTANCE ):
                #需要再改!!!
                if self.layer == 4:
                    self.special_case()
                else:
                    self.forward = BACK_NORMAL + FORWARD_CORRECTION
            elif min(self.distance) <= WARNING_DISTANCE + 3:
                self.forward = FORWARD_CORRECTION
            elif min(self.distance) >= SECOND_FORWORD_CHANGE_LINE:
                self.forward = FORWARD_BIG + FORWARD_CORRECTION
            elif min(self.distance) <= FIRST_FORWORD_CHANGE_LINE:
                self.forward = FORWARD_MIN + FORWARD_CORRECTION
            
            #translate
            left_point = self.return_real_board(outset = FOOTBOARD_LINE + 5, end_range = 70, x = FOOT[0], board = self.now_board.color_parameter, translate_bool = False)
            right_point = self.return_real_board(outset = FOOTBOARD_LINE + 5, end_range = FOOT[5], x = 260, board = self.now_board.color_parameter, translate_bool = False)
            if left_point and right_point:
                self.translation = 0
            elif not left_point and not right_point:
                self.translation = 0
            elif left_point or right_point:
                self.translation = TRANSLATION_NORMAL if right_point else -TRANSLATION_NORMAL

            #theta
            rotate = min(self.distance[0], self.distance[1], self.distance[2]) - min(self.distance[3], self.distance[4], self.distance[5])
            foot_distance_left  = [self.distance[0], self.distance[1], self.distance[2]]
            foot_distance_right = [self.distance[3], self.distance[4], self.distance[5]]
            if ((foot_distance_left.index(min(foot_distance_left)) == 1) or (foot_distance_left.index(min(foot_distance_left)) == 2)) and \
                ((foot_distance_right.index(min(foot_distance_right)) == 0) or (foot_distance_right.index(min(foot_distance_right)) == 1)):
                if (foot_distance_left[0] - foot_distance_left[2]) > (foot_distance_right[2] - foot_distance_right[0]):
                    self.theta = RIGHT_THETA * THETA_NORMAL
                elif (foot_distance_left[0] - foot_distance_left[2]) < (foot_distance_right[2] - foot_distance_right[0]):
                    self.theta = LEFT_THETA * THETA_NORMAL
            elif (max(self.distance) - min(self.distance) <= 3):
                self.theta = 0
            elif abs(rotate) >= 40:
                self.theta = RIGHT_THETA * THETA_BIG if rotate > 0 else LEFT_THETA * THETA_BIG
            elif abs(rotate) >= 20:
                self.theta = RIGHT_THETA * THETA_NORMAL if rotate > 0 else LEFT_THETA * THETA_NORMAL
            elif abs(rotate) >= 10:
                self.theta = RIGHT_THETA * THETA_MIN if rotate > 0 else LEFT_THETA * THETA_MIN
            else:
                self.theta = RIGHT_THETA * 0 if rotate >= 0 else LEFT_THETA * 0
            self.theta += THETA_CORRECTION

            # if abs(rotate) < 5 and 

    def theta_change(self):
    #旋轉修正
        decide_theta = 0
        # if self.distance[2] < 240 and self.distance[3] < 240:
        #     slope = self.distance[2] - self.distance[3]             #計算斜率(使用LR-RL)
        # else:
        #     slope = 0

        slope = edge.slope
        
        # if self.now_board.edge_min.x > self.distance[1] and slope > 5:
        #     self.theta = THETA_NORMAL*RIGHT_THETA + THETA_CORRECTION
        #     rospy.loginfo('板子太右,右旋')
        # elif self.now_board.edge_max.x < self.distance[4] and slope < -5:
        #     self.theta = THETA_NORMAL*LEFT_THETA + THETA_CORRECTION
        #     rospy.loginfo('板子太左,左旋')
        # else:
            #---決定左或右轉---#
        if   (slope > 0):
            decide_theta = LEFT_THETA
            self.angle = '左旋'
        elif (slope < 0):
            decide_theta = RIGHT_THETA
            self.angle = '右旋'
            
        #-----------------#
        if  (abs(slope)) > SLOPE_BIG:                    #斜率過大,角度給最大
            self.theta       =  THETA_BIG*decide_theta + THETA_CORRECTION
            self.translation = TRANSLATION_NORMAL*decide_theta*-1
        elif(abs(slope)) > SLOPE_NORMAL:                 #斜率較大,修正值較大
            self.theta       = THETA_NORMAL*decide_theta + THETA_CORRECTION
            self.translation = TRANSLATION_MIN*decide_theta*-1
        elif(abs(slope)) > SLOPE_MIN:                    #斜率較小,修正值較小
            self.theta       = THETA_MIN*decide_theta + THETA_CORRECTION
            self.translation = 0+THETA_CORRECTION
        else:
            self.translation = 0+TRANSLATION_CORRECTION
            self.theta       = 0+THETA_CORRECTION
            self.angle = '直走'

    def no_up_board(self):
    #上板或下板後影像上無下一層板
        if self.last_board is not None:
            if self.last_board.get_target:
                self.forward     = FORWARD_CORRECTION
                self.theta       = self.theta
                self.translation = self.translation

    def checkout_board(self):
    #上板完後尋找下一板
        self.find_board()
        if (self.distance[0] >250 and self.distance[1] >250 and self.distance[2] >250 ) or (self.distance[3] >250 and self.distance[4] >250 and self.distance[5] >250):
            find_board_in_right = False
            find_board_in_left  = False
            send.sendHeadMotor(2,self.head_Vertical-100,100)
            for i in range(1800,1000,-10):
            #找右邊
                send.sendHeadMotor(1,i,100)
                if self.now_board.get_target:                  
                    if self.now_board.target_size > 2500 and self.now_board.edge_max.x > 160:
                        find_board_in_right = True
                        rospy.loginfo(f"板子在右邊")
                if find_board_in_right:
                    if i < 1200:
                        self.theta = THETA_BIG*RIGHT_THETA
                    elif i <1500:
                        self.theta = THETA_NORMAL*RIGHT_THETA
                    else:
                        self.theta = THETA_MIN*RIGHT_THETA
                    break
                rospy.sleep(0.05) #轉頭delay
            send.sendHeadMotor(1,self.head_Horizontal,100)#水平
            rospy.sleep(2)  #讓頭先轉回正中間 
            if not find_board_in_right:
                for k in range(2200,3000,10):
                #找左邊
                    send.sendHeadMotor(1,k,100)
                    if self.now_board.get_target:
                        if self.now_board.target_size > 2500 and self.now_board.edge_min.x < 160:
                            find_board_in_left  = True
                            rospy.loginfo("板子在左邊")
                    if find_board_in_left :
                        if k > 2650:
                            self.theta = THETA_BIG*LEFT_THETA
                        elif k >2350:
                            self.theta = THETA_NORMAL*LEFT_THETA
                        else:
                            self.theta = THETA_MIN*LEFT_THETA
                        break
                    rospy.sleep(0.05) #轉頭delay
            send.sendHeadMotor(1,self.head_Horizontal,100)#水平
            if self.layer <4:
                send.sendHeadMotor(2,self.head_Vertical,100)#垂直
            else:
                send.sendHeadMotor(2,self.head_Vertical+45,100)#垂直

    def draw_function(self):
    #畫面顯示繪畫資訊    
        #腳的距離判斷線
        send.drawImageFunction(1,0,0,320,FOOTBOARD_LINE,FOOTBOARD_LINE,0,128,255)#膝蓋的橫線
        send.drawImageFunction(2,0,FOOT[0],FOOT[0],0,240,255,128,128)#lr的線
        send.drawImageFunction(3,0,FOOT[1],FOOT[1],0,240,255,128,128)#lm的線
        send.drawImageFunction(4,0,FOOT[2],FOOT[2],0,240,255,128,128)#ll的線
        send.drawImageFunction(5,0,FOOT[3],FOOT[3],0,240,255,128,128)#rl的線
        send.drawImageFunction(6,0,FOOT[4],FOOT[4],0,240,255,128,128)#rm的線
        send.drawImageFunction(7,0,FOOT[5],FOOT[5],0,240,255,128,128)#rr的線
        #邊緣點
        send.drawImageFunction(8,1,FOOT[0]-5,FOOT[0]+5,FOOTBOARD_LINE-self.distance[0]-5,FOOTBOARD_LINE-self.distance[0]+5,255,0,128)
        send.drawImageFunction(9,1,FOOT[1]-5,FOOT[1]+5,FOOTBOARD_LINE-self.distance[1]-5,FOOTBOARD_LINE-self.distance[1]+5,255,0,128)
        send.drawImageFunction(10,1,FOOT[2]-5,FOOT[2]+5,FOOTBOARD_LINE-self.distance[2]-5,FOOTBOARD_LINE-self.distance[2]+5,255,0,128)
        send.drawImageFunction(11,1,FOOT[3]-5,FOOT[3]+5,FOOTBOARD_LINE-self.distance[3]-5,FOOTBOARD_LINE-self.distance[3]+5,255,0,128)
        send.drawImageFunction(12,1,FOOT[4]-5,FOOT[4]+5,FOOTBOARD_LINE-self.distance[4]-5,FOOTBOARD_LINE-self.distance[4]+5,255,0,128)
        send.drawImageFunction(13,1,FOOT[5]-5,FOOT[5]+5,FOOTBOARD_LINE-self.distance[5]-5,FOOTBOARD_LINE-self.distance[5]+5,255,0,128)

    def return_real_board(self, x, board, outset, end_range, translate_bool):
    #檢查回傳的物件是否為板子,確認連續10個點為同一色模
        if translate_bool: 
            for y in range(outset, end_range,-1):
                real_distance_flag = (send.Label_Model[320*y+x] == board)
                if real_distance_flag:
                    for i in range(1,11):
                        real_distance_flag = (real_distance_flag and send.Label_Model[320*(y-i)+x] == board)
                        if not real_distance_flag:
                            break
                if  real_distance_flag:
                    break 
            return (outset - y,y) if real_distance_flag else (9999,9999)
        else:
            for y in range(x, end_range, -1):
                real_distance_flag = (send.Label_Model[320*outset + y] == board)
                if  real_distance_flag:
                    break 
            return True if real_distance_flag else False
        
    
    def special_case(self):
    #頂板判斷
        if   self.distance[0] > 0:
            left_slope = self.distance[0] - self.distance[2]
        elif self.distance[1] > 0:
            left_slope = self.distance[1] - self.distance[2]
        else:
            left_slope = 0

        if   self.distance[4] > 0:
            right_slope = self.distance[3] - self.distance[4]
        elif self.distance[5] > 0:
            right_slope = self.distance[3] - self.distance[5]
        else:
            right_slope = 0

        if left_slope*right_slope > 0:
        #頂板直走
            if (min(self.distance[0],self.distance[1])) < GO_UP_DISTANCE and (min(self.distance[3],self.distance[4])) > FIRST_FORWORD_CHANGE_LINE:
                self.forward     = FORWARD_NORMAL+ FORWARD_CORRECTION
                self.theta       = THETA_CORRECTION
                self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                self.state       = "快掉板了,右平移"
            elif (min(self.distance[1],self.distance[2])) < FIRST_FORWORD_CHANGE_LINE and (min(self.distance[4],self.distance[5])) > GO_UP_DISTANCE:
                self.forward     = FORWARD_NORMAL+ FORWARD_CORRECTION
                self.theta       = THETA_CORRECTION
                self.translation = LEFT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                self.state       = "快掉板了,左平移"
            else:
                self.forward     = FORWARD_BIG+ FORWARD_CORRECTION
                self.theta       = THETA_CORRECTION
                self.translation = TRANSLATION_CORRECTION
        else:
        #看到90度板
            if abs(left_slope)>abs(right_slope):
                self.forward     = FORWARD_CORRECTION
                self.theta       = LEFT_THETA*THETA_NORMAL + THETA_CORRECTION
                self.translation = TRANSLATION_CORRECTION
                self.state       = "角度錯誤,左轉"
            elif abs(left_slope)<abs(right_slope):
                self.forward     = FORWARD_CORRECTION
                self.theta       = RIGHT_THETA*THETA_NORMAL + THETA_CORRECTION
                self.translation = TRANSLATION_CORRECTION
                self.state       = "角度錯誤,右轉"

    def route_plan(self,now_layer):
    #路徑規劃
        start = rospy.get_time()
        end   = 99999
        rospy.sleep(1)       #啟動步態後穩定時間
        while (end-start) < ROUTE_PLAN_TIME[now_layer - 1]:
            end = rospy.get_time()
            print(end-start)
            self.forward     = ROUTE_PLAN_FORWARD[now_layer - 1]
            self.translation = ROUTE_PLAN_TRANSLATION[now_layer - 1]
            self.theta       = ROUTE_PLAN_THETA[now_layer - 1]
            send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)
                 
    def aa(self):
        cnt=0
        for i in range(FOOT[0],FOOT[5],1):
            for j in range(FOOTBOARD_LINE,FOOTBOARD_LINE-10,-1):
                if (send.Label_Model[320*j+i] == self.now_board.color_parameter):
                    cnt = cnt +1 
        return cnt
    
class Coordinate:
#儲存座標
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
#物件的影件資訊
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

        update_strategy = { 'Board': self.get_object,
                            'Ladder': self.get_object,
                            'Ball' : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_object(self):
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
        else:
            self.get_target = False

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
#                                 .#=.    =-+++.#+++++=+*+*#+++++++###**#*=.. ..+:**++-+::*.#=+++-*.++
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