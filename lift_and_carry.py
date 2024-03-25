#!/usr/bin/env python
#coding=utf-8
import sys
import rospy
import numpy as np
import math
from Python_API import Sendmessage
from calculate_edge import deep_calculate
#--校正量--#
#前進量校正
FORWARD_CORRECTION         = -200
#平移校正
TRANSLATION_CORRECTION     = 0
#旋轉校正
THETA_CORRECTION           = 0
#基礎變化量(前進&平移)
BASE_CHANGE                = 100                   
#上下板前進量
LCUP                       = 20000                 #上板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = 3.2
LCDOWN                     = 20000                 #下板 Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = -1.5
#上下板後路徑規劃
ROUTE_PLAN_FLAG            = True
ROUTE_PLAN_FORWARD         = [2000,   0,    200,    -400,     -500, 0]
ROUTE_PLAN_TRANSLATION     = [500,   500, -1200,    -800,     0, 0]   #pos = left, neg = right
ROUTE_PLAN_THETA           = [   4,  -4,    4,    5,    5, 0]   #pos = left, neg = right
ROUTE_PLAN_TIME            = [  5,   4,    3,   17,     5, 0]
#---微調站姿開關---#
STAND_CORRECT_LC           = True                  #sector(30) LC_stand微調站姿

GND_BOARD_LC               = False                  #地板到板 磁區33              1
UPBOARD_LAYER_TWO          = False                  #sector(31) 上板微調站姿      2
UPBOARD_LAYER_THREE        = False                  #sector(35) 上板微調站姿      3
DOWNBOARD_LAYER_FOUR       = False                  #sector(32) 下板微調站姿      4
DOWNBOARD_LAYER_FIVE       = False                  #sector(36) 下板微調站姿      5
BOARD_GND_LC               = False                   #板到地 磁區34

DRAW_FUNCTION_FLAG         = True                 #影像繪圖開關
START_LAYER                = 1
BOARD_COLOR                = ["Green"  ,           #板子顏色(根據比賽現場調整)
                              "Red"   ,           #Blue Red Yellow Green
                              "Yellow"    , 
                              "Blue" , 
                              "Yellow"    , 
                              "Red"   , 
                              "Green"]              
#----------#                       右腳           左腳
#                              左 ,  中,  右|  左,  中,   右
FOOT                       = [115 , 134, 153, 176, 194, 213]
HEAD_HORIZONTAL            = 2048                  #頭水平
HEAD_VERTICAL              = 1400                #頭垂直 #down 2750
##判斷值
FOOTBOARD_LINE             = 215                 #上板基準線
WARNING_DISTANCE           = 4                     #危險距離
GO_UP_DISTANCE             = 15                    #上板距離
FIRST_FORWORD_CHANGE_LINE  = 50                    #小前進判斷線
SECOND_FORWORD_CHANGE_LINE = 100                   #前進判斷線
THIRD_FORWORD_CHANGE_LINE  = 150                   #大前進判斷線
UP_BOARD_DISTANCE          = 60                    #最低上板需求距離

BACK_MIN                   = -500                  #小後退
BACK_NORMAL                = -1000                  #後退
FORWARD_MIN                = 1000                  #小前進
FORWARD_NORMAL             = 1500                  #前進
FORWARD_BIG                = 2000                  #大前進
FORWARD_SUPER              = 3000                  #超大前進

##平移值
TRANSLATION_MIN            = 500                   #小平移
TRANSLATION_NORMAL         = 800                  #平移
TRANSLATION_BIG            = 1200                  #大平移
##旋轉值
THETA_MIN                  = 3                     #小旋轉
THETA_NORMAL               = 4                     #旋轉
THETA_BIG                  = 5                     #大旋轉
SLOPE_MIN                  = 2                     #有點斜
SLOPE_NORMAL               = 4                     #斜
SLOPE_BIG                  = 12                     #過斜
#左基礎參數
LEFT_THETA                 = 1
#右基礎參數
RIGHT_THETA                = -1
#前進基礎參數
FORWARD_PARAM              = 1
#後退基礎參數
BACK_PARAM                 = -1

send       = Sendmessage()
edge       = deep_calculate(5,1)
class LiftandCarry:
#LC主策略
    def __init__(self):
        self.init()
    def main(self,strategy):
        send.sendHeadMotor(1,self.head_Horizontal,100)#水平
        if self.layer <4:
            send.sendHeadMotor(2,self.head_Vertical,100)#垂直
        else:
            send.sendHeadMotor(2,self.head_Vertical-25,100)#垂直

        if DRAW_FUNCTION_FLAG:
            self.draw_function()

        sys.stdout.write("\033[J\033[H")
        rospy.loginfo('________________________________________\033[K')
        rospy.loginfo(f"SLOPE: {edge.slope}\033[K")
        if self.layer < 7:
            rospy.loginfo(f"層數: {self.layer},{BOARD_COLOR[self.layer]}\033[K")

        if strategy == "Lift_and_Carry_off":
        #關閉策略,初始化設定
            if not self.walkinggait_stop:
                rospy.loginfo("🔊LC parameter reset\033[K")
                send.sendHeadMotor(1,self.head_Horizontal,100)  #水平
                send.sendHeadMotor(2,self.head_Vertical,100)    #垂直
                send.sendBodyAuto(0,0,0,0,1,0)
                rospy.sleep(1.5)
                send.sendBodySector(29)             #基礎站姿磁區
                rospy.loginfo("reset🆗🆗🆗\033[K")
            self.init()
            send.sendSensorReset(1,1,1)
            rospy.loginfo("turn off\033[K")
        elif strategy == "Lift_and_Carry_on":
        #開啟LC策略
            if self.layer < 7:
                if self.walkinggait_stop and self.first_in:
                    sys.stdout.write("\033[H")
                    sys.stdout.write("\033[J")
                    send.sendBodySector(29)             #基礎站姿磁區
                    while not send.execute:
                        rospy.logdebug("站立姿勢\033[K")
                    send.execute = False
                    if STAND_CORRECT_LC:
                        send.sendBodySector(102)             #LC基礎站姿調整磁區
                        while not send.execute:
                            rospy.logdebug("站立姿勢調整\033[K")
                        send.execute = False
                        rospy.sleep(1) 
                    send.sendBodyAuto(self.forward,0,0,0,1,0)
                    self.walkinggait_stop = False
                    self.first_in         = False
                    self.route_plan(self.layer)
                elif self.walkinggait_stop and not self.first_in:
                    if self.layer > 3:
                        send.data_check = False
                        self.find_board()
                        # if (max(self.distance) - min(self.distance) <= 5) and min(self.distance) == 0:
                        if (self.distance[0] == 0 and self.distance[1] == 0 and self.distance[2] == 0 and self.distance[3] == 0) or \
                           (self.distance[2] == 0 and self.distance[3] == 0 and self.distance[4] == 0 and self.distance[5] == 0) or \
                           (self.distance[0] == 0 and self.distance[1] == 0 and max(self.distance) < 2) or \
                           (self.distance[4] == 0 and self.distance[5] == 0 and max(self.distance) < 2):
                            rospy.logwarn("！！！！！！！！！！直接下板！！！！！！！！！！\033[K")
                            self.walkinggait(motion = 'continue_to_lc')
                   
                    send.sendBodyAuto(0,0,0,0,1,0)
                    self.walkinggait(motion = 'walking')
                    self.walkinggait_stop = False
                    self.route_plan(self.layer)
                elif not self.walkinggait_stop:
                    send.data_check = False
                    self.find_board()
                    self.walkinggait(motion=self.edge_judge())
        rospy.loginfo('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣\033[K')
                    
    def init(self):
        #狀態
        self.state                 = '停止'
        self.angle                 = '直走'
        self.search                = 'right'
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
        edge.layer = self.layer
        
    def find_board(self):
    #獲取板子資訊、距離資訊
        if send.data_check == True:
            if self.layer < 6:
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
            self.distance[i],now_edge_point[i] = self.return_real_board(outset=FOOTBOARD_LINE,x=FOOT[i],board=self.now_board.color_parameter)
        #-----------------#
        if self.layer != 6 or self.layer != 3:
        #除了上最頂層和下最底層以外,偵測上下板空間
            for i in range(6):
                if now_edge_point[i]>240:
                    continue
                else:
                    self.next_distance[i] ,next_edge_point[i]= self.return_real_board(outset=now_edge_point[i],x=FOOT[i],board=self.next_board.color_parameter)

        rospy.loginfo(f"距離板: {self.distance}\033[K")
        rospy.loginfo(f"上板空間: {self.next_distance}\033[K")
        rospy.loginfo(f"板大小: {self.now_board.target_size}\033[K")
    
    def walkinggait(self,motion):
    #步態函數,用於切換countiue 或 LC 步態
        rospy.loginfo(f"\r機器人狀態: {self.state}\033[K")
        if motion == 'ready_to_lc' or motion == 'continue_to_lc':
            rospy.loginfo("對正板子\033[K")
            rospy.sleep(0.25)
            if motion == 'ready_to_lc':
                send.sendBodyAuto(0,0,0,0,1,0)           #停止步態
                rospy.sleep(3)                           #穩定停止後的搖晃
            send.sendSensorReset(1,1,1)              #IMU reset 避免機器人步態修正錯誤
            send.sendBodySector(29)                  #這是基本站姿的磁區
            while not send.execute:
                rospy.logdebug("站立姿勢\033[K")
            send.execute = False
            if self.layer < 4:
                if GND_BOARD_LC and self.layer == 1:
                    send.sendWalkParameter('send',\
                                                walk_mode = 2,\
                                                com_y_shift =-5,\
                                                y_swing = 4.5,\
                                                period_t = 450,\
                                                t_dsp = 0.4,\
                                                base_default_z = 3,\
                                                right_z_shift = 3,\
                                                base_lift_z = 5,\
                                                com_height = 29.5,\
                                                stand_height = 23.5,\
                                                back_flag = 0)
                    rospy.sleep(1.5)
                    rospy.loginfo("準備上板\033[K")
                    send.sendBodySector(100)          #上板前站姿調整
                    while not send.execute:
                        rospy.logdebug("上板前姿勢\033[K")
                    rospy.sleep(1.5)
                    send.execute = False  
                elif UPBOARD_LAYER_TWO and self.layer == 2:
                    send.sendWalkParameter('send',\
                                                walk_mode = 2,\
                                                com_y_shift =-4,\
                                                y_swing = 4.5,\
                                                period_t = 420,\
                                                t_dsp = 0.4,\
                                                base_default_z = 3,\
                                                right_z_shift = 3,\
                                                base_lift_z = 5,\
                                                com_height = 29.5,\
                                                stand_height = 23.5,\
                                                back_flag = 0)
                    rospy.sleep(1.5)
                    rospy.loginfo("準備上板\033[K")
                    send.sendBodySector(100)          #上板前站姿調整
                    while not send.execute:
                        rospy.logdebug("上板前姿勢\033[K")
                    rospy.sleep(1.5)
                    send.execute = False                   #微調站姿延遲
                elif UPBOARD_LAYER_THREE and self.layer == 3:
                    send.sendWalkParameter('send',\
                                                walk_mode = 2,\
                                                com_y_shift =-4,\
                                                y_swing = 4.5,\
                                                period_t = 420,\
                                                t_dsp = 0.4,\
                                                base_default_z = 3,\
                                                right_z_shift = 3,\
                                                base_lift_z = 5,\
                                                com_height = 29.5,\
                                                stand_height = 23.5,\
                                                back_flag = 0)
                    rospy.sleep(1.5)
                    rospy.loginfo("準備上板\033[K")
                    send.sendBodySector(100)          #上板前站姿調整
                    while not send.execute:
                        rospy.logdebug("上板前姿勢\033[K")
                    rospy.sleep(1.5)
                    send.execute = False                   #微調站姿延遲
                else:
                    send.sendWalkParameter('send',\
                                                walk_mode = 2,\
                                                com_y_shift =-4,\
                                                y_swing = 4.5,\
                                                period_t = 420,\
                                                t_dsp = 0.4,\
                                                base_default_z = 3,\
                                                right_z_shift = 3,\
                                                base_lift_z = 5,\
                                                com_height = 29.5,\
                                                stand_height = 23.5,\
                                                back_flag = 0)
                    
                    rospy.sleep(1.5)
                send.sendBodyAuto(LCUP,0,0,0,2,0)    #上板步態
            else:
                if BOARD_GND_LC and self.layer == 6:
                    send.sendWalkParameter('send',\
                                            walk_mode = 3,\
                                            com_y_shift = -3,\
                                            y_swing = 4.5,\
                                            period_t = 480,\
                                            t_dsp = 0.4,\
                                            base_default_z = 4,\
                                            right_z_shift = 2,\
                                            base_lift_z = -2,\
                                            com_height = 29.5,\
                                            stand_height = 23.5,\
                                            back_flag = 0)
                    rospy.sleep(2)
                    rospy.loginfo("準備下板\033[K")
                    send.sendBodySector(101)          #下板前站姿調整
                    while not send.execute:
                        rospy.logdebug("下板前姿勢\033[K")
                    rospy.sleep(1.5)
                    send.execute = False               #微調站姿延遲
                elif DOWNBOARD_LAYER_FOUR and self.layer == 4:
                    send.sendWalkParameter('send',\
                                            walk_mode = 3,\
                                            com_y_shift = -3,\
                                            y_swing = 4.5,\
                                            period_t = 480,\
                                            t_dsp = 0.4,\
                                            base_default_z = 4,\
                                            right_z_shift = 2,\
                                            base_lift_z = -2,\
                                            com_height = 29.5,\
                                            stand_height = 23.5,\
                                            back_flag = 0)
                    rospy.sleep(2)
                    rospy.loginfo("準備下板\033[K")
                    send.sendBodySector(101)          #下板前站姿調整
                    while not send.execute:
                        rospy.logdebug("下板前姿勢\033[K")
                    rospy.sleep(1.5)
                    send.execute = False               #微調站姿延遲
                elif DOWNBOARD_LAYER_FIVE and self.layer == 5:
                    send.sendWalkParameter('send',\
                                            walk_mode = 3,\
                                            com_y_shift = -3,\
                                            y_swing = 4.5,\
                                            period_t = 480,\
                                            t_dsp = 0.4,\
                                            base_default_z = 4,\
                                            right_z_shift = 2,\
                                            base_lift_z = -2,\
                                            com_height = 29.5,\
                                            stand_height = 23.5,\
                                            back_flag = 0)
                    rospy.sleep(2)
                    rospy.loginfo("準備下板\033[K")
                    send.sendBodySector(101)          #下板前站姿調整
                    while not send.execute:
                        rospy.logdebug("下板前姿勢\033[K")
                    rospy.sleep(1.5)
                    send.execute = False               #微調站姿延遲
                else:
                    send.sendWalkParameter('send',\
                                            walk_mode = 3,\
                                            com_y_shift = -3,\
                                            y_swing = 4.5,\
                                            period_t = 480,\
                                            t_dsp = 0.4,\
                                            base_default_z = 4,\
                                            right_z_shift = 2,\
                                            base_lift_z = -2,\
                                            com_height = 29.5,\
                                            stand_height = 23.5,\
                                            back_flag = 0)
                    rospy.sleep(2)
                if self.layer == 4:
                    send.sendBodyAuto(20000, 0, 0, 0, 3, 0)
                else:
                    send.sendBodyAuto(LCDOWN,0,0,0,3,0)  #下板步態
            rospy.sleep(3)                           #剛下板,等待搖晃            
            send.sendWalkParameter('send',\
                                    walk_mode = 1,\
                                    com_y_shift = -2,\
                                    y_swing = 5,\
                                    period_t = 330,\
                                    t_dsp = 0,\
                                    base_default_z = 1.5,\
                                    com_height = 29.5,\
                                    stand_height = 23.5)
            rospy.sleep(2) 
            send.sendBodySector(29)                  #這是基本站姿的磁區
            while not send.execute:
                rospy.logdebug("站立姿勢\033[K")
            send.execute = False
            rospy.sleep(1.5)
            if STAND_CORRECT_LC:
                send.sendBodySector(102)              #基礎站姿調整
                while not send.execute:
                    rospy.logdebug("站立姿勢調整\033[K")
                send.execute = False
            rospy.sleep(1)
            #-初始化-#
            self.forward        = 0
            self.translation    = 0
            self.theta          = 0
            self.layer += 1                          #層數加一
            self.walkinggait_stop   = True
            if self.layer < 7:
                edge.color = ObjectInfo.color_dict[BOARD_COLOR[self.layer]]
                edge.layer = self.layer
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
            rospy.loginfo(f'x: {self.now_forward} ,y: {self.now_translation} ,theta: {self.now_theta}\033[K')
            rospy.loginfo(f'Goal_x: {self.forward} ,Goal_y: {self.translation} ,Goal_theta: {self.theta}\033[K')

    def edge_judge(self):
    #邊緣判斷,回傳機器人走路速度與走路模式
        if ((self.distance[0] < GO_UP_DISTANCE+5) and (self.distance[1] < GO_UP_DISTANCE+3) and\
           (self.distance[2] < GO_UP_DISTANCE+3) and (self.distance[3] < GO_UP_DISTANCE+2) and\
           (self.distance[4] < GO_UP_DISTANCE+2)and (self.distance[5] < GO_UP_DISTANCE+5) and self.layer < 4):
           #上板
           self.state = "上板"
           return 'ready_to_lc'
        elif ((self.distance[0] < GO_UP_DISTANCE +5) and (self.distance[1] < GO_UP_DISTANCE+5) and\
           (self.distance[2] < GO_UP_DISTANCE +5 ) and (self.distance[3] < GO_UP_DISTANCE+5) and\
           (self.distance[4] < GO_UP_DISTANCE+5)and (self.distance[5] < GO_UP_DISTANCE+5)and self.layer == 6):
           self.state = "下底板"
           return 'ready_to_lc'
        elif ((self.distance[0] < GO_UP_DISTANCE) and (self.distance[1] < GO_UP_DISTANCE) and\
           (self.distance[2] < GO_UP_DISTANCE) and (self.distance[3] < GO_UP_DISTANCE) and\
           (self.distance[4] < GO_UP_DISTANCE)and (self.distance[5] < GO_UP_DISTANCE)and self.layer >=4):
           #上板
           self.state = "下板"
           return 'ready_to_lc'
        else:
            if (self.distance[0] <= WARNING_DISTANCE) or (self.distance[1] <= WARNING_DISTANCE) or (self.distance[2] <= WARNING_DISTANCE) or (self.distance[3] <= WARNING_DISTANCE) or (self.distance[4] <= WARNING_DISTANCE) or (self.distance[5] <= WARNING_DISTANCE): 
            #即將踩板
                # if self.layer == 4:
                #     self.special_case()
                # else:
                if self.layer < 4:
                    if max(self.distance[0],self.distance[1],self.distance[2])>30:
                        self.forward = FORWARD_CORRECTION 
                        self.translation = RIGHT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                        if abs(self.distance[0]-self.distance[2]) < 5:
                            self.theta   =  0
                        else:
                            self.theta   = RIGHT_THETA*THETA_NORMAL + THETA_CORRECTION
                        self.state   = "!!!右平移!!!"
                    elif max(self.distance[3],self.distance[4],self.distance[5])>30:
                        self.forward = FORWARD_CORRECTION 
                        self.translation = LEFT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                        if abs(self.distance[3]-self.distance[5]) < 5:
                            self.theta   =  0
                        else:
                            self.theta   = LEFT_THETA*THETA_NORMAL + THETA_CORRECTION
                        self.state   = "!!!左平移!!!"
                    else:
                        self.forward = FORWARD_MIN + FORWARD_CORRECTION
                        self.theta_change()
                        self.state = "!!!小心踩板,後退!!!"
                else:
                    if self.distance[0] < GO_UP_DISTANCE and min(self.distance[3],self.distance[4],self.distance[5]) > GO_UP_DISTANCE:
                        self.forward = BACK_MIN + FORWARD_CORRECTION 
                        self.translation = RIGHT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                        self.theta   =  THETA_MIN*LEFT_THETA
                        self.state   = "!!!右平移,左旋!!!"
                    elif self.distance[5] < GO_UP_DISTANCE and min(self.distance[0],self.distance[1],self.distance[2]) > GO_UP_DISTANCE:
                        self.forward = BACK_MIN + FORWARD_CORRECTION 
                        self.translation = LEFT_THETA * TRANSLATION_MIN + TRANSLATION_CORRECTION
                        self.theta   =  THETA_MIN*RIGHT_THETA
                        self.state   = "!!!左平移,右旋!!!"
                    else:
                        self.forward = FORWARD_CORRECTION
                        self.theta_change()
                        self.state = "!!!小心踩板,後退!!!"
            else:
                # if self.layer > 1 and not self.now_board.get_target:
                #     self.state = "前方沒有要上的板子"
                #     self.no_up_board()
                # # elif self.now_board.get_target and max(self.distance[0],self.distance[1],self.distance[2]) > 240 and max(self.distance[3],self.distance[4],self.distance[5]) > 240:
                # #     if min(self.distance[0],self.distance[1],self.distance[2]) < 240:
                # #         self.forward     = FORWARD_CORRECTION
                # #         self.theta       = LEFT_THETA*THETA_NORMAL
                # #         self.translation = TRANSLATION_CORRECTION
                # #     elif min(self.distance[3],self.distance[4],self.distance[5]) < 240:
                # #         self.forward     = FORWARD_CORRECTION
                # #         self.theta       = RIGHT_THETA*THETA_NORMAL
                # #         self.translation = TRANSLATION_CORRECTION
                # else:
                if self.layer == 4:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.theta       = THETA_CORRECTION
                elif self.distance[0] < SECOND_FORWORD_CHANGE_LINE or self.distance[1] < SECOND_FORWORD_CHANGE_LINE or self.distance[2] < SECOND_FORWORD_CHANGE_LINE or self.distance[3] < SECOND_FORWORD_CHANGE_LINE or self.distance[4] < SECOND_FORWORD_CHANGE_LINE or self.distance[5] < SECOND_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.theta_change()
                    self.state = '前進'
                elif self.distance[0] < THIRD_FORWORD_CHANGE_LINE or self.distance[5] < THIRD_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                    self.theta_change()
                    self.state = '大前進'
                else:
                    self.theta = THETA_CORRECTION
                    if self.layer == 1:
                        self.forward     = FORWARD_SUPER + FORWARD_CORRECTION
                        self.state = '超大前進' 
                    else:
                        self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                        self.state = '大前進'
                self.translation = TRANSLATION_CORRECTION           #距離板太遠不須平移
            return 'walking'
    def theta_change(self):
    #旋轉修正
        decide_theta = 0
        # if self.distance[2] < 240 and self.distance[3] < 240:
        #     slope = self.distance[2] - self.distance[3]             #計算斜率(使用LR-RL)
        # else:
        #     slope = 0

        slope = edge.slope
        rospy.logerr(slope)
        sys.stdout.write("\033[K")
        

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
        
        if slope > 10 and self.layer == 4:
            self.theta       = 0+THETA_CORRECTION
        rospy.loginfo(f"機器人角度: {self.angle}\033[K")

    def no_up_board(self):
    #上板或下板後影像上無下一層板
        rospy.logerr(self.now_board.color)
        sys.stdout.write("\033[K")
        rospy.logerr(self.now_board.get_target)
        sys.stdout.write("\033[K")
        if self.layer != 4:
            if self.now_board.edge_min.x >= 162:
                self.theta = RIGHT_THETA * THETA_BIG + THETA_CORRECTION
            elif self.now_board.edge_max.x <= 158 and self.now_board.edge_max.x != 0:
                self.theta = LEFT_THETA * THETA_BIG + THETA_CORRECTION
            else:
                self.theta = THETA_CORRECTION
            if self.layer < 4:
                self.forward     = FORWARD_BIG+FORWARD_CORRECTION
            else:
                self.forward     = FORWARD_CORRECTION
        else:
            self.forward     = FORWARD_BIG+FORWARD_CORRECTION
            self.theta       = THETA_CORRECTION
            self.translation = TRANSLATION_CORRECTION

    def checkout_board(self, right_max = 2048-600, left_max = 2048+600, up_max = 2048, down_max = 2048-300 , scale = 30):
    #找板 右->下->左->上
        while not self.now_board.get_target:
            if self.search == 'right':
                self.control_head(1, self.head_horizon, scale)
                self.head_horizon -= scale
                if self.head_horizon < right_max:
                    self.head_horizon = right_max
                    self.search = 'down'

            elif self.search == 'down':
                self.control_head(2, self.head_vertical, scale)
                self.head_vertical -= scale
                if self.head_vertical < down_max:
                    self.head_vertical = down_max
                    self.search = 'left'

            elif self.search == 'left':
                self.control_head(1, self.head_horizon, scale)
                self.head_horizon += scale
                if self.head_horizon > left_max:
                    self.head_horizon = left_max
                    self.search = 'up'

            elif self.search == 'up':
                self.control_head(2, self.head_vertical, scale)
                self.head_vertical += scale
                if self.head_vertical > up_max:
                    self.head_vertical = up_max
                    self.search = 'right'

        if self.head_horizon > 2248:
            self.theta       = THETA_NORMAL*LEFT_THETA + THETA_CORRECTION
        elif self.head_horizon > 2048:
            self.theta       = THETA_MIN*LEFT_THETA + THETA_CORRECTION
        elif self.head_horizon < 2048:
            self.theta       = THETA_MIN*RIGHT_THETA + THETA_CORRECTION
        elif self.head_horizon < 1848:
            self.theta       = THETA_NORMAL*RIGHT_THETA + THETA_CORRECTION

        send.sendHeadMotor(1,self.head_Horizontal,100)#水平
        send.sendHeadMotor(2,self.head_Vertical,100)#垂直
        rospy.sleep(1)

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
        #第二板邊緣點
        send.drawImageFunction(14,1,FOOT[0]-5,FOOT[0]+5,FOOTBOARD_LINE-self.distance[0]-self.next_distance[0]-5,FOOTBOARD_LINE-self.distance[0]-self.next_distance[0]+5,0,90,128)
        send.drawImageFunction(15,1,FOOT[1]-5,FOOT[1]+5,FOOTBOARD_LINE-self.distance[1]-self.next_distance[1]-5,FOOTBOARD_LINE-self.distance[1]-self.next_distance[1]+5,0,90,128)
        send.drawImageFunction(16,1,FOOT[2]-5,FOOT[2]+5,FOOTBOARD_LINE-self.distance[2]-self.next_distance[2]-5,FOOTBOARD_LINE-self.distance[2]-self.next_distance[2]+5,0,90,128)
        send.drawImageFunction(17,1,FOOT[3]-5,FOOT[3]+5,FOOTBOARD_LINE-self.distance[3]-self.next_distance[3]-5,FOOTBOARD_LINE-self.distance[3]-self.next_distance[3]+5,0,90,128)
        send.drawImageFunction(18,1,FOOT[4]-5,FOOT[4]+5,FOOTBOARD_LINE-self.distance[4]-self.next_distance[4]-5,FOOTBOARD_LINE-self.distance[4]-self.next_distance[4]+5,0,90,128)
        send.drawImageFunction(19,1,FOOT[5]-5,FOOT[5]+5,FOOTBOARD_LINE-self.distance[5]-self.next_distance[5]-5,FOOTBOARD_LINE-self.distance[5]-self.next_distance[5]+5,0,90,128)
        # #板子
        # send.drawImageFunction(20,1,self.now_board.edge_min.x,self.now_board.edge_max.x,self.now_board.edge_min.y,self.now_board.edge_max.y,128,0,0)
        # send.drawImageFunction(21,1,self.next_board.edge_min.x,self.next_board.edge_max.x,self.next_board.edge_min.y,self.next_board.edge_max.y,0,128,0)
        # send.drawImageFunction(22,1,self.last_board.edge_min.x,self.last_board.edge_max.x,self.last_board.edge_min.y,self.last_board.edge_max.y,0,0,128)

    def return_real_board(self,x,board,outset):
    #檢查回傳的物件是否為板子,確認連續10個點為同一色模
        for y in range(outset,10,-1):
            real_distance_flag = (send.Label_Model[320*y+x] == board)
            if real_distance_flag:
                for i in range(1,11):
                    real_distance_flag = (real_distance_flag and send.Label_Model[320*(y-i)+x] == board)
                    if not real_distance_flag:
                        break
            if  real_distance_flag:
                break 
        return (outset - y,y)if real_distance_flag else (9999,9999)
    
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
            if (min(self.distance[0],self.distance[1])) < GO_UP_DISTANCE and (min(self.distance[3],self.distance[4],self.distance[5])) > FIRST_FORWORD_CHANGE_LINE:
                self.forward     = FORWARD_NORMAL+ FORWARD_CORRECTION
                self.theta       = THETA_CORRECTION
                self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                self.state       = "快掉板了,右平移"
            elif (min(self.distance[0],self.distance[1],self.distance[2])) < FIRST_FORWORD_CHANGE_LINE and (min(self.distance[4],self.distance[5])) > GO_UP_DISTANCE:
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
        if ROUTE_PLAN_FLAG:
            start = rospy.get_time()
            end   = 99999
            rospy.sleep(1)       #啟動步態後穩定時間
            sys.stdout.write("\033[H\033[J")
            while (end-start) < ROUTE_PLAN_TIME[now_layer-1]:
                end = rospy.get_time()
                print(end-start)
                print(now_layer)
                print("\033[K\033[H")
                self.forward     = ROUTE_PLAN_FORWARD[now_layer-1]+FORWARD_CORRECTION
                self.translation = ROUTE_PLAN_TRANSLATION[now_layer-1]+TRANSLATION_CORRECTION
                self.theta       = ROUTE_PLAN_THETA[now_layer-1]+THETA_CORRECTION
                
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
        return max_object_idx if max_object_size > 500 else None

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
            # send.data_check = False
            # rospy.loginfo(self.target_size)
            # rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
        else:
            self.get_target = False
            self.edge_max.x  = 0
            self.edge_min.x  = 0
            self.edge_max.y  = 0
            self.edge_min.y  = 0
            self.center.x    = 0
            self.center.y    = 0
            self.target_size = 0

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
