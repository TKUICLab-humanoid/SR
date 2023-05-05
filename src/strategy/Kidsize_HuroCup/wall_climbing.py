#!/usr/bin/env python
#coding=utf-8
import sys
import rospy
import numpy as np
import math
from Python_API import Sendmessage

#--校正量--#
#前進量校正
FORWARD_CORRECTION         = 0
#平移校正
TRANSLATION_CORRECTION     = 0
#旋轉校正
THETA_CORRECTION           = 0
#基礎變化量(前進&平移)
BASE_CHANGE                = 200                   
#---微調站姿開關---#
STAND_CORRECT_CW           = False                 #sector(33) CW_stand微調站姿
DRAW_FUNCTION_FLAG         = True                  #影像繪圖開關
LADDER_COLOAR              = "Red"                     
#----------#                       右腳           左腳
#                              左 ,  中,  右|  左,  中,   右
#FOOT                       = [115 , 134, 153, 176, 194, 213]
HEAD_HORIZONTAL            = 2068                  #頭水平
HEAD_VERTICAL              = 2740                  #頭垂直 #down 2750
##判斷值
FOOTLADDER_LINE            = 220                   #上梯基準線
WARNING_DISTANCE           = 4                     #危險距離
#GO_UP_DISTANCE             = 10                    #上板距離
FIRST_FORWORD_CHANGE_LINE  = 50                    #小前進判斷線
SECOND_FORWORD_CHANGE_LINE = 90                    #前進判斷線
THIRD_FORWORD_CHANGE_LINE  = 150                   #大前進判斷線
UP_LADDER_DISTANCE         = 60                    #最低上板需求距離
##前後值
BACK_MIN                   = -500                  #小退後
BACK_NORMAL                = -1000                 #退後
FORWARD_MIN                = 1000                  #小前進
FORWARD_NORMAL             = 2000                  #前進
FORWARD_BIG                = 3000                  #大前進
FORWARD_SUPER              = 5000                  #超大前進
##平移值
TRANSLATION_MIN            = 200                   #小平移
TRANSLATION_NORMAL         = 700                  #平移
TRANSLATION_BIG            = 1200                  #大平移
##旋轉值
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

send       = Sendmessage()
class WallClimbing:
#CW主策略
    def __init__(self):
        self.ladder = ObjectInfo(LADDER_COLOAR,'Ladder')
        self.init()
        
    def main(self,strategy):
        send.sendHeadMotor(1,self.head_Horizontal,100)#水平
        send.sendHeadMotor(2,self.head_Vertical,100)#垂直
        # if DRAW_FUNCTION_FLAG:
        #     self.draw_function()

        sys.stdout.write("\033[H")
        sys.stdout.write("\033[J")
        rospy.loginfo('________________________________________')
        rospy.loginfo(f'x: {self.now_forward} ,y: {self.now_translation} ,theta: {self.now_theta}')
        rospy.loginfo(f'Goal_x: {self.forward} ,Goal_y: {self.translation} ,Goal_theta: {self.theta}')
        rospy.loginfo(f"機器人狀態: {self.state}")
        #rospy.loginfo(f"距離梯: {self.distance}")
        rospy.loginfo('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')
        
        if strategy == "Wall_Climb_off":
        #關閉策略,初始化設定
            if not self.walkinggait_stop:
                rospy.loginfo("🔊CW parameter reset")
                send.sendHeadMotor(1,self.head_Horizontal,100)  #水平
                send.sendHeadMotor(2,self.head_Vertical,100)    #垂直
                send.sendBodyAuto(0,0,0,0,1,0)
                send.sendSensorReset()              #IMUreset
                rospy.sleep(2)
                self.init()
                send.sendBodySector(29)             #基礎站姿磁區
                rospy.sleep(1.5)
                if STAND_CORRECT_CW:
                    send.sendBodySector(33)             #CW基礎站姿調整磁區
                rospy.loginfo("reset🆗🆗🆗")
            rospy.loginfo("turn off")
        elif strategy == "Wall_Climb_on":
        #開啟CW策略
            if self.state != 'cw_finish':
                if self.imu_reset:
                    send.sendSensorReset()
                    send.sendBodyAuto(0,0,0,0,1,0)
                    self.imu_reset = False
                rospy.loginfo(f"blue ymax: {self.lower_blue_ymax}")
                self.find_ladder()
                self.walkinggait(motion=self.edge_judge(strategy))
                    
    def init(self):
        #狀態
        self.state                 = '停止'
        #步態啟動旗標
        self.walkinggait_stop      = True
        self.first_in              = True  
        #設定頭部馬達
        self.head_Horizontal       = HEAD_HORIZONTAL
        self.head_Vertical         = HEAD_VERTICAL
        #imu_reast
        self.imu_reset             = True
        #距離矩陣                     [左左,左中,左右 ,右左,右中,右右 ]
        # self.distance              = [9999,9999,9999,9999,9999,9999]
        # self.next_distance         = [9999,9999,9999,9999,9999,9999]
        #步態參數
        self.forward               = FORWARD_NORMAL + FORWARD_CORRECTION
        self.translation           = 0              + TRANSLATION_CORRECTION
        self.theta                 = 0              + THETA_CORRECTION
        self.now_forward           = 0 
        self.now_translation       = 0
        self.now_theta             = 0  
        self.lower_blue_ymax       = 0

    def find_ladder(self):
    #獲取梯子資訊、距離資訊
        self.ladder.update()

        self.lower_blue_ymax = 0
        self.new_target_xmax = 0
        self.new_target_xmin = 0
        self.new_target_ymax = 0
        self.blue_x_middle = 160
        #-------距離判斷-------#
        for blue_cnt in range (send.color_mask_subject_cnts[2]):
            if send.color_mask_subject_size[2][blue_cnt] > 50:
                self.new_target_xmax = send.color_mask_subject_XMax[2][blue_cnt]
                self.new_target_xmin = send.color_mask_subject_XMin[2][blue_cnt]
                self.new_target_ymax = send.color_mask_subject_YMin[2][blue_cnt]
                
                if self.lower_blue_ymax < self.new_target_ymax:
                    self.lower_blue_ymax = self.new_target_ymax
                    self.blue_x_middle = (self.new_target_xmax + self.new_target_xmin) / 2
        send.drawImageFunction(14,1,self.new_target_xmin,self.new_target_xmax,self.lower_blue_ymax-5,self.lower_blue_ymax+5,255,0,128)
    
    def walkinggait(self,motion):
    #步態函數,用於切換countiue 或 LC 步態
        if motion == 'ready_to_cw':
            rospy.loginfo("對正梯子")
            send.sendBodyAuto(0,0,0,0,1,0)           #停止步態
            send.sendSensorReset()                   #IMU reset 避免機器人步態修正錯誤
            rospy.sleep(3)                           #穩定停止後的搖晃
            send.sendBodySector(29)                  #這是基本站姿的磁區
            while not send.execute:
                rospy.logdebug("站立姿勢")
            send.execute = False
            #-爬梯磁區-#
            send.sendBodySector(40)                  #
            while not send.execute:
                rospy.logdebug("40號磁區")
            send.execute = False
            #---------#
            self.status = 'cw_finish'
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
            if send.imu_value_Yaw > 1:
                self.now_theta = -THETA_MIN
            elif send.imu_value_Yaw < -1:
                self.now_theta = THETA_MIN
            else:
                self.now_theta = 0
            #速度調整
            send.sendContinuousValue(self.now_forward,self.now_translation,0,self.now_theta,0)

    def edge_judge(self,strategy):
    #邊緣判斷,回傳機器人走路速度與走路模式
        if (self.lower_blue_ymax >= FOOTLADDER_LINE - 20) and (self.blue_x_middle >= 158) and (self.blue_x_middle <= 162):
            self.state = "爬梯"
            return "ready_to_cw"
        else:
            if (self.lower_blue_ymax > FOOTLADDER_LINE):
                self.theta = send.imu_value_Yaw
                self.forward = BACK_MIN + FORWARD_CORRECTION
                self.state = "!!!小心採到梯子,後退!!!"
            elif (self.lower_blue_ymax >= FOOTLADDER_LINE - 20) and (self.blue_x_middle < 160):
                self.forward     = BACK_MIN+ FORWARD_CORRECTION
                self.theta       =  0
                self.translation = LEFT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                self.state = "左平移"
            elif (self.lower_blue_ymax >= FOOTLADDER_LINE - 20) and (self.blue_x_middle >160):
                self.forward     = BACK_MIN+ FORWARD_CORRECTION
                self.theta       =  0
                self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                self.state = "右平移"
            else:
                if self.blue_x_middle < 160: #左移
                    self.translation = LEFT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                    self.state = "左平移  "
                elif self.blue_x_middle > 160: #右移
                    self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                    self.state = "右平移  "
                else:
                    self.translation = TRANSLATION_CORRECTION
                
                if (FOOTLADDER_LINE - self.lower_blue_ymax) < FIRST_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_MIN + FORWARD_CORRECTION
                    self.state += '小前進'
                elif (FOOTLADDER_LINE - self.lower_blue_ymax) < SECOND_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.state += '前進'
                elif (FOOTLADDER_LINE - self.lower_blue_ymax) < THIRD_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                    self.state += '大前進'
                else:
                    self.theta      = THETA_CORRECTION
                    self.forward    = FORWARD_BIG + FORWARD_CORRECTION
                    self.state     += '大前進'
            return 'walking'

    # def theta_change(self):
    # #旋轉修正
    #     decide_theta = 0
    #     if self.distance[2] < 240 and self.distance[3] < 240:
    #         slope = self.distance[2] - self.distance[3]             #計算斜率(使用LR-RL)
    #     else:
    #         slope = 0

    #     if self.now_board.edge_min.x > self.distance[1] and slope > 5:
    #         self.theta = THETA_NORMAL*RIGHT_THETA + THETA_CORRECTION
    #         rospy.loginfo('板子太右,右旋')
    #     elif self.now_board.edge_max.x < self.distance[4] and slope < -5:
    #         self.theta = THETA_NORMAL*LEFT_THETA + THETA_CORRECTION
    #         rospy.loginfo('板子太左,左旋')
    #     else:
    #         #---決定左或右轉---#
    #         if   (slope < -1*(SLOPE_MIN)):
    #             decide_theta = LEFT_THETA
    #             rospy.loginfo('左旋')
    #         elif (slope > SLOPE_MIN):
    #             decide_theta = RIGHT_THETA
    #             rospy.loginfo('右旋')
    #         else:
    #             rospy.loginfo('直走')
    #         #-----------------#
    #         if  (abs(slope)) > SLOPE_BIG:                    #斜率過大,角度給最大
    #             self.theta       =  THETA_BIG*decide_theta + THETA_CORRECTION
    #             self.translation = TRANSLATION_NORMAL*decide_theta*-1
    #         elif(abs(slope)) > SLOPE_NORMAL:                 #斜率較大,修正值較大
    #             self.theta       = THETA_NORMAL*decide_theta + THETA_CORRECTION
    #             self.translation = TRANSLATION_MIN*decide_theta*-1
    #         elif(abs(slope)) > SLOPE_MIN:                    #斜率較小,修正值較小
    #             self.theta       = THETA_MIN*decide_theta + THETA_CORRECTION
    #             self.translation = 0+THETA_CORRECTION
    #         else:
    #             self.translation = 0+TRANSLATION_CORRECTION
    #             self.theta       = 0+THETA_CORRECTION

    # def draw_function(self):
    # #畫面顯示繪畫資訊    
    #     #腳的距離判斷線
    #     send.drawImageFunction(1,0,0,320,FOOTLADDER_LINE,FOOTLADDER_LINE,0,128,255)#膝蓋的橫線
    #     send.drawImageFunction(2,0,FOOT[0],FOOT[0],0,240,255,128,128)#lr的線
    #     send.drawImageFunction(3,0,FOOT[1],FOOT[1],0,240,255,128,128)#lm的線
    #     send.drawImageFunction(4,0,FOOT[2],FOOT[2],0,240,255,128,128)#ll的線
    #     send.drawImageFunction(5,0,FOOT[3],FOOT[3],0,240,255,128,128)#rl的線
    #     send.drawImageFunction(6,0,FOOT[4],FOOT[4],0,240,255,128,128)#rm的線
    #     send.drawImageFunction(7,0,FOOT[5],FOOT[5],0,240,255,128,128)#rr的線
    #     #邊緣點
    #     send.drawImageFunction(8,1,FOOT[0]-5,FOOT[0]+5,FOOTLADDER_LINE-self.distance[0]-5,FOOTLADDER_LINE-self.distance[0]+5,255,0,128)
    #     send.drawImageFunction(9,1,FOOT[1]-5,FOOT[1]+5,FOOTLADDER_LINE-self.distance[1]-5,FOOTLADDER_LINE-self.distance[1]+5,255,0,128)
    #     send.drawImageFunction(10,1,FOOT[2]-5,FOOT[2]+5,FOOTLADDER_LINE-self.distance[2]-5,FOOTLADDER_LINE-self.distance[2]+5,255,0,128)
    #     send.drawImageFunction(11,1,FOOT[3]-5,FOOT[3]+5,FOOTLADDER_LINE-self.distance[3]-5,FOOTLADDER_LINE-self.distance[3]+5,255,0,128)
    #     send.drawImageFunction(12,1,FOOT[4]-5,FOOT[4]+5,FOOTLADDER_LINE-self.distance[4]-5,FOOTLADDER_LINE-self.distance[4]+5,255,0,128)
    #     send.drawImageFunction(13,1,FOOT[5]-5,FOOT[5]+5,FOOTLADDER_LINE-self.distance[5]-5,FOOTLADDER_LINE-self.distance[5]+5,255,0,128)

    # def return_real_ladder(self,x,board,outset):
    # #檢查回傳的物件是否為板子,確認連續10個點為同一色模
    #     for y in range(outset,10,-1):
    #         real_distance_flag = (send.Label_Model[320*y+x] == board)
    #         if real_distance_flag:
    #             for i in range(1,11):
    #                 real_distance_flag = (real_distance_flag and send.Label_Model[320*(y-i)+x] == board)
    #                 if not real_distance_flag:
    #                     break
    #         if  real_distance_flag:
    #             break 
    #     return (outset - y,y)if real_distance_flag else (9999,9999)

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
#                                 .#=.    =-+++.#+++++=+*+*#+++++++###**#*=.. ..+:**++-+::*.#=+++-*.