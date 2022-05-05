#!/usr/bin/env python
#coding=utf-8
# from sys import orig_argv
from base64 import b16decode
# from msvcrt import LK_LOCK
from re import S
import rospy
import numpy as np
from rospy import Publisher
from tku_msgs.msg import Interface,HeadPackage,SandHandSpeed,DrawImage,SingleMotorData,\
SensorSet,ObjectList,LabelModelObjectList,RobotPos,SetGoalPoint,SoccerDataList,SensorPackage
from std_msgs.msg import Int16,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

from Python_API import Sendmessage
from SR_API import Send_distance

send = Sendmessage()                                                                    #要放在class外面，否則會宣告失效
distance = Send_distance()#建立名稱,順便歸零

'''上次上機測試，策略上碰到了以下問題 :
        1. 機器人在剛起步時就開始左右踏步修正機器人位置，而花費大量時間。
        2. 不完善的判斷，導致機器人往左轉後，就轉不回來。

    本次修改/新增了下列功能 :
        1. 已修正上述問題1. ，現在機器人會持續直行，並只進行左右轉的校正。
        2. 粗略修正上述問題2. ，待本週末(5/7、5/8)新增中心點藍色判斷，進行完善。現階段判端式堪用。
        3. 修改變數名稱、新增註解
        4. 新增梯前微調

    本週預計進度(5/7~5/8) :
        1. 新增指撥、小指撥功能
        2. 完善中心點藍色判斷
    
    與super_strategy對照後，相同用途的變數可使用的不多，或是尚不知如何修改(會陸續更進一步了解詳細的變數用途)，
    故在本次修改中只將倆旗標變數名稱同步，其餘變數皆是新增的。
'''

#訂定〝傳送4筆距離資料，分別是ll, lr, rl, rr類別
class Ladder_send_distance():
    def __init__(self):
# 旗標
        self.walk_flag = False                  # True 為 "走路中"，False 為 "停下"
        self.up_ladder_flag = False      # True 為 "可上梯"，False 為 "未啟動/完成上梯"

# loop數值
        self.Times = 0

# 距離偵測值初始化
    # 雙腳四線
        # LL代表左腳左線
        # LR代表左腳右線
        # RL代表右腳左線
        # RR代表右腳右線

        # RED代表測紅梯
        self.distance_RED_LL = 999
        self.distance_RED_LR = 999
        self.distance_RED_RL = 999
        self.distance_RED_RR = 999
        self.distance_RED = [999,999,999,999]

        # BLUE代表測中心點
        # 此處用於中心點藍色判斷，未完成，預計本週末前完成
        self.distance_BLUE_LL = 999 
        self.distance_BLUE_LR = 999
        self.distance_BLUE_RL = 999
        self.distance_BLUE_RR = 999
        self.distance_BLUE = [999,999,999,999]

    # 邊界距離
        # 邊界只測紅色
        # b1代表左邊界
        # b2代表右邊界
        self.distance_Boundary_Left = 999
        self.distance_Boundary_Right = 999
        self.distance_Boundary = [999,999]


# 各種距離設定值
    # 腳離板子距離，用於停下時及停下後的變數
        self.distance_Backward = 30                     #腳離板子的距離，微調時使用。若小於此值則後退
        self.distance_Up_Stop = 40                        #上梯停下數值

    #下方兩變數皆為ll與rr的誤差，但用於不同狀況
        self.Deviation_UpLadder = 15                          #上梯前停下的ll與rr的可接受誤差。(先停下)
        self.MinorDeviation_UpLadder = 5             #Minor表示"微調時使用"。用在兩腳距離梯子的誤差，使機器人盡量平行梯子。(再微調)


# 各種速度調整 sendContinuousValue 
    # X值調整、走路速度調整
        self.Speed_NoMove_X = 0                                      # 不走x值

        self.Speed_Straight_X = -100                                 # 直走x值
        self.Speed_Backward_X = -300                           # 微調後退值

    # Y值調整、左右踏調整
        self.Speed_NoStep_Y = 0                                     # 原地微調/不踏步y值
        self.Speed_LeftStep_Y = 700                              # 左踏y值，NoStep + 660
        self.Speed_RightStep_Y = -600                         # 右踏y值，NoStop - 330

    # Theta值調整、左右轉調整
        self.Speed_NoTurn_Theta = 0                             # 直走/不轉彎z值

        self.Speed_LeftTurn_Theta = 4                          # 左轉z值
        self.Speed_RightTurn_Theta = -4                     # 右轉z值
        

#Delay時間
        self.Delay_Stop = 1                                                   # 機器人停下，準備上梯的delay


#上梯動作串磁區呼叫（將0改成所須磁區數值即可）
        self.Sector = 0

# sendBodyAutoX值
    #定值
        self.SendBodyAuto_Start = 400                #停下x值。只是用作停下機器人的x值，相當於定值


# 機器人上梯子前位置/距離校正
    def Ladder_revision(self):
        if self.walk_flag == True and self.up_ladder_flag == True:
            if self.distance_RED_LL !=999 and self.distance_RED_RR != 999:
                if self.distance_RED_LL - self.distance_RED_RR < 40 or self.distance_RED_RR - self.distance_RED_LL < 40:    #此行解決robot左轉後無法轉回的問題，暫時使用，之後會修掉
                    if self.distance_RED_LL < self.distance_RED_RR:
                        print(' 左腳在前，向左轉',end = '\n\n ')
                        send.sendContinuousValue(self.Speed_Straight_X,self.Speed_NoStep_Y,0,self.Speed_LeftTurn_Theta,0)
                        if self.distance_RED_LL < self.distance_Up_Stop or self.distance_RED_RR < self.distance_Up_Stop:
                            print(' 機器人到達，原地左轉',end = '\n\n ')
                            send.sendContinuousValue(self.Speed_NoMove_X,self.Speed_NoStep_Y,0,self.Speed_LeftTurn_Theta,0)
                            if self.distance_RED_RR - self.distance_RED_LL < self.Deviation_UpLadder:
                                print(' 機器人到達，準備微調',end = '\n\n ')
                                self.walk_flag = False

                    elif self.distance_RED_LL > self.distance_RED_RR:
                        print(' 右腳在前，向右轉',end = '\n\n ') 
                        send.sendContinuousValue(self.Speed_Straight_X,self.Speed_NoStep_Y,0,self.Speed_RightTurn_Theta,0)
                        if self.distance_RED_LL < self.distance_Up_Stop or self.distance_RED_RR < self.distance_Up_Stop:
                            print(' 機器人到達，原地右轉',end = '\n\n ')
                            send.sendContinuousValue(self.Speed_NoMove_X,self.Speed_NoStep_Y,0,self.Speed_RightTurn_Theta,0)
                            if self.distance_RED_LL - self.distance_RED_RR < self.Deviation_UpLadder:
                                print(' 機器人到達，準備微調',end = '\n\n ')
                                self.walk_flag = False

                    elif self.distance_RED_LL == self.distance_RED_RR:
                        print(' 直走，不調整',end = '\n\n ')
                        send.sendContinuousValue(self.Speed_Straight_X,self.Speed_NoStep_Y,0,self.Speed_NoTurn_Theta,0)
                        if self.distance_RED_LL < self.distance_Up_Stop or self.distance_RED_RR < self.distance_Up_Stop:        
                            print(' 機器人到達，準備微調',end = '\n\n ')
                            self.walk_flag = False


# 機器人在梯子前微調
    def Direction_Deviation(self):
        if self.walk_flag == False and self.up_ladder_flag == True:
            if self.distance_RED_LL < self.distance_Backward or self.distance_RED_LR < self.distance_Backward or self.distance_RED_RL < self.distance_Backward or self.distance_RED_RR < self.distance_Backward:
                print('過於靠梯，微調後退中...')
                send.sendContinuousValue(self.Speed_Backward_X,self.Speed_NoStep_Y,0,self.Speed_NoTurn_Theta,0)

            elif self.distance_RED_LL  < self.distance_RED_RR:
                print(' 左腳在前，原地左轉',end ='\n\n')
                send.sendContinuousValue(self.Speed_NoMove_X,self.Speed_NoStep_Y,0,self.Speed_LeftTurn_Theta,0)

                if self.distance_RED_RR - self.distance_RED_LL < self.MinorDeviation_UpLadder:
                    print(' 左轉微調完成，準備上梯',end ='\n\n')
                    send.sendBodyAuto(self.SendBodyAuto_Start,0,0,0,1,0)
                    time.sleep(self.Delay_Stop)
            elif self.distance_RED_RR < self.distance_RED_LL:
                print(' 右腳在前，原地右轉',end ='\n\n')
                send.sendContinuousValue(self.Speed_NoMove_X,self.Speed_NoStep_Y,0,self.Speed_RightTurn_Theta,0)

                if self.distance_RED_LL - self.distance_RED_RR < self.MinorDeviation_UpLadder:
                    print(' 右轉微調完成，準備上梯',end ='\n\n')
                    send.sendBodyAuto(self.SendBodyAuto_Start,0,0,0,1,0)
                    time.sleep(self.Delay_Stop)


#上梯子磁區呼叫
    def Up_ladder(self):
        if self.walk_flag == False and self.up_ladder_flag == True:
            send.sendBodySector(15)
            time.sleep(2)




#/////////////////////////////////////////////////////////////////////////////////////

# 色模找梯子
    def Find_ladder(self):
        # 距離偵測值初始化
            # LL代表左腳左線
            # LR代表左腳右線
            # RL代表右腳左線
            # RR代表右腳右線

            # RED代表測紅梯
            self.distance_RED_LL = 999
            self.distance_RED_LR = 999
            self.distance_RED_RL = 999
            self.distance_RED_RR = 999
            self.distance_RED = [999,999,999,999]

            self.distance_Boundary_Left = 999
            self.distance_Boundary_Right = 999
            self.distance_Boundary = [999,999]


            for j in range(230,0,-1):        
                # if send.Label_Model[320*j+115] ==  4:                       
                #     self.distance_BLUE_LL = 230 - j
                #     self.distance_BLUE[0] = self.distance_BLUE_LL          
                #     break
                if send.Label_Model[320*j+115] == 32 :                       
                    self.distance_RED_LL = 230 - j
                    self.distance_RED[0] = self.distance_RED_LL          
                    break
        
            for i in range(230,0,-1):
                # if send.Label_Model[320*i+115] ==  4:                       
                #     self.distance_BLUE_LR = 230 - i
                #     self.distance_BLUE[1] = self.distance_BLUE_LR          
                #     break
                if send.Label_Model[320*i+150] == 32 :              
                    self.distance_RED_LR = 230 - i
                    self.distance_RED[1] = self.distance_RED_LR
                    break

            for k in range(230,0,-1):
                # if send.Label_Model[320*k+115] ==  4:                       
                #     self.distance_BLUE_RL = 230 - k
                #     self.distance_BLUE[2] = self.distance_BLUE_RL          
                #     break                                                     
                if send.Label_Model[320*k+165] == 32 :         
                    self.distance_RED_RL = 230 - k
                    self.distance_RED[2] = self.distance_RED_RL
                    break

            for p in range(230,0,-1):
                # if send.Label_Model[320*p+115] ==  4:                       
                #     self.distance_BLUE_RR = 230 - p
                #     self.distance_BLUE[3] = self.distance_BLUE_RR          
                #     break
                if send.Label_Model[320*p+200] == 32 :        
                    self.distance_RED_RR = 230 - p
                    self.distance_RED[3] = self.distance_RED_RR
                    break

            # 邊界測距
            for b1 in range(230,0,-1):
                if send.Label_Model[320*b1+15] == 32 :                       #左線與板子的距離
                    self.distance_Boundary_Left = 230 - b1
                    self.distance_Boundary[0] = self.distance_Boundary_Left
                    break

            for b2 in range(230,0,-1):
                if send.Label_Model[320*b2+300] == 32 :                      #左線與板子的距離
                    self.distance_Boundary_Right = 230 - b2
                    self.distance_Boundary[1] = self.distance_Boundary_Right
                    break
    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            time.sleep(0.5)
            print(' //////////////////')                                                              #作為分隔線
            print(' [Rll ,Rlr ,Rrl ,Rrr] :',self.distance_RED,end = '\n\n ')      #確認 ll lr rl rr的值
            print(' [Bll ,Blr ,Brl ,Brr] :',self.distance_BLUE,end = '\n\n ')      #確認 ll lr rl rr的值
            print(' [LB , RB] :',self.distance_Boundary, end='\n\n')                     #確認邊界的值
    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#爬梯判斷，此時機器人停下，且應該大致與梯子平行，故只以 rl 一線作為基準測量
    def Find_ladder_gap(self):
        self.distance_RED_RL = 0
        self.Ladder_W = 0                   # 梯子與梯子間距
        self.Ladder_L = 0                    # 梯子本身的寬度
        self.a=0
        self.b=0
        self.c=0

        for self.a in range(230,0,-1):                                                                   #從膝蓋往前偵測梯(1)前端（偵測紅色）
            if send.Label_Model[320*self.a+200] == 32 :
                self.distance_RED_RL = 230 - self.a
                break
                        
        for self.b in range(self.a,0,-1):                                                                   #從膝蓋往前偵測梯(1)末端（在偵測到紅色後，再偵測綠色）
            if send.Label_Model[320*self.b+200] == 8 :
                self.Ladder_L = self.a - self.b
                break

        for self.c in range(self.b,0,-1):                                                                   #從膝蓋往前偵測梯(2)前端（在偵測到紅色後，再偵測綠色，最後在偵測紅色）
            if send.Label_Model[320*self.c+200] == 32 :
                self.Ladder_W = self.b - self.c
                break
            
        self.Ladder_L=self.a - self.b
        self.Ladder_W=self.b - self.c
        
    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        time.sleep(0.5)
        print(' //////////////////')                                                              #作為分隔線
        print(' [L,W]: ', self.Ladder_L,self.Ladder_W,end = '\n\n ')
    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////