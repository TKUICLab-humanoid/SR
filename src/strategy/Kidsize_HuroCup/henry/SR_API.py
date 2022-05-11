#!/usr/bin/env python
#coding=utf-8
# from sys import orig_argv
from curses import BUTTON1_DOUBLE_CLICKED
from dis import dis
from faulthandler import disable
from functools import lru_cache
from re import S
from resource import RLIMIT_FSIZE
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
import math
from Python_API import Sendmessage

send = Sendmessage()                                                                    #要放在class外面，否則會宣告失效

#訂定〝傳送4筆距離資料，分別是ll, lr, rl, rr類別
class Send_distance():
    def __init__(self):     #初始化
#旗標
        self.Walk = False
        self.Progress = False

#loop數值
        self.Times = 0

#距離偵測值初始化
        self.lldistance = 999
        self.lrdistance = 999
        self.rldistance = 999
        self.rrdistance = 999
        self.distance = [999,999,999,999]

        self.b1distance = 999
        self.b2distance = 999
        self.bdistance = [999,999]

#距離值
        self.Du = 15                  #上板前停下的可接受誤差
        self.Iu = 15                    #上板緊急停下數值
        self.Id = 10                    #下板緊急停下數值
        self.Su = 40                  #上板停下數值
        self.Sd = 20                  #下板停下數值
        self.Bd = 50                 # b1 b2邊界臨界值
        self.value = 40            # 微調 凹凸面狀況判斷值
        
#sendBodyAuto
        self.B1x = 400                #用愛對待地板的值（直走x值）
        self.B2x = 6800             #把地板踩壞的值（上板x值）

#sendContinuousValue
        self.C1x = -100            # 微調、直走x值
        self.C2x = 400            # 大幅修正
        self.C3x = -200                 # 原地微調x值

        self.C1z = -8               # 一般右轉z值
        self.C2z = 2                # 一般左轉z值
        self.C3z = -6                # 直走z值

        self.C1y = -800         # 右踏y值
        self.C2y = 300          # 左踏y值
        self.C3y = -560               # 原地微調y值/直走/不踏步y值


        # self.Color == 32

#A校正機器人對板子的方位（上板）
    def revision(self):
        if self.Walk == True and self.Progress == True:
            if self.lldistance < self.lrdistance and self.rrdistance < self.rldistance:             #凹面判斷
                print (' 凹面判斷中...',end ='\n\n')
                if self.rldistance < self.lrdistance:
                    print(' 右腳在前，向右轉',end ='\n\n')
                    send.sendContinuousValue(self.C1x,self.C3y,0,self.C1z,0)  #self.C1x=200,self.C1z=左轉值
                    if self.lrdistance < self.Su and self.rldistance < self.Su:
                        print(' 機器人到達，準備微調',end = '\n\n')
                        self.Walk = False
                elif self.rldistance > self.lrdistance:
                    print(' 左腳在前，向左轉',end ='\n\n')
                    send.sendContinuousValue(self.C1x,self.C3y,0,self.C2z,0)
                    if self.lrdistance < self.Su and self.rldistance < self.Su:
                        print(' 機器人到達，準備微調',end = '\n\n')
                        self.Walk = False
                elif self.rldistance == self.lrdistance:
                    print(' 直走，不調整',end = '\n\n')
                    send.sendContinuousValue(self.C1x,self.C3y,0,self.C3z,0)   #C3z=直走值
                    if self.lldistance < self.Su and self.rrdistance < self.Su:
                        print(' 機器人到達，準備微調',end = '\n\n')                  
                        self.Walk = False

            elif self.lrdistance <self.lldistance and self.rldistance <= self.rrdistance:            #凸面判斷
                print(' 凸面判斷中...',end ='\n\n')               
                if self.lrdistance < self.Su and self.rldistance < self.Su:
                    print(' 機器人到達，準備微調',end = '\n\n')
                    self.Walk = False
                elif self.b1distance != 999 and self.b2distance ==999 or self.lrdistance - self.lldistance <= self.rldistance - self.rrdistance:
                    print(' 判斷上左板，向左踏步',end ='\n\n')
                    send.sendContinuousValue(self.C3x,self.C2y,0,self.C3z,0)
                    if self.lldistance <= self.Iu:
                        print(' 踏步緊急停下，準備微調',end = '\n\n')
                        self.Walk = False
                elif self.b1distance == 999 and self.b2distance !=999 or self.lrdistance - self.lldistance > self.rldistance - self.rrdistance:
                    print(' 判斷上右板，向右踏步',end ='\n\n')
                    send.sendContinuousValue(self.C3x,self.C1y,0,self.C3z,0)
                    if self.rrdistance <= self.Iu:
                        print(' 踏步緊急停下，準備微調',end = '\n\n')
                        self.Walk = False

            elif self.b1distance == 999 and self.b2distance != 999:
                if self.b2distance > self.Bd:
                    print(' 右轉，大幅度校正板子位置',end = '\n\n')
                    send.sendContinuousValue(self.C2x,self.C3y,0,self.C1z,0)
                elif self.b2distance < self.Bd  and self.lldistance > self.rrdistance:
                    print(' 右腳在前（邊），右轉微調',end = '\n\n')
                    send.sendContinuousValue(self.C1x,self.C3y,0,self.C1z,0)

            elif self.b1distance != 999 and self.b2distance == 999:
                if self.b1distance > self.Bd:
                    print(' 左轉，大幅度校正板子位置',end = '\n\n')
                    send.sendContinuousValue(self.C2x,self.C3y,0,self.C2z,0)
                elif self.b1distance < self.Bd and self.lldistance > self.rrdistance:
                    print(' 左腳在前（邊），左轉微調',end = '\n\n')
                    send.sendContinuousValue(self.C1x,self.C3y,0,self.C2z,0)

            elif self.lldistance !=999 and self.rrdistance == 999:                   #右腳沒偵測到板子，但左腳有，代表機器人需要左轉進行修正
                print(' 向左踏，校正板子位置',end = '\n\n')
                send.sendContinuousValue(self.C3x,self.C2y,0,self.C3z,0)
                if self.lldistance <= self.Iu:
                    print(' 踏步緊急停下，準備微調',end = '\n\n')
                    self.Walk = False

            elif self.lldistance == 999 and self.rrdistance != 999 :                  #左腳沒偵測到板子，但右腳有，代表機器人需要右轉進行修正
                print(' 向右踏，校正板子位置',end = '\n\n')
                send.sendContinuousValue(self.C3x,self.C1y,0,self.C3z,0)
                if self.rrdistance <= self.Iu:
                    print(' 踏步緊急停下，準備微調',end = '\n\n')
                    self.Walk = False

            elif self.lldistance < self.rrdistance:
                print(' 左腳在前，向左轉',end = '\n\n')
                send.sendContinuousValue(self.C1x,self.C3y,0,self.C2z,0)
                if self.lldistance < self.Su:
                    if self.rrdistance - self.lldistance < self.Du:
                        print(' 機器人到達，準備微調',end = '\n\n')
                        self.Walk = False
                    elif self.lldistance <= self.Iu:
                        print(' 緊急停下，準備微調',end = '\n\n')
                        self.Walk = False

            elif self.lldistance > self.rrdistance:
                print(' 右腳在前，向右轉',end = '\n\n') 
                send.sendContinuousValue(self.C1x,self.C3y,0,self.C1z,0)
                if self.rrdistance < self.Su:
                    if self.lldistance - self.rrdistance < self.Du:
                        print(' 機器人到達，準備微調',end = '\n\n')
                        self.Walk = False
                    elif self.rrdistance <= self.Iu:
                        print(' 緊急停下，準備微調',end = '\n\n')
                        self.Walk = False

            elif self.lldistance == self.rrdistance :
                print(' 直走，不調整',end = '\n\n ')
                send.sendContinuousValue(self.C1x,self.C3y,0,self.C3z,0)
                if self.lldistance < self.Su and self.rrdistance < self.Su:  #self.disvar=35 
                    print(' 機器人到達，準備微調',end = '\n\n')               
                    self.Walk = False      

#B校正機器人對板子方位（下板）
    def revision_down(self):
            if self.lldistance == 0 and self.rrdistance != 0 :                  
                print(' 左腳偏外，向右踏步',end = '\n\n ')
                send.sendContinuousValue(self.C3x,self.C1y,0,self.C3z,0)
                if self.rrdistance <= self.Id:
                    print(' 機器人到達，停下',end = '\n\n ')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(2)
                    self.Walk = False

            elif self.lldistance !=0 and self.rrdistance == 0:                
                print(' 右腳偏外，向左踏步',end = '\n\n')
                send.sendContinuousValue(self.C3x,self.C2y,0,self.C3z,0)
                if self.lldistance <= self.Id:
                    print(' 機器人到達，停下',end = '\n\n ')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(2)
                    self.Walk = False

            elif self.lrdistance > self.lldistance and self.rldistance > self.rrdistance:
                print(' 凸面狀況',end = '\n\n ')
                if self.lrdistance - self.lldistance > self.rldistance - self.rrdistance:
                    print('平面偏左，原地左轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C2z,0)
                else:
                    print('平面偏右，原地右轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C1z,0)

            elif self.lldistance < self.rrdistance:
                print(' 左腳在前，向左轉',end = '\n\n')
                send.sendContinuousValue(self.C1x,self.C3y,0,self.C2z,0)
                if self.lrdistance < self.Sd or self.rldistance < self.Sd : #self.Sd=20
                    print(' 機器人到達，停下',end = '\n\n ')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(2)
                    self.Walk = False

            elif self.lldistance > self.rrdistance:
                print(' 右腳在前，向右轉',end = '\n\n ') 
                send.sendContinuousValue(self.C1x,self.C3y,0,self.C1z,0)
                if self.lrdistance < self.Sd or self.rldistance < self.Sd :  #self.Sd=20
                    print(' 機器人到達，停下',end = '\n\n ')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(2)
                    self.Walk = False

            elif self.lldistance  == 0 or self.rrdistance == 0 or self.lldistance - self.rrdistance <= 10 or self.rrdistance - self.lldistance <= 10  :
                print(' 直走，不調整',end = '\n\n ')
                send.sendContinuousValue(self.C1x,self.C3y,0,self.C3z,0)
                if self.lrdistance < self.Sd or self.rldistance < self.Sd :   #self.Sd=20
                    print(' 機器人到達，停下',end = '\n\n ')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(2)
                    self.Walk = False

#C到板子前停下，原地校正位置
    def revision_stop(self):
        if self.Walk == False and self.Progress == True:
            if  self.lldistance < 10 and self.rrdistance >15:
                if self.Times <= 3:
                    print(' 左腳過於靠板，原地左轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C2z,0)
                elif self.Times > 3:
                    print('左腳過於靠外，原地左轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C2z,0)

            elif self.rrdistance < 10 and self.lldistance >15:
                if self.Times <= 3:
                    print(' 右腳過於靠板，原地右轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C1z,0)
                elif self.Times > 3:
                    print('右腳過於靠外，原地右轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C1z,0)

            elif self.lldistance > self.value and self.lrdistance > self.rldistance:    #value=40
                print(' 上板位在右，向右踏步並原地左轉',end = '\n\n ')
                send.sendContinuousValue(self.C3x,self.C1y,0,self.C3z,0)
                send.sendContinuousValue(self.C3x,self.C3y,0,self.C2z,0)

            elif self.rrdistance > self.value and self.lrdistance < self.rldistance:    #value=40
                print(' 上板位在左，向左踏步並原地右轉',end = '\n\n ')
                send.sendContinuousValue(self.C3x,self.C2y,0,self.C3z,0)
                send.sendContinuousValue(self.C3x,self.C3y,0,self.C1z,0)

            elif self.lldistance < self.rrdistance and self.lrdistance < self.rldistance:
                if self.rrdistance - self.lldistance > self.Du:   #self.Du=15
                    print(' 平面狀況，板子在左，原地左轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C2z,0)

            elif self.lldistance > self.rrdistance and self.lrdistance > self.rldistance:
                if self.lldistance - self.rrdistance > self.Du:   #self.Du=15
                    print(' 平面狀況，板子在右，原地右轉')
                    send.sendContinuousValue(self.C3x,self.C3y,0,self.C1z,0)

            elif self.rrdistance - self.lldistance < self.Du or self.lldistance - self.rrdistance < self.Du:
                if self.Times <=3:
                    print(' 機器人就位，準備上板')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(2)
                    send.sendBodySector(29)
                    time.sleep(2)
                    send.sendBodySector(2)
                    time.sleep(2)
                    send.sendBodyAuto(self.B2x,0,0,0,2,0)
                    time.sleep(4)
                    send.sendBodySector(29)
                    time.sleep(2)
                    self.Progress = False
                    self.Walk = False
                elif self.Times >3:
                    print(' 機器人就位，準備下板')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(2)
                    send.sendBodySector(29)
                    time.sleep(2)
                    send.sendBodySector(1)
                    time.sleep(2)
                    send.sendBodyAuto(self.B2x,0,0,0,3,0)
                    time.sleep(4)
                    send.sendBodySector(29)
                    time.sleep(2)
                    self.Progress = False
                    self.Walk = False

#測量並取得與紅板的距離
    def find_Rboard(self):
        self.lldistance = 999
        self.lrdistance = 999
        self.rldistance = 999
        self.rrdistance = 999
        self.distance = [999,999,999,999]

        self.b1distance = 999
        self.b2distance = 999
        self.bdistance = [999,999]

        for j in range(230,0,-1):        
            if send.Label_Model[320*j+115] == 32 :                       #左腳左點與板子的距離
                self.lldistance = 230 - j
                self.distance[0] = self.lldistance                                  #將lldistance放到self.distance的第0個位置
                break
       
        for i in range(230,0,-1):
            if send.Label_Model[320*i+150] == 32 :                       #左腳右點與板子的距離
                self.lrdistance = 230 - i
                self.distance[1] = self.lrdistance
                break

        for k in range(230,0,-1):                                                     
            if send.Label_Model[320*k+165] == 32 :                       #右腳左點與板子的距離
                self.rldistance = 230 - k
                self.distance[2] = self.rldistance
                break

        for p in range(230,0,-1):
            if send.Label_Model[320*p+200] == 32 :                       #右腳右點與板子的距離
                self.rrdistance = 230 - p
                self.distance[3] = self.rrdistance
                break
        
        for b1 in range(230,0,-1):
            if send.Label_Model[320*b1+15] == 32 :                       #左線與板子的距離
                self.b1distance = 230 - b1
                self.bdistance[0] = self.b1distance
                break

        for b2 in range(230,0,-1):
            if send.Label_Model[320*b2+300] == 32 :                      #左線與板子的距離
                self.b2distance = 230 - b2
                self.bdistance[1] = self.b2distance
                break

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        time.sleep(0.5)
        print(' 紅板偵測中...',self.Times)
        print(' //////////////////')                                                              #分隔線
        print(' [ll ,lr ,rl ,rr] :',self.distance)                                #確認 ll lr rl rr的值
        print(' [LB , RB] :',self.bdistance, end='\n\n')                     #確認邊界的值
#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#測量並取得與黃板的距離
    def find_Yboard(self):
        self.lldistance = 999
        self.lrdistance = 999
        self.rldistance = 999
        self.rrdistance = 999
        self.distance = [999,999,999,999]

        self.b1distance = 999
        self.b2distance = 999
        self.bdistance = [999,999]

        for j in range(230,0,-1):        
            if send.Label_Model[320*j+115] == 2 :                       #左腳左點與板子的距離
                self.lldistance = 230 - j
                self.distance[0] = self.lldistance                                  #將lldistance放到self.distance的第0個位置
                break
    
        for i in range(230,0,-1):
            if send.Label_Model[320*i+150] == 2 :                       #左腳右點與板子的距離
                self.lrdistance = 230 - i
                self.distance[1] = self.lrdistance
                break

        for k in range(230,0,-1):                                                     
            if send.Label_Model[320*k+165] == 2 :                       #右腳左點與板子的距離
                self.rldistance = 230 - k
                self.distance[2] = self.rldistance
                break

        for p in range(230,0,-1):
            if send.Label_Model[320*p+200] == 2 :                       #右腳右點與板子的距離
                self.rrdistance = 230 - p
                self.distance[3] = self.rrdistance
                break
        
        for b1 in range(230,0,-1):
            if send.Label_Model[320*b1+15] == 2 :                       #左線與板子的距離
                self.b1distance = 230 - b1
                self.bdistance[0] = self.b1distance
                break

        for b2 in range(230,0,-1):
            if send.Label_Model[320*b2+300] == 2 :                      #左線與板子的距離
                self.b2distance = 230 - b2
                self.bdistance[1] = self.b2distance
                break

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        time.sleep(0.5)
        print(' 黃板偵測中...',self.Times)
        print(' //////////////////')                                                              #分隔線
        print(' [ll ,lr ,rl ,rr] :',self.distance)                                #確認 ll lr rl rr的值
        print(' [LB , RB] :',self.bdistance, end='\n\n')                     #確認邊界的值
#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#測量並取得與藍板的距離
    def find_Bboard(self):
        self.lldistance = 999
        self.lrdistance = 999
        self.rldistance = 999
        self.rrdistance = 999
        self.distance = [999,999,999,999]

        self.b1distance = 999
        self.b2distance = 999
        self.bdistance = [999,999]

        for j in range(230,0,-1):        
            if send.Label_Model[320*j+115] == 4 :                       #左腳左點與板子的距離
                self.lldistance = 230 - j
                self.distance[0] = self.lldistance                                  #將lldistance放到self.distance的第0個位置
                break
    
        for i in range(230,0,-1):
            if send.Label_Model[320*i+150] == 4 :                       #左腳右點與板子的距離
                self.lrdistance = 230 - i
                self.distance[1] = self.lrdistance
                break

        for k in range(230,0,-1):                                                     
            if send.Label_Model[320*k+165] == 4 :                       #右腳左點與板子的距離
                self.rldistance = 230 - k
                self.distance[2] = self.rldistance
                break

        for p in range(230,0,-1):
            if send.Label_Model[320*p+200] == 4 :                       #右腳右點與板子的距離
                self.rrdistance = 230 - p
                self.distance[3] = self.rrdistance
                break
        
        for b1 in range(230,0,-1):
            if send.Label_Model[320*b1+15] == 4 :                       #左線與板子的距離
                self.b1distance = 230 - b1
                self.bdistance[0] = self.b1distance
                break

        for b2 in range(230,0,-1):
            if send.Label_Model[320*b2+300] == 4 :                      #左線與板子的距離
                self.b2distance = 230 - b2
                self.bdistance[1] = self.b2distance
                break

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        time.sleep(0.5)
        print(' 藍板偵測中...',self.Times)
        print(' //////////////////')                                                              #分隔線
        print(' [ll ,lr ,rl ,rr] :',self.distance)                                #確認 ll lr rl rr的值
        print(' [LB , RB] :',self.bdistance, end='\n\n')                     #確認邊界的值
#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#測量並取得與綠板的距離
    def find_Gboard(self):
        self.lldistance = 999
        self.lrdistance = 999
        self.rldistance = 999
        self.rrdistance = 999
        self.distance = [999,999,999,999]

        self.b1distance = 999
        self.b2distance = 999
        self.bdistance = [999,999]

        for j in range(230,0,-1):        
            if send.Label_Model[320*j+115] == 8 :                       #左腳左點與板子的距離
                self.lldistance = 230 - j
                self.distance[0] = self.lldistance                                  #將lldistance放到self.distance的第0個位置
                break
    
        for i in range(230,0,-1):
            if send.Label_Model[320*i+150] == 8 :                       #左腳右點與板子的距離
                self.lrdistance = 230 - i
                self.distance[1] = self.lrdistance
                break

        for k in range(230,0,-1):                                                     
            if send.Label_Model[320*k+165] == 8 :                       #右腳左點與板子的距離
                self.rldistance = 230 - k
                self.distance[2] = self.rldistance
                break

        for p in range(230,0,-1):
            if send.Label_Model[320*p+200] == 8 :                       #右腳右點與板子的距離
                self.rrdistance = 230 - p
                self.distance[3] = self.rrdistance
                break
        
        for b1 in range(230,0,-1):
            if send.Label_Model[320*b1+15] == 8 :                       #左線與板子的距離
                self.b1distance = 230 - b1
                self.bdistance[0] = self.b1distance
                break

        for b2 in range(230,0,-1):
            if send.Label_Model[320*b2+300] == 8 :                      #左線與板子的距離
                self.b2distance = 230 - b2
                self.bdistance[1] = self.b2distance
                break

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        time.sleep(0.5)
        print(' 綠板偵測中...',self.Times)
        print(' //////////////////')                                                              #分隔線
        print(' [ll ,lr ,rl ,rr] :',self.distance)                                #確認 ll lr rl rr的值
        print(' [LB , RB] :',self.bdistance, end='\n\n')                     #確認邊界的值
#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////