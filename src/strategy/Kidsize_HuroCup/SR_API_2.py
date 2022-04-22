#!/usr/bin/env python
#coding=utf-8
# from sys import orig_argv
from base64 import b16decode
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

#訂定〝傳送4筆距離資料，分別是ll, lr, rl, rr類別
class Ladder_send_distance():
    def __init__(self):
#旗標
        self.Walk = False
        self.Progress = False

#loop數值
        self.Times = 0

#距離偵測值初始化
        self.lldistance = 0
        self.lrdistance = 0
        self.rldistance = 0
        self.rrdistance = 0
        self.distance = [0,0,0,0]

        self.b1distance = 999
        self.b2distance = 999
        self.bdistance = [999,999]

#sendBodyAuto
        self.B1x = 500                #停下x值

#sendContinuousValue
        self.C1x = 0            # 微調、直走x值
        self.C2x = -400                 # 原地微調/不走x值

        self.C1z = -4               # 一般右轉z值
        self.C2z = 4                # 一般左轉z值
        self.C3z = 1                # 直走z值
        self.C4z = -2               # 原地右轉z值
        self.C5z = 2                # 原地左轉z值

        self.C1y = -900         # 右踏y值
        self.C2y = 300          # 左踏y值
        self.C3y = -300              # 原地微調/不踏步y值
        
        self.Su = 20              #上梯停下數值
        self.Du = 15                  #上板前停下的可接受誤差
    def Ladder_revision(self):
        # if self.b1distance != 999 or self.lldistance == 999:
        #         print(' 機器人偏右，向左踏步',end = '\n\n ')
        #         send.sendContinuousValue(self.C2x,self.C2y,0,self.C3z,0)
        # elif self.b2distance !=999 or self.rrdistance == 999:
        #         print(' 機器人偏左，向右踏步',end = '\n\n ')
        #         send.sendContinuousValue(self.C2x,self.C1y,0,self.C3z,0)

        if self.lldistance < self.rrdistance:
            print(' 左腳在前，向左轉',end = '\n\n ')
            send.sendContinuousValue(self.C1x,self.C3y,0,self.C2z,0)
            if self.lldistance < self.Su or self.rrdistance < self.Su:
                print(' 機器人到達，原地左轉',end = '\n\n ')
                send.sendContinuousValue(self.C2x,self.C3y,0,self.C5z,0)
                if self.rrdistance - self.lldistance < self.Du:
                    print(' 機器人到達，停下',end = '\n\n ')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(1)
                    self.Walk = False

        elif self.lldistance > self.rrdistance:
            print(' 右腳在前，向右轉',end = '\n\n ') 
            send.sendContinuousValue(self.C1x,self.C3y,0,self.C1z,0)
            if self.lldistance < self.Su or self.rrdistance < self.Su:
                print(' 機器人到達，原地右轉',end = '\n\n ')
                send.sendContinuousValue(self.C2x,self.C3y,0,self.C4z,0)
                if self.lldistance - self.rrdistance < self.Du:
                    print(' 機器人到達，停下',end = '\n\n ')
                    send.sendBodyAuto(self.B1x,0,0,0,1,0)
                    time.sleep(1)
                    self.Walk = False

        elif self.lldistance == self.rrdistance:
            print(' 直走，不調整',end = '\n\n ')
            send.sendContinuousValue(self.C1x,self.C3y,0,self.C3z,0)
            if self.lldistance < self.Su or self.rrdistance < self.Su:        
                print(' 機器人到達，停下',end = '\n\n ')
                send.sendBodyAuto(self.B1x,0,0,0,1,0)
                time.sleep(1)
                self.Walk = False

#上梯子磁區呼叫
    def Up_ladder(self):
        if self.Walf == False and self.Progress == True:
            pass
        pass

#/////////////////////////////////////////////////////////////////////////////////////
    def Find_ladder(self):
            self.lldistance = 999
            self.rrdistance = 999
            self.distance = [999,999,999,999]

            self.b1distance = 999
            self.b2distance = 999
            self.bdistance = [999,999]

            for j in range(230,0,-1):        
                if send.Label_Model[320*j+115] == 32 :                       
                    self.lldistance = 230 - j
                    self.distance[0] = self.lldistance          
                    break
        
            for i in range(230,0,-1):
                if send.Label_Model[320*i+150] == 32 :              
                    self.lrdistance = 230 - i
                    self.distance[1] = self.lrdistance
                    break

            for k in range(230,0,-1):                                                     
                if send.Label_Model[320*k+165] == 32 :         
                    self.rldistance = 230 - k
                    self.distance[2] = self.rldistance
                    break

            for p in range(230,0,-1):
                if send.Label_Model[320*p+200] == 32 :        
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
            print(' //////////////////')                                                              #作為分隔線
            print(' [Rll ,Rlr ,Rrl ,Rrr] :',self.distance,end = '\n\n ')      #確認 ll lr rl rr的值
            print(' [LB , RB] :',self.bdistance, end='\n\n')                     #確認邊界的值
    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#爬梯判斷，此時機器人停下，且應該大致與梯子平行，故只以 rl 一線作為基準測量
    def Find_ladder_gap(self):
        self.rldistance = 0
        self.Ladder_W = 0
        self.Ladder_L = 0
        self.a=0
        self.b=0
        self.c=0

        for self.a in range(230,0,-1):                                                                   #從膝蓋往前偵測梯(1)前端（偵測紅色）
            if send.Label_Model[320*self.a+200] == 32 :
                self.rldistance = 230 - self.a
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