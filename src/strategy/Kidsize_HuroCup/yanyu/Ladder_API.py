#!/usr/bin/env python
#coding=utf-8
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

send = Sendmessage()#要放在class外面,不然不能宣告

class Send_Climb():
    def __init__(self):#初始話
        # self.up_flag = 0
        # self.speed_flag = 0
        # self.a = 0
        # self.b = 0

        #腳掌標準線x值
        self.knee=210
        self.f_ll=98
        self.f_lr=150
        self.f_rl=170
        self.f_rr=222

        #距離矩陣初始化
        self.climb_distance = [999,999,999,999]

        self.layer = [32]

        #旗標初始化
        self.stop_flag = 1      # 1表示停止
        self.up_ladder_flag =0
#//////////////////////////////////////////////////////////
        #校正變數
        self.c_theta=3
        
        self.c_speed=-100
        # 下板校正的平移
        self.c_yspeed =-100
#///////////////////////////////////////////////////////////////
        #角度速度初始化
        self.theta = 0 + self.c_theta
        self.speed = 500 + self.c_speed
        self.yspeed =0 + self.c_yspeed

        self.up_1 = 40
        self.up_2 = 75

        self.speed_1 = 200 + self.c_speed
        self.speed_2 = 600 + self.c_speed

        #角度設定 左旋
        self.l_theta_1 = 1 + self.c_theta
        self.l_theta_2 = 2 + self.c_theta
        self.l_theta_3 = 3 + self.c_theta
        self.l_theta_4 = 4 + self.c_theta
        self.l_theta_5 = 5 + self.c_theta
        #角度設定 右旋
        self.r_theta_1 = -1 + self.c_theta
        self.r_theta_2 = -2 + self.c_theta
        self.r_theta_3 = -3 + self.c_theta
        self.r_theta_4 = -4 + self.c_theta
        self.r_theta_5 = -5 + self.c_theta

        #旋轉差
        self.feet_distance_1=5
        self.feet_distance_2=6      
        self.feet_distance_3=8
        self.feet_distance_4=10

        #3-0
        self.climb_feet_distance = 0

    # def find_ladder_mid(self):
    #     self.a = send.color_mask_subject_Y[2][0]
    #     self.b = send.color_mask_subject_Y[2][1]
    #     print('1',self.a)
    #     print(self.b)
    #     if self.a > 143 and self.up_flag ==0:
    #         print('PPPPPPPPPPPPP')
    #         send.sendBodyAuto(0,0,0,0,1,0)
    #         self.up_flag = 1

    # def ladder_mid_setup(self):#調整角度

    def find_ladder(self):
        #左左腳距離 x=115
        for ll in range(self.knee,5,-1):
            if send.Label_Model[320*ll+self.f_ll] == self.layer[0] and send.Label_Model[320*(ll-5)+self.f_ll] == self.layer[0]:
                self.climb_distance[0] = self.knee - ll
                break
        #左右腳距離 x=150
        for lr in range(self.knee,5,-1):
            if send.Label_Model[320*ll+self.f_ll] == self.layer[0] and send.Label_Model[320*(lr-5)+self.f_lr] == self.layer[0]:
                self.climb_distance[1] = self.knee - lr
                break
        #右左腳距離 x=165
        for rl in range(self.knee,5,-1):
            if send.Label_Model[320*ll+self.f_ll] == self.layer[0] and send.Label_Model[320*(rl-5)+self.f_rl] == self.layer[0]:
                self.climb_distance[2] = self.knee - rl
                break
        #右右腳距離 x=200
        for rr in range(self.knee,5,-1):
            if send.Label_Model[320*ll+self.f_ll] == self.layer[0] and send.Label_Model[320*(rr-5)+self.f_rr] == self.layer[0]:
                self.climb_distance[3] = self.knee - rr
                break



    def parallel_setup(self):

        if(self.climb_distance[0]<=self.up_2 and self.climb_distance[3]<=self.up_2):#30
            self.speed=self.speed_1
            self.yspeed = self.c_yspeed
            self.theta_func()
        else:
            self.speed=self.speed_2
            self.yspeed = self.c_yspeed
            self.theta_func()


    
    def up_ladder(self): #要上梯了
        print('climb_distance',self.climb_distance)
    
        if ((self.climb_distance[0]<=self.up_1) or (self.climb_distance[1]<=self.up_1) or (self.climb_distance[2]<=self.up_1) or (self.climb_distance[3]<=self.up_1)):
            if self.stop_flag==0 and self.up_ladder_flag==0:
                print('ready upladder')
                self.speed=0
                self.yspeed=0
                self.theta=0
                self.stop_flag=1
                self.up_ladder_flag=1
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(4)
                send.sendBodySector(15)
                time.sleep(30)
                send.sendBodySector(800)
                time.sleep(30)
                send.sendBodySector(600)
                time.sleep(30)
                self.climb_distance = [999,999,999,999]                 
        else:
            self.parallel_setup()
            send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)

    def theta_func(self):
        self.climb_feet_distance=self.climb_distance[3]-self.climb_distance[0]
        
        if(self.climb_feet_distance>self.feet_distance_1):
            print('turn left')
        elif (self.climb_feet_distance<(-1*self.feet_distance_1)):
            print('turn right')
        else:
            print('walk forward')

        if(self.climb_feet_distance)<(-1*self.feet_distance_4):#右旋
            self.theta = self.r_theta_4
        elif(self.climb_feet_distance)<(-1*self.feet_distance_3):
            self.theta = self.r_theta_3
        elif(self.climb_feet_distance)<(-1*self.feet_distance_2):
            self.theta = self.r_theta_2
        elif(self.climb_feet_distance)<(-1*self.feet_distance_1):
            self.theta =  self.r_theta_1

        elif(self.climb_feet_distance)>self.feet_distance_4:#左旋
            self.theta =  self.l_theta_4
        elif(self.climb_feet_distance)>self.feet_distance_3:
            self.theta = self.l_theta_3
        elif(self.climb_feet_distance)>self.feet_distance_2:
            self.theta = self.l_theta_2
        elif(self.climb_feet_distance)>self.feet_distance_1:
            self.theta = self.l_theta_1
        else:
            self.theta = 0+self.c_theta

    

