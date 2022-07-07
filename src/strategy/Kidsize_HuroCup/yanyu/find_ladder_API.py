#!/usr/bin/env python
#coding=utf-8
from operator import eq
from symtable import Symbol

from cv2 import solve
import sympy
import rospy
import numpy as np
import math
from rospy import Publisher
from sympy import *
from tku_msgs.msg import Interface,HeadPackage,SandHandSpeed,DrawImage,SingleMotorData,\
SensorSet,ObjectList,LabelModelObjectList,RobotPos,SetGoalPoint,SoccerDataList,SensorPackage
from std_msgs.msg import Int16,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from Python_API import Sendmessage
from SR_API import Send_distance

send = Sendmessage()#要放在class外面,不然不能宣告
distance = Send_distance()

class Find_ladder():
    def __init__(self):#初始化

        self.eyeline_y=120        #眼睛基準線（y值)
        self.eyeline_x=100        #眼睛左線
        self.init_robot_distance=20
        self.find_ladder_flag = 1
        
        #機器人高
        self.robot_high=43
        self.robot_l1=2
        self.robot_l2=3
        self.robot_l3=4
        self.robot_l4=6
        self.robot_l5=1
        self.robot_lcam=2
        
        self.ladder_n=5         #樓梯數
        self.read_ladder_p=0    #用來控制存在array的第幾個index

        self.read_ladder_f=0

        self.ladder_dis=[999,999]

        self.head_theta=[0 for i in range(self.ladder_n)]                   #0數量要等於階數
        self.head_360=[0 for i in range(self.ladder_n)]
        self.ladder_hight=[0 for i in range(self.ladder_n)]

        self.head_highest = 2460
        self.head_lowest  = 1200

        self.head_init = self.head_highest
        self.head_now  = self.head_init

    #算機器人高
    def robot_high_cal(self):
        self.robot_high=42
        self.robot_mid_theta = 2239
        self.robot_l1=2
        self.robot_l2=3
        self.robot_l3=4
        self.robot_l4=6
        self.robot_l5=1
        self.robot_lcam=2

        theta_1=30
        theta_2=30
        theta_3=30
        theta_v=30

        a=self.robot_l1
        b=self.robot_l2*cos(math.radians(theta_1))
        c=self.robot_l3*cos(math.radians(theta_2-theta_1))
        d=sqrt((self.robot_l4**2+self.robot_l5**2))
        e=atan(self.robot_l5/self.robot_l4)
        f=theta_3+e-theta_2+theta_1

        self.robot_high = a+b+c+(d*cos(f))


    def ladder_distance(self):

        #從畫面的y基準線 往下掃
        for d in range(self.eyeline_y,240):
            if send.Label_Model[320*d+self.eyeline_x] == 32:
                self.ladder_dis[0] = d-self.eyeline_y
                break
            else:
                self.ladder_dis[0] = 999
                break


        #從畫面的y基準線 往上掃
        for u in range(self.eyeline_y,0,-1):
            if send.Label_Model[320*u+self.eyeline_x] == 32:
                self.ladder_dis[1] = self.eyeline_y-u
                break
            else:
                self.ladder_dis[1] = 999
                break
        
        print('ladder distance func')


    def find_ladder_theta(self):
        print('find ladder theta func')
        self.ladder_distance()
        #開始偵測到樓梯
        if self.ladder_dis[0]==0 and self.read_ladder_f==0:
            self.read_ladder_f=1
            
        #在樓梯竿中間
        elif self.ladder_dis[0]==0 and self.read_ladder_f==1:
            self.head_theta[self.read_ladder_p]=self.head_now
            self.head_360[self.read_ladder_p]=(self.head_theta[self.read_ladder_p]-2238)/4096*360
            
        #樓梯結束
        elif self.ladder_dis[0] > 0 and self.read_ladder_f==1:
            #紀錄最後一格
            if self.read_ladder_p == self.ladder_n-1 and (self.ladder_dis[1]==0 and self.ladder_dis[0]==0):
                self.head_theta[self.read_ladder_p]=self.head_now
                self.head_360[self.read_ladder_p]=(self.head_theta[self.read_ladder_p]-2238)/4096*360
                self.head_theta.sort()
                self.head_360.sort()
                self.read_ladder_f=0
                self.read_ladder_p+=1    
            else:
                self.read_ladder_f = 0
                self.read_ladder_p+=1

        

    def rise_head(self):
        if self.head_now <= self.head_highest:
            self.head_now+=5
            send.sendHeadMotor(2,self.head_now,100)#垂直
            print("rise head")
            #time.sleep(0.01)

    def down_head(self):
        if self.head_now >= self.head_lowest:
            self.head_now-=7
            send.sendHeadMotor(2,self.head_now,100)
            print("down head")
            #time.sleep(0.01)

    def calculate_ladder_hight(self):
        n = symbols('n')
        for j in range(5):
            i=self.head_360[j]
            sin_theta=sin(math.radians(i))
            tri_a=self.init_robot_distance+n
            tri_b=n-self.robot_high
            tri_c=sqrt((tri_a**2)+(tri_b**2))
            eq =(sin_theta)-(tri_b/tri_c)
            # print(eq)
            ans = solve(eq,n)
            print(ans)
            self.ladder_hight[j] = ans
    
    
            

    def print_state(self):
        print("////////////////////////////////////////")
        print("robot high    :" ,self.robot_high)
        print("ladder_pointer: ",self.read_ladder_p)
        print("ladder_flag   : ",self.read_ladder_f)
        print("fing_flag     : ",self.find_ladder_flag)
        print("next_distance : ",self.ladder_dis)
        print("head now      : ",self.head_now)
        print("head_theta    : ",self.head_theta)
        print("head_360      : ",self.head_360)
        print("ladder_hight  : ",self.ladder_hight)