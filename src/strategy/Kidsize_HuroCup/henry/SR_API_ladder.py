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

send = Sendmessage()#要放在class外面,不然不能宣告

class Climb_ladder():
    def __init__(self):#初始化

        self.eyeline_y=120        #眼睛基準線（y值)
        self.eyeline_x=135          #眼睛左線
        self.init_lad_dis=20
        #機器人高
        self.robot_high=47
        self.robot_l1=2
        self.robot_l2=3
        self.robot_l3=4
        self.robot_l4=6
        self.robot_l5=1
        self.robot_lcam=2



        
        self.ladder_n=5         #樓梯數
        self.read_ladder_p=0

        self.read_ladder_f=0

        self.ladder_dis=[999,0]

        # self.ladder_in_screen=send.color_mask_subject_cnts[self.node_color] #在畫面中的node數
        self.head_theta=[0 for i in range(self.ladder_n)]                   #0數量要等於階數
        self.head_360=[0 for i in range(self.ladder_n)]
        self.ladder_hight=[0 for i in range(self.ladder_n)]

        self.head_init=1300
        self.head_now=self.head_init

    def ladder_distance(self):
        for u in range(self.eyeline_y,0,-1):
            if send.Label_Model[320*u+self.eyeline_x] == 32:
                self.ladder_dis[0] = self.eyeline_y-u
                break
        for d in range(self.eyeline_y,240):
            if send.Label_Model[320*d+self.eyeline_x] == 32:
                self.ladder_dis[1] = d-self.eyeline_y
                break
        print('ladder distance func')


        
    def find_up_ladder(self):
        print('find up ladder func')
        self.ladder_distance()
        # self.middle_point_y=send.color_mask_subject_YMin[2][self.ladder_in_screen-1]
        if self.read_ladder_p == self.ladder_n-1 :
            if self.ladder_dis[0]==0 and self.ladder_dis[1]==0 and self.read_ladder_f ==1:
                self.head_theta[self.read_ladder_p]=self.head_now
                self.head_360[self.read_ladder_p]=(self.head_theta[self.read_ladder_p]-2024)/4096*360
                self.read_ladder_f=0
                self.read_ladder_p+=1
                self.print_state()

        if self.ladder_dis[0]==0 and self.read_ladder_f==0:
            self.read_ladder_f=1
            self.print_state()
            
        elif self.ladder_dis[0]==0 and self.read_ladder_f==1:
            self.head_theta[self.read_ladder_p]=self.head_now
            self.head_360[self.read_ladder_p]=(self.head_theta[self.read_ladder_p]-2024)/4096*360
            self.print_state()

        elif self.ladder_dis[0] > 0 and self.read_ladder_f==1:
            self.read_ladder_f = 0
            self.read_ladder_p+=1
            self.print_state()

    def rise_head(self):
        if self.head_now < 2400:
            self.head_now+=1
            send.sendHeadMotor(2,self.head_now,100)
            time.sleep(0.01)

    def find_ladder_hight(self):
        n = symbols('n')
        for i in self.head_360:
            # print(i)
            sin_theta=sin(math.radians(i))
            tri_a=self.init_lad_dis+n
            tri_b=n-self.robot_high
            tri_c=sqrt((tri_a**2)+(tri_b**2))
            eq =(sin_theta)-(tri_b/tri_c)
            # print(eq)
            print(solve(eq,n))
    
    def robot_high_cal(self):
        self.robot_high=47
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

        # self.robot_high = a+b+c+(d*cos(f))



    def print_state(self):
        print("////////////////////////////////////////")
        print("ladder_pointer: ",self.read_ladder_p)
        print("ladder_flag   : ",self.read_ladder_f)
        print("next_distance : ",self.ladder_dis)
        print("head now      : ",self.head_now)
        print("head_theta    : ",self.head_theta)
        print("head_360      : ",self.head_360)
        print("ladder_hight  : ",self.ladder_hight)