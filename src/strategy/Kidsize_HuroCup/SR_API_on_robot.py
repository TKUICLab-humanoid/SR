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

class Send_distance():
    def __init__(self):#初始化
        #腳掌標準線x值
        self.f_ll=98
        self.f_lr=150
        self.f_rl=165
        self.f_rr=215
        #距離矩陣初始化
        self.up_distance = [999,999,999,999]        #要上的層
        self.down_distance = [0,0,0,0]      #要下的層
        self.next_distance = [999,999,999,999]      #要下下的層
        #色模
        self.color_model=[3,5,1,2]       
        self.point_x=0    #色模Ymax的x值
        self.point_y=0    #色模Ymax
        self.m_xmin=0
        self.m_xmax=0
        self.m_l_d=0
        self.m_r_d=0
        #角度速度初始化
        self.theta = 0
        self.speed = 500
        self.yspeed = 0
        #旗標初始化
        self.stop_flag = 1      # 1表示停止
        self.up_board_flag =0
        #第幾層
        self.layer_n= 1        #現在站的層,從1開始
        self.layer = [8,32,2,4]     #用在labelMode
        self.direction = 0      #0 上板 1 下板

        #校正變數
        self.rc_theta=2
        self.lc_theta=2


        #角度設定 左旋
        self.l_theta_1 = 1 + self.lc_theta
        self.l_theta_2 = 2 + self.lc_theta
        self.l_theta_3 = 3 + self.lc_theta
        self.l_theta_4 = 4 + self.lc_theta
        self.l_theta_5 = 5 + self.lc_theta
        #角度設定 右旋
        self.r_theta_1 = -1 + self.rc_theta
        self.r_theta_2 = -2 + self.rc_theta
        self.r_theta_3 = -3 + self.rc_theta
        self.r_theta_4 = -4 + self.rc_theta
        self.r_theta_5 = -5 + self.rc_theta
        #上板速度
        self.up_b_speed=8300
        self.speed_1=100
        self.speed_2=200
        self.speed_3=300
        self.speed_4=400
        self.speed_5=500
        #下板速度
        self.down_speed_1=100
        self.down_speed_2=400
        self.down_speed_3=500
        #上板補償的平移
        self.c_up_yspeed = 0
        #下板補償的平移
        self.c_down_yspeed = 0
        #上板腳離板子差
        self.up_bd_1=13
        self.up_bd_2=30
        self.up_bd_3=60
        self.up_bd_4=100
        #下板腳離板子差
        self.down_bd_1=10
        self.down_bd_2=30
        self.down_bd_3=50
        self.down_bd_4=60
        

        #腳前距離差
        self.feet_distance_1=2
        self.feet_distance_2=5      
        self.feet_distance_3=8
        self.feet_distance_4=10

        #上板3-0
        self.up_feet_distance = 0
        #下板3-0
        self.down_feet_distance = 0

        #距離不夠的速度角度
        self.no_space_speed = 0
        self.no_space_yspeed = 0
        self.no_space_theta = 0


        
    def find_up_board(self):
        print('find up board func')
        self.point_y=send.color_mask_subject_YMax[self.color_model[self.layer_n]][0]
        #找色模（下一層）Ymax點的X座標
        for mp in range (0,320):
            if send.Label_Model[320*self.point_y+mp] == self.layer[self.layer_n]:
                self.point_x=mp
                break
        #左左腳距離 x=115
        for ll in range(230,0,-1):
            if send.Label_Model[320*ll+self.f_ll] == self.layer[self.layer_n]:
                self.up_distance[0] = 230 - ll
                break
        #左右腳距離 x=150
        for lr in range(230,0,-1):
            if send.Label_Model[320*lr+self.f_lr] == self.layer[self.layer_n]:
                self.up_distance[1] = 230 - lr
                break
        #右左腳距離 x=165
        for rl in range(230,0,-1):
            if send.Label_Model[320*rl+self.f_rl] == self.layer[self.layer_n]:
                self.up_distance[2] = 230 - rl
                break
        #右右腳距離 x=200
        for rr in range(230,0,-1):
            if send.Label_Model[320*rr+self.f_rr] == self.layer[self.layer_n]:
                self.up_distance[3] = 230 - rr
                break

    def find_down_board(self):
        print('find down board func')
        #self.print_state()
        self.point_y=send.color_mask_subject_YMin[self.color_model[self.layer_n]][0]
        #找色模(自己層）Ymin點的X座標
        for mp in range (0,320):
            if send.Label_Model[320*self.point_y+mp] == self.layer[self.layer_n]:
                self.point_x=mp
                break
        #左左腳距離
        for ll in range(230,0,-1):      #找要下的層(本身層邊界)
            if send.Label_Model[320*ll+self.f_ll] == self.layer[self.layer_n-1]:
                self.down_distance[0] = 230 - ll
                break
        #左右腳距離
        for lr in range(230,0,-1):
            if send.Label_Model[320*lr+self.f_lr] ==  self.layer[self.layer_n -1]:
                self.down_distance[1] = 230 - lr
                break

        #右左腳距離
        for rl in range(230,0,-1):
            if send.Label_Model[320*rl+self.f_rl] ==  self.layer[self.layer_n-1]:
                self.down_distance[2] = 230 - rl
                break
        #右右腳距離
        for rr in range(230,0,-1):
            if send.Label_Model[320*rr+self.f_rr] == self.layer[self.layer_n-1]:
                self.down_distance[3] = 230 - rr
                break
        
        if self.layer_n>=2:
            for ll_2 in range(ll,0,-1):     #找下下一層
                # print('ll_22222222222222222222222222222222222k',ll_2)
                if send.Label_Model[320*ll_2+self.f_ll] == self.layer[self.layer_n-2]:
                    self.next_distance[0] = ll-ll_2
                    # print('ll = ',ll)
                    # print('ll_2 = ',ll_2)
                    # print('next_distance_qq = ',self.next_distance[0])
                    break
                # if send.Label_Model[320*ll_2+self.f_ll] != self.layer[self.layer_n - 1] and send.Label_Model[320*ll_2+self.f_ll] != self.layer[self.layer_n]:
                #     print('ll_22222222222222222222222222222222222k',ll_2)
                #     self.next_distance[0] = (230 - ll_2) - self.down_distance[0]
                #     break
            for lr_2 in range(230,0,-1):
                if send.Label_Model[320*lr_2+self.f_lr] == self.layer[self.layer_n-2]:
                    self.next_distance[1] = (230 - lr_2) - self.down_distance[1]
                    break
            
            for rl_2 in range(230,0,-1):
                if send.Label_Model[320*rl_2+self.f_rl] == self.layer[self.layer_n-2]:
                    self.next_distance[2] = (230 - rl_2) - self.down_distance[2]
                    break
            
            for rr_2 in range(230,0,-1):
                if send.Label_Model[320*rr_2+self.f_rr] == self.layer[self.layer_n-2]:
                    self.next_distance[3] = (230 - rr_2) - self.down_distance[3]
                    break
        else:
            self.next_distance=[999,999,999,999]

    def parallel_board_setup(self):#上板的角度調整
        print('parallel_board_setup func')
        if(self.up_distance[1]<=self.up_bd_2 or self.up_distance[2]<=self.up_bd_2):
            self.speed=self.speed_1
            self.yspeed = self.c_up_yspeed
            self.up_theta_func()
            
        #上板直角
        # elif((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (self.up_distance[0]>40 or self.up_distance[1]>40 or self.up_distance[2]>40 or self.up_distance[3]>40):
        #     self.up_board_90()

        elif (self.up_distance[0] > self.up_distance[1]) and (self.up_distance[0] > self.up_distance[2]) and (self.up_distance[3] > self.up_distance[1]) and (self.up_distance[3] > self.up_distance[2]):
            self.up_board_90()

        elif(self.up_distance[1]<self.up_bd_3 or self.up_distance[2]<self.up_bd_3):
            self.speed=self.speed_2
            self.yspeed = self.c_up_yspeed
            self.up_theta_func()
        elif(self.up_distance[1]<self.up_bd_4) or (self.up_distance[2]<self.up_bd_4):
            self.speed=self.speed_3
            self.yspeed = self.c_up_yspeed
            self.up_theta_func()
        else:
            self.speed=self.speed_5
            self.yspeed = self.c_up_yspeed
            self.up_theta_func()

    
    def down_parallel_board_setup(self): #下板角度調整
        print('down_parallel_board_setup func')
        if self.down_distance[0] < self.down_bd_2 or self.down_distance[3] < self.down_bd_2:#距離在60的時候
            self.speed = self.down_speed_1
            self.yspeed = self.c_down_yspeed
            self.down_theta_func()

        elif self.down_distance[0] <=self.down_bd_3 or self.down_distance[3] <= self.down_bd_3:#距離在60的時候
            self.speed = self.down_speed_2
            self.yspeed = self.c_down_yspeed
            self.down_theta_func()

        elif self.down_distance[0] > self.down_bd_4 or self.down_distance[3] > self.down_bd_4:#距離在大於60的時候
            self.speed = self.down_speed_3
            self.yspeed = self.c_down_yspeed
            self.down_theta_func()

        

    def up_board(self): #要上板了
        print('up_board_func')
        self.parallel_board_setup()
        #self.print_state()
        if (self.up_distance[1]<self.up_bd_1) and (self.up_distance[2]<self.up_bd_1) :
            if self.stop_flag==0 and self.up_board_flag==0:
                print('ready upboard')
                self.speed=0
                self.yspeed=0
                self.theta=0
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(5)
                self.stop_flag=1
                self.up_board_flag=1
                self.next_board()
                self.up_distance = [999,999,999,999]
                send.sendBodyAuto(self.up_b_speed,0,0,0,2,0)
                time.sleep(5)
                            
        else:
            send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)
            print('cant upboard')
            time.sleep(0.5)
        
            

    
    
    def down_board(self): #要下板了
        print('down_board_func')
        self.down_parallel_board_setup()
        if self.down_distance[0] < 10 and self.down_distance[1] < 10 and self.down_distance[2] < 10 and self.down_distance[3] < 10 :
            if self.stop_flag == 0 and self.up_board_flag == 0:
                print('stop_flag = ',self.stop_flag)
                self.speed = 0
                send.sendBodyAuto(self.speed,0,0,0,1,0)
                time.sleep(3)
                self.stop_flag = 1
                self.up_board_flag = 1
                self.next_board()
                self.down_distance = [0,0,0,0]
                self.next_distance = [999,999,999,999]
                time.sleep(5)

        elif self.layer_n != 1 and self.next_distance[0] < 30 and self.down_distance[0] < 50 and self.down_distance[1] < 50:
            # for a in range(230,0,-1):
            #     if send.Label_Model[320*a+105] != self.layer[self.layer_n]:
            #         self.down_distance[0] = 230 - a
            #         break
            # for b in range(230,0,-1):
            #     if send.Label_Model[320*b+105] != self.layer[self.layer_n - 1] and send.Label_Model[320*b+105] != self.layer[self.layer_n]:
            #         self.next_distance[0] = (230 - b) - self.down_distance[0]
            #         break
            print('self.next_distance[0] = ',self.next_distance[0])
            print('space not enoughhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh')
            self.speed = -100
            self.yspeed = -700
            self.theta = -5
            send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)#之後可能要變成變數
            time.sleep(0.5)
            
        elif (self.f_ll-self.point_x)*(self.f_rr-self.point_x)<0:
            print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
            self.speed = -100
            self.yspeed = 0+self.c_down_yspeed
            self.theta = -5
            send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)#之後可能要變成變數
            time.sleep(0.5)
        
        else:
            print('foward')
            send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)
            time.sleep(0.5)
            


    def up_board_90(self): #上板90度狀況
        print('up_board_90 func')
        self.m_xmin=send.color_mask_subject_XMin[self.color_model[self.layer_n]][0]
        self.m_xmax=send.color_mask_subject_XMax[self.color_model[self.layer_n]][0]
        if((self.m_xmax-self.point_x>self.point_x-self.m_xmin) or (self.up_distance[0]-self.up_distance[3])>self.up_bd_2):
                self.speed=-50
                self.yspeed=-500+self.c_up_yspeed
                self.theta=0
                time.sleep(0.5)
                print("move  right 90")
        elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin )or self.up_distance[3]-self.up_distance[0]>self.up_bd_2:
                self.speed=-50
                self.yspeed=500+self.c_up_yspeed
                self.theta=0
                time.sleep(0.5)
                print("move  left 90")


    # def down_board_90(self): #下板90度狀況
    #     print('down_board_90 func')
    #     send.sendContinuousValue(0,-200,0,-5,0)
    #     time.sleep(0.5)
        

    
    def next_board(self):
        if self.direction==0 and self.layer_n < 3:
            self.layer_n+=1
        elif self.direction == 1 and self.layer_n > 0:
            self.layer_n-=1
        elif self.layer_n == 3:
            self.direction = 1



    def up_theta_func(self):
        self.up_feet_distance=self.up_distance[3]-self.up_distance[0]

        if(self.up_feet_distance>self.feet_distance_1):
            print('turn left')
        elif (self.up_feet_distance<(-1*self.feet_distance_1)):
            print('turn right')
        else:
            print('walk forward')

        if(self.up_feet_distance)<(-1*self.feet_distance_4):#右旋
            self.theta = self.r_theta_4
        elif(self.up_feet_distance)<(-1*self.feet_distance_3):
            self.theta = self.r_theta_3
        elif(self.up_feet_distance)<(-1*self.feet_distance_2):
            self.theta = self.r_theta_2
        elif(self.up_feet_distance)<(-1*self.feet_distance_1):
            self.theta =  self.r_theta_1

        elif(self.up_feet_distance)>self.feet_distance_4:#左旋
            self.theta =  self.l_theta_4
        elif(self.up_feet_distance)>self.feet_distance_3:
            self.theta = self.l_theta_3
        elif(self.up_feet_distance)>self.feet_distance_2:
            self.theta = self.l_theta_2
        elif(self.up_feet_distance)>self.feet_distance_1:
            self.theta = self.l_theta_1
        else:
            self.theta = 0

    def down_theta_func(self):
        self.down_feet_distance=self.down_distance[3]-self.down_distance[0]

        if(self.down_feet_distance)<(-1*self.feet_distance_4):#右旋
            self.theta = self.r_theta_4
        elif(self.down_feet_distance)<(-1*self.feet_distance_3):
            self.theta = self.r_theta_3
        elif(self.down_feet_distance)<(-1*self.feet_distance_2):
            self.theta = self.r_theta_2
        elif(self.down_feet_distance)<(-1*self.feet_distance_1):
            self.theta =  self.r_theta_1

        elif(self.down_feet_distance)>self.feet_distance_4:#左旋
            self.theta =  self.l_theta_4
        elif(self.down_feet_distance)>self.feet_distance_3:
            self.theta = self.l_theta_3
        elif(self.down_feet_distance)>self.feet_distance_2:
            self.theta = self.l_theta_2
        elif(self.down_feet_distance)>self.feet_distance_1:
            self.theta = self.l_theta_1
        else:
            self.theta = 0

        if(self.down_feet_distance>self.feet_distance_1):
            print('turn left')
        elif (self.down_feet_distance<(-1*self.feet_distance_1)):
            print('turn right')
        else:
            print('walk forward')



    def print_state(self):
        print("////////////////////////////////////////")
        print("point   x            : ",self.point_x)
        print("point   y            : ",self.point_y)
        print("stop_flag            : ",self.stop_flag)
        print("up board flag        : ",self.up_board_flag)
        print("speed                : ",self.speed)
        print("yspeed               : ",self.yspeed)
        print("theta                : ",self.theta)
        print("layer_now            : ",self.layer_n)
        print("up_feet_distance     : ",self.up_feet_distance)
        print("down_feet_distance   : ",self.down_feet_distance)
        # print("next board   : ",self.layer[self.layer_n])
        if(self.direction==0):
            print("direction    : up board")
            print("up_distance  : ",self.up_distance)
        elif(self.direction==1):
            print("direction    : down_board")
            print("down distance: ",self.down_distance)
            print("next_distance: ",self.next_distance)
            print('next_distance[0] = ',self.next_distance[0])