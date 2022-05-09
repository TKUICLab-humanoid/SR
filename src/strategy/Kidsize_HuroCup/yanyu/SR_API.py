# 4/19
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
        self.knee=215
        self.f_ll=98
        self.f_lr=150
        self.f_rl=188
        self.f_rr=240
        #距離矩陣初始化
        self.up_distance = [999,999,999,999]        #要上的層
        self.down_distance = [999,999,999,999]              #要下的層
        self.next_up_distance = [999,999,999,999]   #上板的下下層
        self.next_down_distance = [999,999,999,999] #要下下的層
        self.up_horizontal=999
        self.up_horizontal_2=999
        #色模
        self.color_model=[3,5,1,2]       
        self.point_x=0    #色模Ymax的x值
        self.point_y=0    #色模Ymax
        self.m_xmin=0
        self.m_xmax=0
        # ///////////  TEST   ///////
        #self.real_board_size=5000
        #self.board_model=0
        self.color_times =0
        self.color_size = 0
        self.color_loc = 0
        self.color_true_times = 0#無用
        self.board_ture=0
        self.f_mid = 160
        
        
        #旗標初始化
        self.stop_flag = 1      # 1表示停止
        self.up_board_flag =0
        self.board_90_flag=[0,0]
        #第幾層
        self.layer_n= 2       #現在站的層,從1開始
        self.layer = [8,32,2,4]     #用在labelMode
        self.direction = 0      #0 上板 1 下板
#//////////////////////////////////////////////////////////////////////
        #校正變數
        self.rc_theta=-3
        self.lc_theta=-3
        #上板校正的
        self.c_speed=-200
        #上板校正的平移
        self.c_up_yspeed =0
        #下板校正的
        self.c_speed=-200
        # 下板校正的平移
        self.c_down_yspeed =0
        #上板x
        self.up_x=8000
        #下板x
        self.down_x=8000
#////////////////////////////////////////////////////////////////////////
        #角度速度初始化
        self.theta = 0+self.rc_theta
        self.speed = 1000+self.c_speed
        self.yspeed =0+self.c_up_yspeed

        #角度設定 左旋
        self.l_theta_1 = 6 + self.lc_theta
        self.l_theta_2 = 7 + self.lc_theta
        self.l_theta_3 = 8 + self.lc_theta
        self.l_theta_4 = 9 + self.lc_theta
        self.l_theta_5 = 10 + self.lc_theta
        #角度設定 右旋
        self.r_theta_1 = -2 + self.rc_theta
        self.r_theta_2 = -3 + self.rc_theta
        self.r_theta_3 = -4 + self.rc_theta
        self.r_theta_4 = -5 + self.rc_theta
        self.r_theta_5 = -6 + self.rc_theta
        
        #上板速度
        self.speed_1=200+self.c_speed
        self.speed_2=400+self.c_speed
        self.speed_3=500+self.c_speed
        self.speed_4=600+self.c_speed
        self.speed_5=700+self.c_speed

        #下板速度
        self.down_speed_1=100+self.c_speed
        self.down_speed_2=300+self.c_speed
        self.down_speed_3=400+self.c_speed
       
        #上板腳離板子差
        self.up_bd_1=8                      #小白 6  小黑 3
        self.up_bd_2=15
        self.up_bd_3=60
        self.up_bd_4=100
        
        # 離板太近距離
        self.back_dis=6                     #小白 4  小黑 1
        self.back_speed=-300+self.c_speed

        # 空間不夠距離
        #上板
        self.space_nud=70
        self.space_ud=70
        #下板
        self.space_ndd=70
        self.space_dd=80

        #下板腳離板子差
        self.down_bd_1=5
        self.down_bd_2=30
        self.down_bd_3=60
        self.down_bd_4=60

        #腳前距離差
        self.feet_distance_1=3
        self.feet_distance_2=6      
        self.feet_distance_3=8
        self.feet_distance_4=10

        #上板3-0
        self.up_feet_distance = 0
        #下板3-0
        self.down_feet_distance = 0

        #no up board 
        self.up_mask = 999
        self.up_mask2 = 999

        
    def find_up_board(self):
        # print('find up board func')
        # 濾掉小色模
        
        self.find_real_board_model(self.color_model[self.layer_n])
        self.up_distance=[999,999,999,999]

        print("color_loc]",self.color_loc)
        print("size",self.color_size)

        #找色模（下一層）Ymax點的X座標
        if self.color_true_times ==1:
            self.point_y=send.color_mask_subject_YMax[self.color_model[self.layer_n]][self.color_loc]
            for mp in range (0,320):
                if send.Label_Model[320*self.point_y+mp] == self.layer[self.layer_n]:
                    self.point_x=mp
                    break
        #左左腳距離 x=115
        for ll in range(self.knee,10,-1):#下往上掃
            # if send.Label_Model[320*ll+self.f_ll] == self.layer[self.layer_n]:
            if send.Label_Model[320*ll+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-1)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-2)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-3)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-4)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-5)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-6)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-7)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-8)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-9)+self.f_ll] == self.layer[self.layer_n] and send.Label_Model[320*(ll-10)+self.f_ll] == self.layer[self.layer_n]:
                self.up_distance[0] = self.knee - ll
                break
        #左右腳距離 x=150
        for lr in range(self.knee,5,-1):
            # if send.Label_Model[320*lr+self.f_lr] == self.layer[self.layer_n]:
            if send.Label_Model[320*lr+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-1)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-2)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-3)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-4)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-5)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-6)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-7)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-8)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-9)+self.f_lr] == self.layer[self.layer_n] and send.Label_Model[320*(lr-10)+self.f_lr] == self.layer[self.layer_n]:
                self.up_distance[1] = self.knee - lr
                break
        #右左腳距離 x=165
        for rl in range(self.knee,5,-1):
            # if send.Label_Model[320*rl+self.f_rl] == self.layer[self.layer_n]:
            if send.Label_Model[320*rl+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-1)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-2)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-3)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-4)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-5)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-6)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-7)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-8)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-9)+self.f_rl] == self.layer[self.layer_n] and send.Label_Model[320*(rl-10)+self.f_rl] == self.layer[self.layer_n]:
                self.up_distance[2] = self.knee - rl
                break
        #右右腳距離 x=200
        for rr in range(self.knee,5,-1):
            # if send.Label_Model[320*rr+self.f_rr] == self.layer[self.layer_n]:
            if send.Label_Model[320*rr+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-1)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-2)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-3)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-4)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-5)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-6)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-7)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-8)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-9)+self.f_rr] == self.layer[self.layer_n] and send.Label_Model[320*(rr-10)+self.f_rr] == self.layer[self.layer_n]:
                self.up_distance[3] = self.knee - rr
                break

        if self.layer_n<=2:
            self.next_up_distance=[999,999,999,999]
            for ll_2 in range(ll,5,-1):     #找下下一層 從原本掃過得地方再往前掃
                if send.Label_Model[320*ll_2+self.f_ll] == self.layer[self.layer_n+1] and send.Label_Model[320*(ll_2-1)+self.f_ll] == self.layer[self.layer_n+1] and send.Label_Model[320*(ll_2-2)+self.f_ll] == self.layer[self.layer_n+1] and send.Label_Model[320*(ll_2-3)+self.f_ll] == self.layer[self.layer_n+1] and send.Label_Model[320*(ll_2-4)+self.f_ll] == self.layer[self.layer_n+1] and send.Label_Model[320*(ll_2-5)+self.f_ll] == self.layer[self.layer_n+1]:
                    self.next_up_distance[0] = ll-ll_2
                    break
            for lr_2 in range(lr,5,-1):
                if send.Label_Model[320*lr_2+self.f_lr] == self.layer[self.layer_n+1] and send.Label_Model[320*(lr_2-1)+self.f_lr] == self.layer[self.layer_n+1] and send.Label_Model[320*(lr_2-2)+self.f_lr] == self.layer[self.layer_n+1] and send.Label_Model[320*(lr_2-3)+self.f_lr] == self.layer[self.layer_n+1] and send.Label_Model[320*(lr_2-4)+self.f_lr] == self.layer[self.layer_n+1] and send.Label_Model[320*(lr_2-5)+self.f_lr] == self.layer[self.layer_n+1]:
                    self.next_up_distance[1] = lr-lr_2
                    break
            
            for rl_2 in range(rl,5,-1):
                if send.Label_Model[320*rl_2+self.f_rl] == self.layer[self.layer_n+1] and send.Label_Model[320*(rl_2-1)+self.f_rl] == self.layer[self.layer_n+1] and send.Label_Model[320*(rl_2-2)+self.f_rl] == self.layer[self.layer_n+1] and send.Label_Model[320*(rl_2-3)+self.f_rl] == self.layer[self.layer_n+1] and send.Label_Model[320*(rl_2-4)+self.f_rl] == self.layer[self.layer_n+1] and send.Label_Model[320*(rl_2-5)+self.f_rl] == self.layer[self.layer_n+1]:
                    self.next_up_distance[2] = rl-rl_2
                    break
            
            for rr_2 in range(rr,5,-1):
                if send.Label_Model[320*rr_2+self.f_rr] == self.layer[self.layer_n+1] and send.Label_Model[320*(rr_2-1)+self.f_rr] == self.layer[self.layer_n+1] and send.Label_Model[320*(rr_2-2)+self.f_rr] == self.layer[self.layer_n+1] and send.Label_Model[320*(rr_2-3)+self.f_rr] == self.layer[self.layer_n+1] and send.Label_Model[320*(rr_2-4)+self.f_rr] == self.layer[self.layer_n+1] and send.Label_Model[320*(rr_2-5)+self.f_rr] == self.layer[self.layer_n+1]:
                    self.next_up_distance[3] = rr-rr_2
                    break
        else:
            self.next_up_distance=[999,999,999,999]

    def find_down_board(self):
        # print('find down board func')
        self.find_real_board_model(self.color_model[self.layer_n])

        if self.color_true_times ==1:
            self.point_y=send.color_mask_subject_YMin[self.color_model[self.layer_n]][self.color_loc]
            for mp in range (0,320):
                if send.Label_Model[320*self.point_y+mp] == self.layer[self.layer_n]:
                    self.point_x=mp
                    break

        #self.point_y=send.color_mask_subject_YMin[self.color_model[self.layer_n]][self.board_model]
        #找色模(自己層）Ymin點的X座標
        # for mp in range (0,320):
        #     if send.Label_Model[320*self.point_y+mp] == self.layer[self.layer_n]:
        #         self.point_x=mp
        #         break

        #左左腳距離  #找要下的層
        for ll in range(self.knee,5,-1):      
            if send.Label_Model[320*ll+self.f_ll] == self.layer[self.layer_n -1] and send.Label_Model[320*(ll-1)+self.f_ll] == self.layer[self.layer_n -1] and send.Label_Model[320*(ll-2)+self.f_ll] == self.layer[self.layer_n -1] and send.Label_Model[320*(ll-3)+self.f_ll] == self.layer[self.layer_n -1] and send.Label_Model[320*(ll-4)+self.f_ll] == self.layer[self.layer_n -1] and send.Label_Model[320*(ll-5)+self.f_ll] == self.layer[self.layer_n -1]:
                self.down_distance[0] = self.knee - ll
                break
        #左右腳距離
        for lr in range(self.knee,5,-1):
            if send.Label_Model[320*lr+self.f_lr] ==  self.layer[self.layer_n -1] and send.Label_Model[320*(lr-1)+self.f_lr] ==  self.layer[self.layer_n -1] and send.Label_Model[320*(lr-2)+self.f_lr] ==  self.layer[self.layer_n -1] and send.Label_Model[320*(lr-3)+self.f_lr] ==  self.layer[self.layer_n -1] and send.Label_Model[320*(lr-4)+self.f_lr] ==  self.layer[self.layer_n -1] and send.Label_Model[320*(lr-5)+self.f_lr] ==  self.layer[self.layer_n -1]:
                self.down_distance[1] = self.knee - lr
                break

        #右左腳距離
        for rl in range(self.knee,5,-1):
            if send.Label_Model[320*rl+self.f_rl] ==  self.layer[self.layer_n-1] and send.Label_Model[320*(rl-1)+self.f_rl] ==  self.layer[self.layer_n-1] and send.Label_Model[320*(rl-2)+self.f_rl] ==  self.layer[self.layer_n-1] and send.Label_Model[320*(rl-3)+self.f_rl] ==  self.layer[self.layer_n-1] and send.Label_Model[320*(rl-4)+self.f_rl] ==  self.layer[self.layer_n-1] and send.Label_Model[320*(rl-5)+self.f_rl] ==  self.layer[self.layer_n-1]:
                self.down_distance[2] = self.knee - rl
                break

        #右右腳距離
        for rr in range(self.knee,5,-1):
            if send.Label_Model[320*rr+self.f_rr] == self.layer[self.layer_n-1] and send.Label_Model[320*(rr-1)+self.f_rr] == self.layer[self.layer_n-1] and send.Label_Model[320*(rr-2)+self.f_rr] == self.layer[self.layer_n-1] and send.Label_Model[320*(rr-3)+self.f_rr] == self.layer[self.layer_n-1] and send.Label_Model[320*(rr-4)+self.f_rr] == self.layer[self.layer_n-1] and send.Label_Model[320*(rr-5)+self.f_rr] == self.layer[self.layer_n-1]:
                self.down_distance[3] = self.knee - rr
                break
        
        #找下一板可踩空間
        if self.layer_n>=2:
            for ll_2 in range(ll,5,-1):     
                if send.Label_Model[320*ll_2+self.f_ll] == self.layer[self.layer_n-2] and send.Label_Model[320*(ll_2-1)+self.f_ll] == self.layer[self.layer_n-2] and send.Label_Model[320*(ll_2-2)+self.f_ll] == self.layer[self.layer_n-2] and send.Label_Model[320*(ll_2-3)+self.f_ll] == self.layer[self.layer_n-2] and send.Label_Model[320*(ll_2-4)+self.f_ll] == self.layer[self.layer_n-2] and send.Label_Model[320*(ll_2-5)+self.f_ll] == self.layer[self.layer_n-2]:
                    self.next_down_distance[0] = ll-ll_2
                    break
            for lr_2 in range(lr,0,-1):
                if send.Label_Model[320*lr_2+self.f_lr] == self.layer[self.layer_n-2]:
                    self.next_down_distance[1] = lr - lr_2
                    break
            
            for rl_2 in range(rl,0,-1):
                if send.Label_Model[320*rl_2+self.f_rl] == self.layer[self.layer_n-2]:
                    self.next_down_distance[2] = rl - rl_2
                    break
            
            for rr_2 in range(rr,0,-1):
                if send.Label_Model[320*rr_2+self.f_rr] == self.layer[self.layer_n-2]:
                    self.next_down_distance[3] = rr - rr_2
                    break
        else:
            self.next_down_distance=[999,999,999,999]

    def parallel_board_setup(self):#上板的角度調整
        # print('parallel_board_setup func')
        #條件要調整！！！
        # [0][1][2][3]都不能<0
        if(self.up_distance[0]<=self.back_dis or self.up_distance[1]<=self.back_dis or self.up_distance[2]<=self.back_dis or self.up_distance[3]<=self.back_dis):
            if self.up_distance[3]-self.up_distance[0] > 3 or self.up_distance[0]-self.up_distance[3] >3:
                # print("back back back back back back back back")
                self.speed=self.back_speed
                self.yspeed = self.c_up_yspeed
                self.up_theta_func()
            else:
                self.speed=self.speed_1
                self.yspeed = self.c_up_yspeed
                self.up_theta_func()
        else :
            #self.up_mask=send.color_mask_subject_cnts[self.color_model[self.layer_n]]
            #self.find_real_board_model(self.color_model[self.layer_n])
            #self.up_horizontal_2=send.color_mask_subject_X[self.color_model[self.layer_n]][self.board_model]
            # 上板直角
            if((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (20<self.up_distance[0]<100 or 20<self.up_distance[1]<100 or 20<self.up_distance[2]<100 or 20<self.up_distance[3]<100):
                print("90")
                self.up_board_90()
                
            #找不到板
            elif self.layer_n > 1 and self.up_distance[0]>250 and self.up_distance[3]>250:#數值我想測試
            # elif self.board_ture==0:
                print("no")
                self.no_up_board()

            elif(self.up_distance[0]<=self.up_bd_2 and self.up_distance[3]<=self.up_bd_2):#30
                print("1111111111111111111111111111111")
                self.speed=self.speed_1
                self.yspeed = self.c_up_yspeed
                self.up_theta_func()
            elif(self.up_distance[1]<self.up_bd_3 or self.up_distance[2]<self.up_bd_3):
                print("2222222222222222222222222222222")
                self.speed=self.speed_2
                self.yspeed = self.c_up_yspeed
                self.up_theta_func()
            elif(self.up_distance[1]<self.up_bd_4) or (self.up_distance[2]<self.up_bd_4):
                print("333333333333333333333333333")
                self.speed=self.speed_3
                self.yspeed = self.c_up_yspeed
                self.up_theta_func()
            else:
                print("44444444444444444444444444444444")
                self.speed=self.speed_5
                self.yspeed = self.c_up_yspeed
                self.up_theta_func()
            
            # 空間不夠
            # 想要怎麼決定左移還右移
            if self.layer_n != 3 and self.next_up_distance[0] < self.space_nud and self.up_distance[0] < self.space_ud and self.up_distance[1] < self.space_ud:
                print('space not enoughhhhhhhhhhh')
                if self.yspeed==self.c_up_yspeed:#沒進90
                    self.speed = -100+self.c_speed
                    self.yspeed = -800+self.c_up_yspeed
                    self.up_theta_func()

                else:
                    self.speed = -100+self.c_speed
                    self.yspeed = self.yspeed
                    self.up_theta_func()

    
    # def down_parallel_board_setup(self): #下板角度調整

    #     if((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (self.down_distance[0]>40 or self.down_distance[1]>40 or self.down_distance[2]>40 or self.down_distance[3]>40):
    #         self.down_board_90()
    #     # print('down_parallel_board_setup func')
    #     if self.down_distance[0] < self.down_bd_2 or self.down_distance[3] < self.down_bd_2:#距離在60的時候
    #         self.speed = self.down_speed_1
    #         self.yspeed = self.c_down_yspeed
    #         self.down_theta_func()

    #     elif self.down_distance[0] <=self.down_bd_3 or self.down_distance[3] <= self.down_bd_3:#距離在60的時候
    #         self.speed = self.down_speed_2
    #         self.yspeed = self.c_down_yspeed
    #         self.down_theta_func()

    #     elif self.down_distance[0] > self.down_bd_4 or self.down_distance[3] > self.down_bd_4:#距離在大於60的時候
    #         self.speed = self.down_speed_3
    #         self.yspeed = self.c_down_yspeed
    #         self.down_theta_func()

    #     if self.layer_n != 1 and self.next_down_distance[0] < self.space_nud and self.down_distance[0] < self.space_ud and self.down_distance[1] < self.space_ud:
    #         # print('self.next_up_distance[0] = ',self.next_up_distance[0])
    #         # print('space not enoughhhhhhhhhhh')
    #         if self.yspeed==self.c_down_yspeed:
    #             self.speed = -100+self.c_speed
    #             self.yspeed = -1000+self.yspeed
    #             self.up_theta_func()

    #         else:
    #             self.speed = -100+self.c_speed
    #             self.yspeed = self.yspeed
    #             self.up_theta_func()

    def down_parallel_board_setup(self): #下板角度調整       
        # 下板直角
        if((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (self.down_distance[0]>40 or self.down_distance[1]>40 or self.down_distance[2]>40 or self.down_distance[3]>40):
            self.down_board_90()

        elif self.down_distance[0] <= self.down_bd_2 and self.down_distance[3] <= self.down_bd_2:#距離小於30的時候
            self.speed = self.down_speed_1
            self.yspeed = self.c_down_yspeed
            self.down_theta_func()
        elif self.down_distance[0] <=self.down_bd_3 and self.down_distance[3] <= self.down_bd_3:#距離小於60的時候
            self.speed = self.down_speed_2
            self.yspeed = self.c_down_yspeed
            self.down_theta_func()
        elif self.down_distance[0] > self.down_bd_4 or self.down_distance[3] > self.down_bd_4:#距離在大於60的時候
            self.speed = self.down_speed_3
            self.yspeed = self.c_down_yspeed
            self.down_theta_func()

        # 空間不夠
        if self.layer_n != 1 and self.next_down_distance[0] < self.space_ndd and self.down_distance[0] < self.space_dd and self.down_distance[1] < self.space_dd:
            if self.yspeed==self.c_down_yspeed:#沒進90度
                if self.layer_n == 3:
                    self.speed = self.c_speed
                    self.yspeed = self.c_down_yspeed 
                    self.theta = -3 +self.rc_theta
                else:
                    self.speed = -100+self.c_speed
                    self.yspeed = -800+self.c_down_yspeed 
                    self.down_theta_func()

            else:#有進90度
                self.speed = self.speed
                self.yspeed = self.yspeed
                self.theta=self.theta

        

    def up_board(self): #要上板了
        # print('up_board_func')
        self.parallel_board_setup()
        if ((self.up_distance[1]<self.up_bd_1) and (self.up_distance[2]<self.up_bd_1)) and (abs(self.up_distance[3]-self.up_distance[0])<self.feet_distance_1) :
            if self.stop_flag==0 and self.up_board_flag==0:
                print('ready upboard')
                self.speed=0
                self.yspeed=0
                self.theta=0
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(5)
                # if self.layer_n == 1:
                #     send.sendBodySector(1)
                # else:
                #     send.sendBodySector(1)
                time.sleep(2)
                self.stop_flag=1
                self.up_board_flag=1
                self.next_board()
                self.up_distance = [999,999,999,999]
                #send.sendBodyAuto(self.up_x,0,0,0,2,0)
                #time.sleep(5)
                send.sendBodySector(29)
                time.sleep(5)
                            
        else:
            send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)
            #print('cant upboard')

    
    def down_board(self): #要下板了
        # print('down_board_func')
        self.down_parallel_board_setup()
        if self.down_distance[0] < self.down_bd_1 and self.down_distance[1] < self.down_bd_1 and self.down_distance[2] < self.down_bd_1 and self.down_distance[3] < self.down_bd_1 :
            if self.stop_flag == 0 and self.up_board_flag == 0:
                print('ready upboard')
                self.speed=0
                self.yspeed=0
                self.theta=0
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(3)
                #send.sendBodySector(1)
                self.stop_flag = 1
                self.up_board_flag = 1
                self.next_board()
                self.down_distance = [999,999,999,999]
                self.next_down_distance = [999,999,999,999]
                # send.sendBodyAuto(self.down_x,0,0,0,2,0)
                time.sleep(5)
                send.sendBodySector(29)
                time.sleep(5)
        # elif self.layer_n != 1 and self.next_down_distance[0] < self.space_ndd and self.down_distance[0] < self.space_dd and self.down_distance[1] < self.space_dd:#距離不夠
        #     print('self.next_down_distance[0] = ',self.next_down_distance[0])
        #     # print('space not enoughhhhhhh')
        #     self.speed = -100+self.c_speed
        #     self.yspeed =self.c_down_yspeed
        #     self.theta = -5
        #     send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)#之後可能要變成變數
            
        # elif (self.f_ll-self.point_x)*(self.f_rr-self.point_x)<0:#90度
        #     # print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
        #     self.speed = -100+self.c_speed
        #     self.yspeed = self.c_down_yspeed
        #     self.theta = -5
        #     send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)#之後可能要變成變數
            
        else:
            send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)


    def no_up_board(self):#看不到板子時,n>1
        print("color_loc]",self.color_loc)
        print("size",send.color_mask_subject_size[self.layer_n][self.color_loc])
        print("pppppppppppppppppppppppppppppppppppppppppppppppppppppppp")
        self.up_mask=send.color_mask_subject_cnts[self.color_model[self.layer_n]]
        if self.board_ture==0:
            print("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
            self.up_mask2=send.color_mask_subject_cnts[self.color_model[self.layer_n-2]]#我站在紅板,沒看到黃板,看有沒有綠板
            if self.up_mask2==0:
                print("nnnnnnnnnnnnnnnnnnnnnnnnnnnn")
                self.speed = 200 + self.c_speed
                self.yspeed = self.c_up_yspeed
                self.theta = self.lc_theta
            else:
                print("ggggggggggggggggggggggggggggggggggg")#有綠板
                #self.find_real_board_model(self.color_model[self.layer_n-2])
                self.up_horizontal=send.color_mask_subject_X[self.color_model[self.layer_n-2]][0]
                if self.up_horizontal<self.f_mid:
                    self.speed = 0 + self.c_speed
                    self.yspeed = self.c_up_yspeed
                    self.theta = -15 + self.rc_theta
                    # print("cant find board : turn left")
                    # send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)
                elif self.up_horizontal>self.f_mid and self.up_horizontal<999:
                    self.speed = 0 + self.c_speed
                    self.yspeed = self.c_up_yspeed
                    self.theta = 15 + self.lc_theta 

        else:
            print("有囉")
            #print(self.color_loc)
            self.up_horizontal_2=send.color_mask_subject_X[self.color_model[self.layer_n]][self.color_loc]
            if self.up_horizontal_2<self.f_mid:
                self.speed = 0 + self.c_speed
                self.yspeed = self.c_up_yspeed
                self.theta = 12 + self.lc_theta
                # print("cant find board : turn left")
                # send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)
            elif self.up_horizontal_2>self.f_mid and self.up_horizontal_2<999:
                self.speed = 0 + self.c_speed
                self.yspeed = self.c_up_yspeed
                self.theta = -8 + self.rc_theta            
                #print("cant find board : turn right")
                #send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)

    # def no_up_board(self):#看不到板子時
    #     self.up_horizontal=send.color_mask_subject_X[self.color_model[self.layer_n]][0]
    #         if self.up_horizontal<self.f_ll:
    #             self.speed = 300 + self.c_speed
    #             self.yspeed = self.c_up_yspeed
    #             self.theta = 5 + self.lc_theta
    #             # print("cant find board : turn left")
    #             # send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)
    #         elif self.up_horizontal>self.f_rr and self.up_horizontal<999:
    #             self.speed = 300 + self.c_speed
    #             self.yspeed = self.c_up_yspeed
    #             self.theta = -5 + self.lc_theta            
    #             #print("cant find board : turn right")
    #             #send.sendContinuousValue(self.speed,self.yspeed,0,self.theta,0)



    def up_board_90(self): #上板90度狀況
        # print('up_board_90 func')
        #self.find_real_board_model(self.color_model[self.layer_n])
        self.m_xmin=send.color_mask_subject_XMin[self.color_model[self.layer_n]][self.color_loc]
        self.m_xmax=send.color_mask_subject_XMax[self.color_model[self.layer_n]][self.color_loc]
        if(self.m_xmax-self.point_x>self.point_x-self.m_xmin):
            self.speed=self.c_speed
            self.yspeed=-800+self.c_up_yspeed
            self.theta=0 + self.lc_theta
            #print("move  right 90")
        elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin):
            self.speed=self.c_speed
            self.yspeed=800+self.c_up_yspeed
            self.theta=0 + self.rc_theta
            #print("move  left 90") 

    def down_board_90(self): #下板90度狀況
        #self.find_real_board_model(self.color_model[self.layer_n])
        self.m_xmin=send.color_mask_subject_XMin[self.color_model[self.layer_n]][self.board_model]
        self.m_xmax=send.color_mask_subject_XMax[self.color_model[self.layer_n]][self.board_model]
        if(self.m_xmax-self.point_x>self.point_x-self.m_xmin):
        #if((self.m_xmax-self.point_x>self.point_x-self.m_xmin) or (self.up_distance[0]-self.up_distance[3])>self.up_bd_2):
        # if self.point_x<=160:
            self.speed=self.c_speed
            self.yspeed=-800+self.c_up_yspeed
            self.theta=0 + self.lc_theta 
            #print("move  right 90")
        elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin):
        #elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin )or self.up_distance[3]-self.up_distance[0]>self.up_bd_2:
        # else:
            self.speed=self.c_speed
            self.yspeed=800+self.c_up_yspeed
            self.theta=0 + self.rc_theta 
            #print("move  left 90") 

    
    def next_board(self):
        if self.direction==0 and self.layer_n < 3:
            self.layer_n+=1
        elif self.direction == 1 and self.layer_n > 0:
            self.layer_n-=1
        elif self.layer_n == 3:
            self.direction = 1

    
    def find_real_board_model(self,find):
        # # size要再調整 (要大於錢幣)
        # # 如果現在選到的色模小於板子的size就換下一個色模
        # while send.color_mask_subject_size[find][self.board_model]<self.real_board_size:
        #     self.board_model=self.board_model+1
        #     # 但如果編號大於總數量 則歸零並跳出
        #     if self.board_model>send.color_mask_subject_cnts[find]:
        #         self.board_model=0
        #         break
        self.color_times=send.color_mask_subject_cnts[find]
        if self.color_times != 0:
            self.color_true_times =1
            for i in range(self.color_times):
                self.color_size = send.color_mask_subject_size[find][i]
                print("全部size",self.color_size)
                if self.color_size >50000:#錢幣大小(要測試)
                    print("yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
                    self.color_loc = i
                    self.board_ture=1
                    break
        else:
            ptint("uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")
            self.color_true_times = 0#無用
        

    def up_theta_func(self):
        self.up_feet_distance=self.up_distance[3]-self.up_distance[0]

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
            self.theta = 0+self.lc_theta

        if(self.up_feet_distance>self.feet_distance_1):
            print('turn left')
        elif (self.up_feet_distance<(-1*self.feet_distance_1)):
            print('turn right')
        else:
            print('walk forward')

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
        print("/////////////////////////////////////////////////////////")
# //////////////////                     test                           //////////////////////////////
        # print("real board model             : ",self.board_model)
        # print("board size                   : ",send.color_mask_subject_size[self.color_model[self.layer_n]][self.board_model])
# ////////////////////////////////////////////////////////////////////////////////////////////////////  
        print("point   y                    : ",self.point_y)
        print("point   x                    : ",self.point_x)
        print("stop_flag                    : ",self.stop_flag)
        print("up board flag                : ",self.up_board_flag)
        print("speed                        : ",self.speed)
        print("yspeed                       : ",self.yspeed)
        print("theta                        : ",self.theta)
        print("layer_now                    : ",self.layer_n)
        if(self.direction==0):
            print("direction                    :  up board")
            print("up_distance                  : ",self.up_distance)
            print("next_up_distance             : ",self.next_up_distance)
            print('next_up_distance[0]          : ',self.next_up_distance[0])
            print("next board x point           : ",self.up_horizontal_2)
            if(self.up_distance[0]<=self.back_dis or self.up_distance[1]<=self.back_dis or self.up_distance[2]<=self.back_dis or self.up_distance[3]<=self.back_dis):
                if self.up_distance[3]-self.up_distance[0] > 3 or self.up_distance[0]-self.up_distance[3] >3:
                    print("back back back back back back back back")
                else:
                    print("有進back back,但角度可以繼續直走")
            else:
                #90度
                if((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (20<self.up_distance[0]<100 or 20<self.up_distance[1]<100 or 20<self.up_distance[2]<100 or 20<self.up_distance[3]<100):
                    if(self.m_xmax-self.point_x>self.point_x-self.m_xmin):
                        print("move  right 90")
                    elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin):
                        print("move  left 90")
                #找不到板
                elif self.layer_n > 1 and self.up_distance[0]>250 and self.up_distance[3]>250:
                #elif self.layer_n > 1 and (self.up_mask==0 or self.f_ll<self.up_horizontal_2<self.f_rr):
                    if self.up_mask==0:
                        print("cant find board")
                        if self.up_mask2==0:
                            print("cant find green board : forward")
                        else:
                            if self.up_horizontal<self.f_ll:
                                print("find green board : turn right")
                            elif self.up_horizontal>self.f_rr and self.up_horizontal<999:
                                print("find green board : turn left")
                    else:
                        if self.up_horizontal_2<self.f_ll:
                            print("find board : turn left")
                        elif self.up_horizontal_2>self.f_rr and self.up_horizontal_2<999:
                            print("find board : turn right")

                if self.layer_n != 3 and self.next_up_distance[0] < self.space_nud and self.up_distance[0] < self.space_ud and self.up_distance[1] < self.space_ud:
                    if self.yspeed == -800+self.c_up_yspeed:
                        print("空間不夠,沒進90 : 右平移")
                    else:
                        print("空間不夠,有進90 : 繼續90")



            # print("direction                    :  up board")
            # print("up_distance                  : ",self.up_distance)
            # print("next_up_distance             : ",self.next_up_distance)
            # print('next_up_distance[0] = ',self.next_up_distance[0])
            # print("next board x point           :",self.up_horizontal)
            # if((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (self.up_distance[0]>40 or self.up_distance[1]>40 or self.up_distance[2]>40 or self.up_distance[3]>40):
            #     # print('up_board_90 func')
            #     if((self.m_xmax-self.point_x>self.point_x-self.m_xmin) or (self.up_distance[0]-self.up_distance[3])>self.up_bd_2):
            #         print("move  right 90")
            #     elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin )or self.up_distance[3]-self.up_distance[0]>self.up_bd_2:
            #         print("move  left 90")
            # elif self.layer_n != 3 and self.next_up_distance[0] < 50 and (self.up_distance[0] < 60 or self.up_distance[1] < 60):
            #     print('self.next_up_distance[0] = ',self.next_up_distance[0])
            #     print('space not enough')
            # else:
            #     print('cant upboard')





        elif(self.direction==1):
            print("direction                    :  down_board")
            print("down distance                : ",self.down_distance)
            print("next_down_distance           : ",self.next_down_distance)
            print('next_down_distance[0]        : ',self.next_down_distance[0])
            if((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (self.down_distance[0]>40 or self.down_distance[1]>40 or self.down_distance[2]>40 or self.down_distance[3]>40):
                if(self.m_xmax-self.point_x>self.point_x-self.m_xmin):
                    print("move  right 90")
                elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin):
                    print("move  left 90")

            if self.layer_n != 1 and self.next_down_distance[0] < self.space_ndd and self.down_distance[0] < self.space_dd and self.down_distance[1] < self.space_dd:
                if self.yspeed==self.c_down_yspeed or self.speed == -100+self.c_speed:#沒進90度
                    if self.layer_n == 3:
                        print("空間不夠,沒進90 : 右旋")
                    else:
                        print("空間不夠,沒進90 : 右平移")
                else:
                    print("空間不夠,有進90 : 繼續90")