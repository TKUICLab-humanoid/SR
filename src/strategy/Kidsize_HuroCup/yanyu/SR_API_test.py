#!/usr/bin/env python
#coding=utf-8

# 校正變數
# 第幾層
# 改線
# 上板空間不夠
# 下板空間不夠
#revision
import rospy
import numpy as np
from rospy import Publisher
# from tku_msgs.msg import Interface,HeadPackage,SandHandforward,DrawImage,SingleMotorData,SensorSet,ObjectList,LabelModelObjectList,RobotPos,SetGoalPoint,SoccerDataList,SensorPackage
from std_msgs.msg import Int16,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import cv2
import time
from Python_API import Sendmessage

send = Sendmessage()#要放在class外面,不然不能宣告
BLUE   = 2
RED    = 5
YELLOW = 1
GREEN  = 3


#--校正量--#
#校正變數       #revision
THETA_CORRECTION = 0
#前進量校正
FORWARD_CORRECTION  = -100
#平移校正
TRANSLATION_CORRECTION = -500
#----------#


class Send_distance():
    def __init__(self):#初始化
        self.upboard_start = 0
        self.upboard_end   = 9999
        #腳掌標準線x值      #revision
        self.knee=225
        self.f_ll=118
        self.f_lr=self.f_ll+32
        self.f_rl=180
        self.f_rr=self.f_rl+32
        self.head_Horizontal = 2067
        self.head_Vertical = 2727#2698
        #距離矩陣初始化
        self.next_up_distance = [999,999,999,999]     #要上的層
        self.next_down_distance = [999,999,999,999]   #要下的層
        self.second_up_distance = [999,999,999,999]   #上板的下下層
        self.second_down_distance = [999,999,999,999] #要下下的層
        self.previous_board_x_center=999
        self.next_board_x_center=999
        #色模
        self.color_model=[GREEN,BLUE,RED,YELLOW]       #藍紅黃
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
        # ///////////  TEST   ///////
        #距離判斷旗標
        self.ready_to_updown_board = False
        self.first_judge_line      = False
        self.secend_judge_line     = False
        self.third_judge_line      = False
        #旗標初始化
        self.stop_flag = 1      # 1表示停止
        self.up_board_flag =0
        self.board_90_flag=[0,0]
        #第幾層
        self.layer_n= 1     #現在站的層,從1開始
        #            [       8,     32,     2,        4]   #用在labelMode
        self.layer = [2**GREEN,2**BLUE,2**RED,2**YELLOW]   
        self.direction = 0      #0 上板 1 下板

        

        #上板x
        self.LCup_x     =16000      #Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = 3.2
        #下板x
        self.LCdown_x   =20000      #Y_swing = 7,Period_T = 840,OSC_LockRange = 0.4,BASE_Default_Z = 8,BASE_LIFT_Z = -1.5
        
        #角度速度初始化
        self.theta = 0+THETA_CORRECTION
        self.forward = 0+FORWARD_CORRECTION
        self.translation =0+TRANSLATION_CORRECTION

        #角度設定 左旋
        self.l_theta = 1
        #角度設定 右旋
        self.r_theta = -1
        #決定角度
        self.decide_theta = 0

        #前進速度
        #固定值
        self.forward_min    =200    #小前       #revision
        self.forward_normal =800    #前
        self.forward_big    =1300   #大前
        self.forward_super  =2500   #超大前
        #變化值
        self.forward_change =100    #前進變化量

        #下板速度
        self.down_forward_1=500+FORWARD_CORRECTION
        self.down_forward_2=700+FORWARD_CORRECTION
        self.down_forward_3=900+FORWARD_CORRECTION
       
        #上板腳離板子差
        self.up_bd_1=4                      #小白 6  小黑 3
        self.up_bd_2=30
        self.up_bd_3=70
        self.up_bd_4=100
        
        # 上板離板太近距離
        self.back_dis=2                   #小白 4  小黑 1           #revision
        self.step_back   = -600
        # self.step_back_2 = -700+FORWARD_CORRECTION

        # 空間不夠距離
        #上板
        self.space_not_enough_up=30         #revision
        # self.space_not_enough_up=65
        #下板
        self.sec_space_not_enough_down=20
        self.space_not_enough_down=40

        #下板腳離板子差
        self.down_bd_1=4            #revision
        self.down_bd_2=30
        self.down_bd_3=60
        self.down_bd_4=60

        #腳前距離差,斜率
        self.feet_distance_1=3      #有點斜
        self.feet_distance_2=6      #斜
        self.feet_distance_3=10      #過斜

        #上板3-0
        self.up_feet_distance = 0       
        #下板3-0
        self.down_feet_distance = 0     

        #no up board 
        self.up_mask = 999
        self.up_mask2 = 999

        #//test//
        self.down_max = 0
        self.next_down_max = 0
        self.big_to_small_next_down_distance = [999,999,999,999]

        # space not enough counter
        self.counter =0
        self.counter_max = 10
        self.not_enough_flag=0

        # 90 counter
        self.counter_90 =0
        self.counter_90_max = 10
        self.flag_90=0

    # 改線
    def set_line(self):
        if self.layer_n==3 and self.direction==0:
            self.knee=220
            self.f_ll=128
            self.f_lr=self.f_ll+32
            self.f_rl=170
            self.f_rr=self.f_rl+32
        elif self.layer_n==1 and self.direction==1:
            # self.f_ll=78
            # self.f_lr=self.f_ll+52-20
            # self.f_rl=170
            # self.f_rr=self.f_rl+52
            pass
        else:
            self.f_ll=98
            self.f_lr=self.f_ll+52
            self.f_rl=170
            self.f_rr=self.f_rl+52




        
    def find_up_board(self):
        # print('find up board func')
        # 濾掉小色模
        
        self.find_real_board_model(self.color_model[self.layer_n])  #尋找板子為色模中的第幾個物件,
        self.next_up_distance=[999,999,999,999]
        self.second_up_distance = [999,999,999,999]
        self.point_x=0    #色模Ymax的x值
        self.point_y=0

        #找色模（下一層）Ymax點的X座標
        if self.color_true_times ==1 and self.board_ture==1:
            self.point_y=send.color_mask_subject_YMax[self.color_model[self.layer_n]][self.color_loc]
            for mp in range (0,320):
                if send.Label_Model[320*self.point_y+mp] == self.layer[self.layer_n]:
                    self.point_x=mp
                    break

        # self.return_distance(self.next_up_distance,self.layer_n)
        #左左腳距離 x=115
        for ll in range(self.knee,10,-1):#下往上掃
            # if send.Label_Model[320*ll+self.f_ll] == self.layer[self.layer_n]:
            if self.return_real_board(ll,self.f_ll,self.layer_n):
                self.next_up_distance[0] = self.knee - ll
                break

        #左右腳距離 x=150
        for lr in range(self.knee,10,-1):
            # if send.Label_Model[320*lr+self.f_lr] == self.layer[self.layer_n]:
            if self.return_real_board(lr,self.f_lr,self.layer_n):
                self.next_up_distance[1] = self.knee - lr
                break
        #右左腳距離 x=165
        for rl in range(self.knee,10,-1):
            # if send.Label_Model[320*rl+self.f_rl] == self.layer[self.layer_n]:
            if self.return_real_board(rl,self.f_rl,self.layer_n):
                self.next_up_distance[2] = self.knee - rl
                break
        #右右腳距離 x=200
        for rr in range(self.knee,10,-1):
            # if send.Label_Model[320*rr+self.f_rr] == self.layer[self.layer_n]:
            if self.return_real_board(rr,self.f_rr,self.layer_n):
                self.next_up_distance[3] = self.knee - rr
                break
        
        if self.layer_n<=2:
            self.second_up_distance=[999,999,999,999]
            for ll_2 in range(ll,10,-1):     #找下下一層 從原本掃過得地方再往前掃
                if self.return_real_board(ll_2,self.f_ll,self.layer_n+1):
                    self.second_up_distance[0] = ll-ll_2
                    break
            for lr_2 in range(lr,10,-1):
                if self.return_real_board(lr_2,self.f_lr,self.layer_n+1):
                    self.second_up_distance[1] = lr-lr_2
                    break
            
            for rl_2 in range(rl,10,-1):
                if self.return_real_board(rl_2,self.f_rl,self.layer_n+1):
                    self.second_up_distance[2] = rl-rl_2
                    break
            
            for rr_2 in range(rr,10,-1):
                if self.return_real_board(rr_2,self.f_rr,self.layer_n+1):
                    self.second_up_distance[3] = rr-rr_2
                    break
        else:
            self.second_up_distance=[999,999,999,999]

    def find_down_board(self):
        # print('find down board func')
        self.find_real_board_model(self.color_model[self.layer_n])
        self.next_down_distance = [999,999,999,999]
        self.second_down_distance=[999,999,999,999]
        self.point_x=0    #色模Ymax的x值
        self.point_y=0

        if self.color_true_times ==1 and self.board_ture==1:
            self.point_y=send.color_mask_subject_YMin[self.color_model[self.layer_n]][self.color_loc]
            for mp in range (0,320):
                if send.Label_Model[320*self.point_y+mp] == self.layer[self.layer_n]:
                    self.point_x=mp
                    break

        if self.layer_n>=2:
            #左左腳距離  #找要下的層
            for ll in range(self.knee,10,-1):      
                if self.return_real_down_board(ll,self.f_ll,self.layer_n-1):
                    self.next_down_distance[0] = self.knee - ll
                    break
            #左右腳距離
            for lr in range(self.knee,10,-1):
                if self.return_real_down_board(lr,self.f_lr,self.layer_n-1):
                    self.next_down_distance[1] = self.knee - lr
                    break

            #右左腳距離
            for rl in range(self.knee,10,-1):
                if self.return_real_down_board(rl,self.f_rl,self.layer_n-1):
                    self.next_down_distance[2] = self.knee - rl
                    break

            #右右腳距離
            for rr in range(self.knee,10,-1):
                if self.return_real_down_board(rr,self.f_rr,self.layer_n-1):
                    self.next_down_distance[3] = self.knee - rr
                    break

        else:
            #左左腳距離  #找要下的層
            for ll in range(self.knee,10,-1):      
                if self.return_real_board(ll,self.f_ll,self.layer_n-1):
                    self.next_down_distance[0] = self.knee - ll
                    break
            #左右腳距離
            for lr in range(self.knee,10,-1):
                if self.return_real_board(lr,self.f_lr,self.layer_n-1):
                    self.next_down_distance[1] = self.knee - lr
                    break

            #右左腳距離
            for rl in range(self.knee,10,-1):
                if self.return_real_board(rl,self.f_rl,self.layer_n-1):
                    self.next_down_distance[2] = self.knee - rl
                    break

            #右右腳距離
            for rr in range(self.knee,10,-1):
                if self.return_real_board(rr,self.f_rr,self.layer_n-1):
                    self.next_down_distance[3] = self.knee - rr
                    break


        #找下一板可踩空間
        if self.layer_n>=2:
            for ll_2 in range(ll,10,-1):     
                if self.return_real_board(ll_2,self.f_ll,self.layer_n-2):
                    self.second_down_distance[0] = ll-ll_2
                    break
            for lr_2 in range(lr,10,-1):
                if self.return_real_board(lr_2,self.f_lr,self.layer_n-2):
                    self.second_down_distance[1] = lr - lr_2
                    break
            
            for rl_2 in range(rl,10,-1):
                if self.return_real_board(rl_2,self.f_rl,self.layer_n-2):
                    self.second_down_distance[2] = rl - rl_2
                    break
            
            for rr_2 in range(rr,10,-1):
                if self.return_real_board(rr_2,self.f_rr,self.layer_n-2):
                    self.second_down_distance[3] = rr - rr_2
                    break
        else:
            self.second_down_distance=[999,999,999,999]



    def parallel_board_setup(self):#上板的角度調整
        # print('parallel_board_setup func')
        #條件要調整！！！
        # [0][1][2][3]都不能<0
        if(self.next_up_distance[0]<=self.back_dis or self.next_up_distance[1]<=self.back_dis or self.next_up_distance[2]<=self.back_dis or self.next_up_distance[3]<=self.back_dis):
            # 上板空間不夠
            if self.layer_n != 3 and (self.second_up_distance[0] < self.space_not_enough_up or self.second_up_distance[3] < self.space_not_enough_up) and (self.next_up_distance[0] < self.space_not_enough_up or self.next_up_distance[3] < self.space_not_enough_up):
                #print('space not enoughhhhhhhhhhh')
                # 在第一層
                if self.layer_n==1:
                    self.forward = -1000+FORWARD_CORRECTION
                    self.translation = 0+TRANSLATION_CORRECTION
                    self.up_theta_func()
                    print('space not enough move one')

                # 在第其他層
                else:
                    self.forward = -1000+FORWARD_CORRECTION
                    self.translation = 0+TRANSLATION_CORRECTION
                    self.up_theta_func()
                    print('space not enough move other')

            # 有出現再開
            # elif (self.next_up_distance[1]<self.next_up_distance[0]) and (self.next_up_distance[1]<self.next_up_distance[3]) and (self.next_up_distance.index(min(self.next_up_distance))==1) and (max(self.next_up_distance)-min(self.next_up_distance)>20):
            #     self.forward=-600+FORWARD_CORRECTION
            #     self.theta=THETA_CORRECTION
            #     self.translation = -1200+TRANSLATION_CORRECTION

            # elif (self.next_up_distance[2]<self.next_up_distance[0]) and (self.next_up_distance[2]<self.next_up_distance[3]) and (self.next_up_distance.index(min(self.next_up_distance))==2) and (max(self.next_up_distance)-min(self.next_up_distance)>20):
            #     self.forward=-600+FORWARD_CORRECTION
            #     self.theta=THETA_CORRECTION
            #     self.translation = 1200+TRANSLATION_CORRECTION
                


            # 不平行
            elif self.next_up_distance[3]-self.next_up_distance[0] > 5 or self.next_up_distance[0]-self.next_up_distance[3] >5:
                print("back back back back aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaback back back back")
                #self.forward=self.step_back
                self.forward=-500+FORWARD_CORRECTION
                self.up_theta_func()
                if self.theta>THETA_CORRECTION:
                    self.translation = -1000+TRANSLATION_CORRECTION
                elif self.theta<THETA_CORRECTION:
                    self.translation = 1000+TRANSLATION_CORRECTION
                else:
                    self.translation = TRANSLATION_CORRECTION

            



            # 進 back back 但可以繼續直走
            else:
                print("back back back back")
                self.forward=self.forward_min+FORWARD_CORRECTION
                self.translation = TRANSLATION_CORRECTION
                self.up_theta_func()
        
            # else:
            #     self.forward = -100+FORWARD_CORRECTION
            #     self.translation = self.translation
            #     self.up_theta_func()
            #     print('222222222222222222222222222')


        else :
            # 上板空間不夠
            if self.layer_n != 3  and (self.next_up_distance[0] < self.space_not_enough_up or self.next_up_distance[3] < self.space_not_enough_up):  #and (self.second_up_distance[0] < self.space_not_enough_up or self.second_up_distance[3] < self.space_not_enough_up)
                #print('space not enoughhhhhhhhhhh')
                # 在第一層
                if self.layer_n==1:
                    self.forward = self.step_back + FORWARD_CORRECTION
                    self.translation = 0+TRANSLATION_CORRECTION
                    self.up_theta_func()
                    print('space not enough move one')

                # 在第其他層
                else:
                    self.forward = self.step_back + FORWARD_CORRECTION
                    self.translation = 0+TRANSLATION_CORRECTION
                    self.up_theta_func()
                    print('space not enough move other')
            # 上板直角
            elif((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (self.next_up_distance[0]<70 or self.next_up_distance[1]<70 or self.next_up_distance[2]<70 or self.next_up_distance[3]<70):
                #腳要掉下去
                if self.next_up_distance[0]==999 or self.next_up_distance[0]==0:
                    self.forward=self.forward_min+FORWARD_CORRECTION
                    self.translation = -1000+TRANSLATION_CORRECTION
                    self.up_theta_func()
                    print('90 move right')
                elif self.next_up_distance[3]==999 or self.next_up_distance[3]==0:
                    self.forward=self.forward_min+FORWARD_CORRECTION
                    self.translation = 1000+TRANSLATION_CORRECTION
                    self.up_theta_func()
                    print('90 move left')
                else:
                    print("90")
                    self.up_board_90()
                    self.up_theta_func()
                
            #板上找不到板
            elif self.layer_n > 1 and self.next_up_distance[0]>250 and self.next_up_distance[3]>250:#數值我想測試
                #print("no")
                self.no_up_board()
                print("no up baord")
            else :
                #上紅板後
                if self.layer_n > 1:
                    if(self.next_up_distance[1]<=self.up_bd_2 and self.next_up_distance[2]<=self.up_bd_2):
                        self.forward=self.forward_min+FORWARD_CORRECTION
                        self.translation = TRANSLATION_CORRECTION
                        self.up_theta_func()
                        print("小前進")
                    elif(self.next_up_distance[1]<self.up_bd_3 or self.next_up_distance[2]<self.up_bd_3):
                        self.forward=self.forward
                        self.translation = TRANSLATION_CORRECTION
                        self.up_theta_func()
                        print("前進")
                    else:
                        self.forward=self.forward + self.forward_change
                        self.translation = TRANSLATION_CORRECTION
                        self.up_theta_func()
                        print("大前進")
                #上紅板前
                else :
                    if(self.next_up_distance[0]<=self.up_bd_2 and self.next_up_distance[3]<=self.up_bd_2):  # 小前進
                        self.forward=self.forward_min + FORWARD_CORRECTION
                        self.translation = TRANSLATION_CORRECTION
                        self.up_theta_func()
                        print("小前進")
                    elif(self.next_up_distance[1]<self.up_bd_3 or self.next_up_distance[2]<self.up_bd_3):   # 前進 
                        self.forward=self.forward_normal + FORWARD_CORRECTION
                        self.translation = TRANSLATION_CORRECTION
                        self.theta = THETA_CORRECTION
                        self.up_theta_func()
                        print("前進")
                    elif(self.next_up_distance[1]<self.up_bd_4) or (self.next_up_distance[2]<self.up_bd_4): #大前進
                        self.forward=self.forward_big + FORWARD_CORRECTION
                        self.translation = TRANSLATION_CORRECTION
                        self.theta = THETA_CORRECTION
                        self.up_theta_func()
                        print("大前進")
                    else:
                        self.forward=self.forward_super + FORWARD_CORRECTION                           # 超大前進
                        self.translation=TRANSLATION_CORRECTION
                        self.theta = THETA_CORRECTION
                        # self.up_theta_func()
                        print("超大前進")
            


    def down_parallel_board_setup(self): #下板角度調整       
        if  self.layer_n ==3:
            print("晏宇")
            # self.down_max = self.next_down_distance.index(max(self.next_down_distance))
            # self.next_down_max = self.next_down_distance.index(max(self.next_down_distance))
            # self.big_to_small_next_down_distance = sorted(self.next_down_distance,reverse = True)
            # if((self.next_down_distance[0]<=self.back_dis or self.next_down_distance[0]==999) and self.second_down_distance:
            if (self.next_down_distance[1] < self.down_bd_1+5 or self.next_down_distance[2] < self.down_bd_1+5) and (abs(self.next_down_distance[2]-self.next_down_distance[1])<self.feet_distance_1):
                print("晏宇不會走")
                self.forward = 300 + FORWARD_CORRECTION
                self.translation = TRANSLATION_CORRECTION
                self.down_theta_func()
            elif self.next_down_distance[0]<=5:
                print("晏宇y")
                self.forward = 0+FORWARD_CORRECTION
                self.translation = -1200+TRANSLATION_CORRECTION
                self.theta = -3
            elif self.next_down_distance[3]<=5 :
                print("晏宇好走")
                self.forward = 0+FORWARD_CORRECTION
                self.translation = 1200+TRANSLATION_CORRECTION
                self.theta = -1
            elif (self.second_down_distance[0] < 70 and self.next_down_distance[0]<=60):
                print("晏宇y")
                self.forward = 0+FORWARD_CORRECTION
                self.translation = -1200+TRANSLATION_CORRECTION
                self.theta = -3
            elif (self.second_down_distance[3] < 70 and self.next_down_distance[3]<=60):
                print("晏宇好走")
                self.forward = 0+FORWARD_CORRECTION
                self.translation = 1200+TRANSLATION_CORRECTION
                self.theta = -1

            else:
                print("我直走")
                if self.next_down_distance[1] <= self.down_bd_2 and self.next_down_distance[2] <= self.down_bd_2:#距離小於30的時候
                    self.forward = self.down_forward_1
                    self.translation = TRANSLATION_CORRECTION
                    self.down_theta_func()
                elif self.next_down_distance[1] <=self.down_bd_3 and self.next_down_distance[2] <= self.down_bd_3:#距離小於60的時候
                    self.forward = self.down_forward_2
                    self.down_theta_func()
                elif self.next_down_distance[1] > self.down_bd_4 or self.next_down_distance[2] > self.down_bd_4:#距離在大於60的時候
                    self.forward = self.down_forward_3
                    self.translation = TRANSLATION_CORRECTION
                    self.down_theta_func()
                


        # 不在第三板
        elif self.layer_n!=3:
            if(self.next_down_distance[0]<=self.back_dis or self.next_down_distance[1]<=self.back_dis or self.next_down_distance[2]<=self.back_dis or self.next_down_distance[3]<=self.back_dis):  
                print("我進back")   
                # if ((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 or self.next_down_distance.index(max(self.next_down_distance)) == 1 or self.next_down_distance.index(max(self.next_down_distance)) == 2:
                #     print("back 90")
                #     self.down_board_90()
                #     self.forward=self.step_back
                # if self.next_down_distance[0]==0  and (self.next_down_distance[1]>self.back_dis or self.next_down_distance[2]>self.back_dis) and self.next_down_distance[3]>self.back_dis:
                #     print("right 0")
                #     self.forward=self.step_back
                #     self.translation = -800+TRANSLATION_CORRECTION
                #     self.up_theta_func()
                # elif self.next_down_distance[3]==0 and (self.next_down_distance[1]>self.back_dis or self.next_down_distance[2]>self.back_dis) and self.next_down_distance[0]>self.back_dis:
                #     print("left 0")
                #     self.forward=self.step_back
                #     self.translation = 1200+TRANSLATION_CORRECTION
                #     self.up_theta_func()

                if self.layer_n != 1 and (self.second_down_distance[0] < self.sec_space_not_enough_down or self.second_down_distance[1] < self.sec_space_not_enough_down or self.second_down_distance[2] < self.sec_space_not_enough_down or self.second_down_distance[3] < self.sec_space_not_enough_down) and (self.next_down_distance[0] < self.space_not_enough_down or self.next_down_distance[3] < self.space_not_enough_down):
                    self.counter +=1
                    self.not_enough_flag=1
                else:
                    self.counter =0
                    self.not_enough_flag=0



                if max(self.next_down_distance)-min(self.next_down_distance)>25:
                    if self.next_down_distance.index(max(self.next_down_distance)) == 3 :
                        print("index : ",self.next_down_distance.index(min(self.next_down_distance)))
                        print("back right right right")
                        self.forward=self.step_back
                        self.translation = -1200+TRANSLATION_CORRECTION
                        self.down_theta_func()
                    elif self.next_down_distance.index(max(self.next_down_distance)) == 0 :
                        print("index : ",self.next_down_distance.index(min(self.next_down_distance)))
                        print("back left left left")
                        self.forward=self.step_back
                        self.translation = 1200+TRANSLATION_CORRECTION
                        self.down_theta_func()
                    else:
                        self.forward=self.step_back
                        self.translation = TRANSLATION_CORRECTION
                        self.down_theta_func()
            

                else:
                    #print("gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg")
                    self.forward=self.down_forward_1-200
                    self.translation = TRANSLATION_CORRECTION
                    self.down_theta_func()
            else:
                # 下板空間不夠
                if self.layer_n != 1 and (self.second_down_distance[0] < self.sec_space_not_enough_down or self.second_down_distance[1] < self.sec_space_not_enough_down or self.second_down_distance[2] < self.sec_space_not_enough_down or self.second_down_distance[3] < self.sec_space_not_enough_down) and (self.next_down_distance[0] < self.space_not_enough_down or self.next_down_distance[3] < self.space_not_enough_down):
                    self.counter +=1
                    self.not_enough_flag=1

                    if self.counter >= self.counter_max:
                        self.forward = self.step_back

                        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        if self.second_down_distance[0]>self.sec_space_not_enough_down and self.second_down_distance[0]<999:
                            self.translation = 1200+TRANSLATION_CORRECTION 
                            self.down_theta_func
                            print('counter>max move left')
                            print('vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv')

                        else:
                            self.translation = 0+TRANSLATION_CORRECTION
                            self.theta = -12 +THETA_CORRECTION
                            print('counter>max big turn')



                    # else:
                    #     self.forward = self.step_back_2
                    #     self.translation = 0+TRANSLATION_CORRECTION

                    #     #直接設定遇到空間不夠轉哪邊 
                    #     # self.theta = 8 +THETA_CORRECTION  
                    #     # 讓他透過哪邊空間大設定轉向
                    #     if self.second_down_distance[0]>self.second_down_distance[3]:
                    #         self.theta = 8+THETA_CORRECTION
                    #         print('counter<max turn right') 
                        
                    #     else:
                    #         self.theta = -8+THETA_CORRECTION
                    #         print('counter<max turn left')

                elif self.next_down_distance[0] <= self.down_bd_2 and self.next_down_distance[3] <= self.down_bd_2:#距離小於30的時候
                    self.counter =0
                    self.not_enough_flag=0

                    self.forward = self.down_forward_1
                    self.translation = TRANSLATION_CORRECTION
                    self.down_theta_func()
                    print('normal 1')
                elif self.next_down_distance[0] <=self.down_bd_3 and self.next_down_distance[3] <= self.down_bd_3:#距離小於60的時候
                    self.counter =0
                    self.not_enough_flag=0

                    self.forward = self.down_forward_2
                    self.translation = TRANSLATION_CORRECTION
                    self.down_theta_func()
                    print('normal 2')
                elif self.next_down_distance[0] > self.down_bd_4 or self.next_down_distance[3] > self.down_bd_4:#距離在大於60的時候
                    self.counter =0
                    self.not_enough_flag=0

                    self.forward = self.down_forward_3
                    self.translation = TRANSLATION_CORRECTION
                    self.down_theta_func()
                    print('normal 3')

                else:
                    self.counter =0
                    self.not_enough_flag=0

                    self.forward = self.down_forward_3
                    self.translation = TRANSLATION_CORRECTION
                    self.down_theta_func()
                    print('normal 4')



        

    def up_board(self): #要上板了
        # print('up_board_func')
        if ((self.next_up_distance[1]<self.up_bd_1) and (self.next_up_distance[2]<self.up_bd_1)) : #and (abs(self.next_up_distance[3]-self.next_up_distance[0])<self.feet_distance_1) and (self.second_up_distance[0] > self.space_not_enough_up and self.second_up_distance[3] > self.space_not_enough_up)
            if self.stop_flag==0 and self.up_board_flag==0:
                print('ready upboard')
                self.upboard_start=time.time()
                while(self.upboard_end-self.upboard_start>2):
                    send.sendContinuousValue(-500,self.translation,0,self.theta,0)
                    print(self.upboard_end-self.upboard_start)
                # send.sendContinuousValue(FORWARD_CORRECTION,TRANSLATION_CORRECTION,0,THETA_CORRECTION,0)
                # time.sleep(2)
                self.forward=0
                self.translation=0
                self.theta=0
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(5)
                send.sendBodySector(24)#上板前微調站姿
                send.sendSensorReset()
                time.sleep(2)
                self.stop_flag=1
                self.up_board_flag=1
                self.next_board()
                self.next_up_distance = [999,999,999,999]
                self.second_up_distance = [999,999,999,999]
                send.sendBodyAuto(self.LCup_x,0,0,0,2,0)
                print("up board !!!!!!!")
                time.sleep(5)
                send.sendBodySector(29)#這是基本站姿的磁區
                time.sleep(1)
                send.sendBodySector(299)#縮雙腳
                time.sleep(1)
                send.sendBodySector(22)#縮左腳
                time.sleep(1)
                            
        else:
            self.parallel_board_setup()
            self.speed_limit()
            send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)

    
    def down_board(self): #要下板了
        # print('down_board_func')
        # if (self.next_down_distance[1] < self.down_bd_1 or self.next_down_distance[2] < self.down_bd_1) and (abs(self.next_down_distance[3]-self.next_down_distance[0])<self.feet_distance_1) and (self.second_down_distance[0] >= self.sec_space_not_enough_down and self.second_down_distance[3] >= self.sec_space_not_enough_down):
        if (self.next_down_distance[1] < self.down_bd_1 or self.next_down_distance[2] < self.down_bd_1) and (abs(self.next_down_distance[3]-self.next_down_distance[0])<self.feet_distance_1) and (max(self.next_down_distance) - min( self.next_down_distance)<20) and (self.second_down_distance[0] >= self.sec_space_not_enough_down and self.second_down_distance[3] >= self.sec_space_not_enough_down):
            if self.stop_flag == 0 and self.up_board_flag == 0:
                print('ready downboard')
                self.forward=0
                self.translation=0
                self.theta=0
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(4)
                send.sendSensorReset()
                send.sendBodySector(25)#下板前微調站姿
                # if self.layer_n == 1:
                #     send.sendBodySector(2)
                # else:
                #     send.sendBodySector(2)
                # time.sleep(3)
                self.stop_flag = 1
                self.up_board_flag = 1
                self.next_board()
                self.next_down_distance = [999,999,999,999]
                self.second_down_distance = [999,999,999,999]
                send.sendBodyAuto(self.LCdown_x,0,0,0,3,0)
                time.sleep(5)
                send.sendBodySector(29)
                print("down board !!!!!!!")
                time.sleep(5)
                send.sendBodySector(29)#這是基本站姿的磁區
                time.sleep(1)
                send.sendBodySector(299)#縮雙腳
                time.sleep(1)
                send.sendBodySector(22)#縮左腳
                time.sleep(1)
        else:
            self.down_parallel_board_setup()
            send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)


    def no_up_board(self):#看不到板子時,n>1
        #print("color_loc]",self.color_loc)
        #print("size",send.color_mask_subject_size[self.layer_n][self.color_loc])
        #self.up_mask=send.color_mask_subject_cnts[self.color_model[self.layer_n]]
        if self.board_ture==0:
            # self.up_mask2=send.color_mask_subject_cnts[self.color_model[self.layer_n-2]]#我站在紅板,沒看到黃板,看有沒有綠板
            # if self.up_mask2==0:
            #     #print("nnnnnnnnnnnnnnnnnnnnnnnnnnnn")
            #     self.forward = 200 + FORWARD_CORRECTION
            #     self.translation = TRANSLATION_CORRECTION
            #     self.theta = THETA_CORRECTION
            # else:
            #     #print("ggggggggggggggggggggggggggggggggggg")#有綠板
            #     #self.find_real_board_model(self.color_model[self.layer_n-2])
            #     self.previous_board_x_center=send.color_mask_subject_X[self.color_model[self.layer_n-2]][0]
            #     if self.previous_board_x_center<self.f_mid:
            #         self.forward = 0 + FORWARD_CORRECTION
            #         self.translation = -200+TRANSLATION_CORRECTION
            #         self.theta = -5 + THETA_CORRECTION
            #         # print("cant find board : turn left")
            #         # send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)
            #     elif self.previous_board_x_center>self.f_mid and self.previous_board_x_center<999:
            self.forward = 0 + FORWARD_CORRECTION
            self.translation = -200+TRANSLATION_CORRECTION
            self.theta = 3 + THETA_CORRECTION 

        else:
            #print("有囉")
            #print(self.color_loc)
            self.next_board_x_center=send.color_mask_subject_X[self.color_model[self.layer_n]][self.color_loc]
            if self.next_board_x_center<self.f_mid:
                self.forward = self.forward + self.forward_change
                self.translation = TRANSLATION_CORRECTION
                self.theta = 5 + THETA_CORRECTION
                # print("cant find board : turn left")
                # send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)
            elif self.next_board_x_center>self.f_mid and self.next_board_x_center<999:
                self.forward = self.forward + self.forward_change
                self.translation = TRANSLATION_CORRECTION
                self.theta = 5 + THETA_CORRECTION            
                #print("cant find board : turn right")
                #send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)

    def up_board_90(self): #上板90度狀況
        # print('up_board_90 func')
        #self.find_real_board_model(self.color_model[self.layer_n])
        self.m_xmin=send.color_mask_subject_XMin[self.color_model[self.layer_n]][self.color_loc]
        self.m_xmax=send.color_mask_subject_XMax[self.color_model[self.layer_n]][self.color_loc]
        # if(self.m_xmax-self.point_x>self.point_x-self.m_xmin):
        #     self.forward=FORWARD_CORRECTION
        #     self.translation=-1200+TRANSLATION_CORRECTION
        #     self.theta= THETA_CORRECTION   #為什麼這邊是0
        #     #print("move  right 90")
        # elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin):
        #     self.forward=FORWARD_CORRECTION
        #     self.translation=1200+TRANSLATION_CORRECTION
        #     self.theta=THETA_CORRECTION   #為什麼這邊是0
        #     #print("move  left 90") 

        if(self.next_up_distance[0]>self.next_up_distance[3]) and (self.next_up_distance[0]>70 and self.next_up_distance[3]>70):
            self.forward=self.step_back+FORWARD_CORRECTION
            self.translation=-500+TRANSLATION_CORRECTION
            self.theta= THETA_CORRECTION   
            print("move  left 90")
        elif (self.next_up_distance[3]>self.next_up_distance[0]) and (self.next_up_distance[0]>70 and self.next_up_distance[3]>70):
            self.forward=FORWARD_CORRECTION
            self.translation=500+TRANSLATION_CORRECTION
            self.theta= THETA_CORRECTION   
            print("move  right 90")
        else:
            if(self.m_xmax-self.point_x>self.point_x-self.m_xmin):
                self.forward=self.step_back+FORWARD_CORRECTION
                self.translation=-1000+TRANSLATION_CORRECTION
                self.theta= THETA_CORRECTION   
                print("move  right 90 not use distance")
            elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin):
                self.forward=FORWARD_CORRECTION
                self.translation=1000+TRANSLATION_CORRECTION
                self.theta=THETA_CORRECTION   
                print("move  left 90 not use distance")


    def down_board_90(self): #下板90度狀況
        #self.find_real_board_model(self.color_model[self.layer_n])
        self.m_xmin=send.color_mask_subject_XMin[self.color_model[self.layer_n]][self.color_loc]
        self.m_xmax=send.color_mask_subject_XMax[self.color_model[self.layer_n]][self.color_loc]
        if(self.m_xmax-self.point_x>self.point_x-self.m_xmin):
            print("down 90 left turn")
            self.forward=FORWARD_CORRECTION
            self.translation=TRANSLATION_CORRECTION
            self.theta=THETA_CORRECTION
            #print("move  right 90")
        elif(self.m_xmax-self.point_x<self.point_x-self.m_xmin):
            print("down 90 right turn")
            self.forward=FORWARD_CORRECTION
            self.translation=TRANSLATION_CORRECTION
            self.theta=THETA_CORRECTION
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
        self.color_times=send.color_mask_subject_cnts[find]       #搜尋物件個數
        if self.color_times != 0:
            self.color_true_times =1                                
            for i in range(self.color_times):                     
                self.color_size = send.color_mask_subject_size[find][i]   #標記物件大小
                print("全部size",self.color_size)
                if self.color_size >10000:#錢幣大小(要測試)
                    #print("yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
                    self.color_loc = i                                    #紀錄第i個物體為板子
                    self.board_ture=1
                    break
                else:
                    self.board_ture=0
        else:
            self.color_true_times = 0#無用
        

    def up_theta_func(self):
        self.up_feet_distance=self.next_up_distance[3]-self.next_up_distance[0]
        if   (self.up_feet_distance>0):
            self.decide_theta = self.l_theta
            print('turn left')
        elif (self.up_feet_distance<0):
            self.decide_theta = self.r_theta
            print('turn right')
        else:
            print('walk forward')


        if  (abs(self.up_feet_distance))>self.feet_distance_3:               #斜率過大,角度給最大
            self.theta =  5*self.decide_theta
            self.translation = 1000*self.decide_theta
        elif(abs(self.up_feet_distance))>self.feet_distance_2:               #斜率較大,修正值較大
            self.theta = self.theta + 3*self.decide_theta + THETA_CORRECTION
        elif(abs(self.up_feet_distance))>self.feet_distance_1:               #斜率較小,修正值較小
            self.theta = self.theta + 1*self.decide_theta + THETA_CORRECTION
        else:
            self.theta = 0+THETA_CORRECTION

        

    def down_theta_func(self):
        self.down_feet_distance=self.next_down_distance[2]-self.next_down_distance[1]
        
        if   (self.down_feet_distance>0):
            self.decide_theta = self.l_theta
            print('turn left')
        elif (self.down_feet_distance<0):
            self.decide_theta = self.r_theta
            print('turn right')
        else:
            print('walk forward')

        if  (abs(self.down_feet_distance))>self.feet_distance_3:
            self.theta =  5*self.decide_theta
        elif(abs(self.down_feet_distance))>self.feet_distance_2:
            self.theta = self.theta + 3*self.decide_theta + THETA_CORRECTION
            print("big theta")
        elif(abs(self.down_feet_distance))>self.feet_distance_1:
            self.theta = self.theta + 1*self.decide_theta + THETA_CORRECTION
            print("small theta")
        else:
            self.theta = 0+THETA_CORRECTION

        

    def speed_limit(self): # 速度限制 
        if self.forward > 2000:
            self.forward = 2000
        elif self.forward < -2000:
            self.forward = -2000
        if self.translation > 1000:
            self.translation = 1000
        elif self.translation < -1000:
            self.translation = -1000
        if self.theta > 5:
            self.theta = 5
        elif self.theta < -5:
            self.theta = -5
    
    def return_distance_flag(self):
        if ((self.next_up_distance[1]<self.up_bd_1) and (self.next_up_distance[2]<self.up_bd_1)) and (abs(self.next_up_distance[3]-self.next_up_distance[0])<self.feet_distance_1) and (self.second_up_distance[0] > self.space_not_enough_up and self.second_up_distance[3] > self.space_not_enough_up):
            self.ready_to_updown_board = True
        elif (self.next_up_distance[0]<=self.up_bd_2 and self.next_up_distance[3]<=self.up_bd_2):
            self.first_judge_line = True
        elif (self.next_up_distance[0]<=self.up_bd_4 and self.next_up_distance[3]<=self.up_bd_3):
            self.secend_judge_line = True
        elif (self.next_up_distance[0]<=self.up_bd_4 and self.next_up_distance[3]<=self.up_bd_4):
            self.third_judge_line = True
# ////////////////只針對10個點拿出來//////////////////   
    def return_real_board(self,y,x,layer):
        real_distance_flag=0
        real_distance_flag= (send.Label_Model[320*y+x] == self.layer[layer])
        if real_distance_flag==1:
            for i in range(1,11):
                real_distance_flag=real_distance_flag and send.Label_Model[320*(y-i)+x] == self.layer[layer]
                if real_distance_flag==0:
                    break
        return real_distance_flag


    def return_real_down_board(self,y,x,layer):
        real_distance_flag=0
        real_distance_flag= (send.Label_Model[320*y+x] == self.layer[layer]) or (send.Label_Model[320*y+x] == self.layer[layer-1])
        if real_distance_flag==1:
            for i in range(1,11):
                real_distance_flag=real_distance_flag and (send.Label_Model[320*(y-i)+x] == self.layer[layer] or (send.Label_Model[320*y+x] == self.layer[layer-1]))
                if real_distance_flag==0:
                    break
        return real_distance_flag




    def print_state(self): 
            print("/////////////////////////////////////////////////////////")
            print("counter         :  ",self.counter)
            print("not enough flag :  ",self.not_enough_flag)
            print("/////////////////////////////////////////////////////////")
    # //////////////////                     test                           //////////////////////////////
            # print("real board model             : ",self.board_model)
            # print("board size                   : ",send.color_mask_subject_size[self.color_model[self.layer_n]][self.board_model])
    # ////////////////////////////////////////////////////////////////////////////////////////////////////  
            print("forward                        : ",self.forward )
            print("translation                       : ",self.translation)
            print("theta                        : ",self.theta)
            print("point   y                    : ",self.point_y)
            print("point   x                    : ",self.point_x)
            print("stop_flag                    : ",self.stop_flag)
            print("up board flag                : ",self.up_board_flag)
           
            print("layer_now                    : ",self.layer_n)
            if(self.direction==0):
                print("direction                    :  up board")
                print("next_up_distance                  : ",self.next_up_distance)
                print("second_up_distance             : ",self.second_up_distance)
                print('second_up_distance[0]          : ',self.second_up_distance[0])
                print("next board x point           : ",self.next_board_x_center)


                # if(self.next_up_distance[0]<=self.back_dis or self.next_up_distance[1]<=self.back_dis or self.next_up_distance[2]<=self.back_dis or self.next_up_distance[3]<=self.back_dis):
                    
                #     if self.next_up_distance[3]-self.next_up_distance[0] > 5 or self.next_up_distance[0]-self.next_up_distance[3] >5:
                #         print("back back back back aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaback back back back")
                #     else:
                #         print("back back back back")
                        


                # else :
                #     # 空間不夠
                #     if self.layer_n != 3 and (self.second_up_distance[0] < self.space_not_enough_up or self.second_up_distance[3] < self.space_not_enough_up) and (self.next_up_distance[0] < self.space_not_enough_up or self.next_up_distance[3] < self.space_not_enough_up):
                        
                #         if self.second_up_distance[0]>self.second_up_distance[3] :
                #             print('space not enoughhhhhhhhhhh left')
                #         elif self.second_up_distance[3]>self.second_up_distance[0]:
                #             print('space not enoughhhhhhhhhhh right')

                    

                    
                #     # 上板直角
                #     elif((self.f_ll-self.point_x)*(self.f_rr-self.point_x))<0 and (self.next_up_distance[0]<90 or self.next_up_distance[1]<90 or self.next_up_distance[2]<90 or self.next_up_distance[3]<90):
                #         #腳要掉下去
                #         if self.next_up_distance[0]==999 or self.next_up_distance[0]==0:
                #             print('90 move right')
                #         elif self.next_up_distance[3]==999 or self.next_up_distance[3]==0:
                #             print('90 move left')
                #         else:
                #             print("90")
                        
                #     #找不到板
                #     elif self.layer_n > 1 and self.next_up_distance[0]>250 and self.next_up_distance[3]>250:#數值我想測試
                #         print("no up baord")
                #     else :
                #         #上紅板後
                #         if self.layer_n > 1:
                #             if(self.next_up_distance[0]<=self.up_bd_2 and self.next_up_distance[3]<=self.up_bd_2):#30
                #                 print("forward_1")
                #             elif(self.next_up_distance[1]<self.up_bd_3 or self.next_up_distance[2]<self.up_bd_3):
                #                print("forward_2")
                #             else:
                #                print("forward_2")
                #         #上紅板前
                #         else :
                #             if(self.next_up_distance[0]<=self.up_bd_2 and self.next_up_distance[3]<=self.up_bd_2):#30
                #                 print("forward_1")
                #             elif(self.next_up_distance[1]<self.up_bd_3 or self.next_up_distance[2]<self.up_bd_3):
                #                 print("forward_2")
                #             elif(self.next_up_distance[1]<self.up_bd_4) or (self.next_up_distance[2]<self.up_bd_4):
                #                 print("forward_3")
                #             else:
                #                 print("forward_5")
                            

                
            elif(self.direction==1):
                print("direction                    :  down_board")
                print("down distance                : ",self.next_down_distance)
                print("second_down_distance           : ",self.second_down_distance)
                print('second_down_distance[0]        : ',self.second_down_distance[0])
                

                # # 下板距離不夠
                # if(self.next_down_distance[0]<=self.back_dis or self.next_down_distance[1]<=self.back_dis or self.next_down_distance[2]<=self.back_dis or self.next_down_distance[3]<=self.back_dis):       
                #     if max(self.next_down_distance)-min(self.next_down_distance)>25 and self.counter < self.counter_max:
                #         if (self.next_down_distance.index(max(self.next_down_distance)) == 3):
                #             print("index : ",self.next_down_distance.index(min(self.next_down_distance)))
                #             print("back right right right")
                           
                #         elif self.next_down_distance.index(max(self.next_down_distance)) == 0:
                #             print("index : ",self.next_down_distance.index(min(self.next_down_distance)))
                #             print("back left left left")
                           
                #     elif self.counter > self.counter_max and self.not_enough_flag==1:
                #         print('back not enough big turn')
                #     # 腳已經超過板
                #     elif (self.next_down_distance.count(999)+self.next_down_distance.count(0))>=2:
                #         print('over board')
                        
                #     else:
                #         print(" down back back back bcak")
                        
                
                    
                # else:
                   
                #     # 空間不夠
                #     if self.layer_n != 1 and (self.second_down_distance[0] < self.sec_space_not_enough_down or self.second_down_distance[1] < self.sec_space_not_enough_down or self.second_down_distance[2] < self.sec_space_not_enough_down or self.second_down_distance[3] < self.sec_space_not_enough_down) and (self.next_down_distance[0] < self.space_not_enough_down or self.next_down_distance[3] < self.space_not_enough_down):
                        
                #         if self.layer_n == 3 and self.counter < self.counter_max:
                           
                #             if self.second_down_distance[0]>self.second_down_distance[3]:
                #                 print('layer 3 counter<max turn left')    
                #             else:
                #                 print('layer 3 counter<max turn right')
                #         elif self.counter >= self.counter_max:
                #             if self.second_down_distance[0]>self.sec_space_not_enough_down :
                #                 print('counter>max move left')

                #             elif self.second_down_distance[3]>self.sec_space_not_enough_down:
                #                 print('counter>max move right') 
                #             else:
                #                 print('counter>max big turn') 

                #         else:
                            
                #             #直接設定遇到空間不夠轉哪邊 
                #             # self.theta = 8 +THETA_CORRECTION 
                #             # print("counter<max big turn by input")
                #             # 讓他透過哪邊空間大設定轉向
                #             if self.second_down_distance[0]>self.second_down_distance[3]:
                #                 print('counter<max move right') 
                            
                #             else:
                #                 print('counter<max move left')
                        

                #     elif self.next_down_distance[0] <= self.down_bd_2 and self.next_down_distance[3] <= self.down_bd_2:#距離小於30的時候
                #         print('normal <30')
                #     elif self.next_down_distance[0] <=self.down_bd_3 and self.next_down_distance[3] <= self.down_bd_3:#距離小於60的時候
                #         print('normal <60')

                    
                #     else:
                #         # 空間大的時候如果能直走就直走
                #         if self.layer_n==3:
                #             print('layer3 forward')
                #         else:
                #             print('other layer normal')
