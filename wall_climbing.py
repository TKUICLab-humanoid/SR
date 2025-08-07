#!/usr/bin/env python3
#coding=utf-8
import sys
import rospy
import numpy as np
import math
from Python_API import Sendmessage

#--校正量--#
#前進量校正
FORWARD_CORRECTION         = -300
#平移校正
TRANSLATION_CORRECTION     = -100
#旋轉校正
THETA_CORRECTION           = 0
#基礎變化量(前進&平移)
BASE_CHANGE                = 200      

# ---微調站姿開關---#
STAND_CORRECT_CW           = False                 #sector(33) CW_stand微調站姿
DRAW_FUNCTION_FLAG         = True                  #影像繪圖開關
TARGET_COLOAR              = 'Red'                  #'Orange' 'Yellow' 'Blue' 'Green' 'Black' 'Red' 'White'       


#------------------#
HEAD_HORIZONTAL            = 2300                  #頭水平
HEAD_VERTICAL              = 2450                  #頭垂直 #30cm2150

#判斷值
# FOOTLADDER_LINE            = 215                   #上梯基準線
TARGET_LINE                = 50 #攀岩基準線

FIRST_FORWORD_CHANGE_LINE  = 0                    #小前進判斷線
SECOND_FORWORD_CHANGE_LINE = 0                    #前進判斷線
THIRD_FORWORD_CHANGE_LINE  = 0                   #大前進判斷線
UP_LADDER_DISTANCE         = 0                    #最低上板需求距離

#前後值
BACK_MIN                   = -300                  #小退後
FORWARD_MIN                = 0                  #小前進 300
FORWARD_NORMAL             = 800                  #前進
FORWARD_BIG                = 1000                  #大前進

#平移值
TRANSLATION_BIG            = 800                  #大平移
TRANSLATION_NORMAL         = 400
#旋轉值
# THETA_MIN                  = 3                     #小旋轉
THETA_NORMAL               = 1                    #旋轉
# THETA_BIG                  = 5                     #大旋轉

#左基礎參數
LEFT_THETA                 = 1
#右基礎參數
RIGHT_THETA                = -1

BLUE_MID = 65
BLUE_ERROR = 2

ROUTE_PLAN_FLAG = True
ROUTE_PLAN = [1000,300,0,3] #[x,y,theta,sec]

send       = Sendmessage()
class WallClimbing:
#CW主策略
    def __init__(self):
        self.target = ObjectInfo(TARGET_COLOAR,'Target')
        self.init()
        # self.STAND_CORRECT_CW = STAND_CORRECT_CW
        self.blue_x_middle = 0
        self.blue_y_middle = 0
        self.moveTotal = 0
        self.head = True
    def main(self,strategy):
        # send.sendHeadMotor(1, self.head_Horizontal, 100)#水平
        # send.sendHeadMotor(2, self.head_Vertical, 100)#垂直
        if DRAW_FUNCTION_FLAG:
            self.draw_function()
        sys.stdout.write("\033[J")
        sys.stdout.write("\033[H")
        rospy.loginfo('________________________________________\033[K')
        rospy.loginfo(f'x: {self.now_forward} ,y: {self.now_translation} ,theta: {self.now_theta}\033[K')
        rospy.loginfo(f'Goal_x: {self.forward} ,Goal_y: {self.translation} ,Goal_theta: {self.theta}\033[K')
        rospy.loginfo(f"機器人狀態: {self.state}\033[K")
        # rospy.loginfo(f"距離梯: {(FOOTLADDER_LINE - self.lower_blue_ymax)}\033[K")
        rospy.loginfo(f"藍色中心: { (send.color_mask_subject_XMin[self.target.color][0]+send.color_mask_subject_XMax[self.target.color][0])/2}\033[K")
        rospy.loginfo(f"blue_middle: x:{ self.blue_x_middle} y:{self.blue_y_middle}\033[K")
        rospy.loginfo(f"head {int(self.now_head_Vertical)}\033[K")
        rospy.loginfo(f"head {self.blue_y_middle - TARGET_LINE}\033[K")
        rospy.loginfo(f"move {self.moveTotal}\033[K")
        
        rospy.loginfo('￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣')
        
        if strategy == "Wall_Climb_off":
        #關閉策略,初始化設定
        
            if not self.walkinggait_stop:
                rospy.loginfo("🔊CW parameter reset\033[K")
                self.init()

                send.sendBodyAuto(0,0,0,0,1,0)
                send.sendSensorReset(1,1,1)              #IMUreset
                rospy.sleep(2)
                send.sendBodySector(29)             #基礎站姿磁區
                rospy.sleep(1.5)
                # if STAND_CORRECT_CW:
                #     send.sendBodySector(30)             #CW基礎站姿調整磁區
                #     STAND_CORRECT_CW = False 
                
                rospy.loginfo("reset🆗🆗🆗\033[K")
                self.imu_reset = True
            send.sendHeadMotor(1,self.head_Horizontal,100)  #水平
            send.sendHeadMotor(2,self.head_Vertical,100)    #垂直
            self.init()
            self.find_ladder()
            # self.keep_head()
            rospy.loginfo("turn off\033[K")
            self.walkinggait_stop = True

        elif strategy == "Wall_Climb_on":
            send.sendWalkParameter('send',\
                        walk_mode = 1,\
                        com_y_shift = -1.5,\
                        y_swing = 4.5,\
                        period_t = 270,\
                        t_dsp = 0,\
                        base_default_z = 1.5,\
                        com_height = 29.5,\
                        stand_height = 23.5,\
                        back_flag=False)
            send.sendWalkParameter('send',\
                        walk_mode = 1,\
                        com_y_shift = -1.5,\
                        y_swing = 4.5,\
                        period_t = 270,\
                        t_dsp = 0,\
                        base_default_z = 1.5,\
                        com_height = 29.5,\
                        stand_height = 23.5,\
                        back_flag=True)                
        #開啟CW策略 
            if self.state != 'cw_finish':
                # if self.STAND_CORRECT_CW:
                #     send.sendBodySector(102)             #CW基礎站姿調整磁區
                #     while not send.execute:
                #         rospy.logdebug("站立姿勢\033[K")
                #     send.execute = False
                #     self.STAND_CORRECT_CW = False
                #     rospy.sleep(2)
                if self.imu_reset:
                    send.sendSensorReset(1,1,1)
                    send.sendHeadMotor(1, self.head_Horizontal, 100)#水平
                    send.sendHeadMotor(2, self.head_Vertical, 100)#垂直
                    if STAND_CORRECT_CW:
                        send.sendBodySector(102)             
                        while not send.execute:
                            rospy.logdebug("站立姿勢調整\033[K")
                        send.execute = False
                        rospy.sleep(1)                    
                    send.sendBodyAuto(0,0,0,0,1,0)
                    self.route_plan()
                    self.imu_reset = False

                rospy.loginfo(f"blue ymax: {self.lower_blue_ymax}\033[K")
                self.find_ladder()
                if(self.head):
                    self.keep_head()
                self.walkinggait(motion=self.edge_judge(strategy))
            self.walkinggait_stop = False
                    
    def init(self):
        #狀態
        self.state                 = '停止'
        
        #步態啟動旗標
        self.walkinggait_stop      = True  
        
        #設定頭部馬達
        self.head_Horizontal       = HEAD_HORIZONTAL
        self.head_Vertical         = HEAD_VERTICAL
        self.now_head_Vertical = self.head_Vertical
        
        #imu_reast
        self.imu_reset             = True

        #步態參數
        self.forward               = FORWARD_NORMAL + FORWARD_CORRECTION
        self.translation           = 0              + TRANSLATION_CORRECTION
        self.theta                 = 0              + THETA_CORRECTION
        self.now_forward           = 0 
        self.now_translation       = 0
        self.now_theta             = 0

        self.new_target_xmin       = 0
        self.new_target_xmax       = 0  
        self.lower_blue_ymax       = 240
        self.lower_blue_xmax       = 320
        self.moveTotal = 0
        self.head = True

    def find_ladder(self):
    #獲取梯子資訊、距離資訊
        # self.ladder.update()
        self.lower_blue_ymax = 120
        self.lower_blue_xmax = 160
        self.new_target_xmax = 320
        self.new_target_xmin = 320
        self.new_target_ymax = 240
        self.new_target_ymin = 240
        self.blue_x_middle = 160
        self.blue_y_middle = 160
        rospy.loginfo(f"blue mask subject cnts: {send.color_mask_subject_cnts[self.target.color]}\033[K")
        sys.stdout.write("\033[K")
        #-------距離判斷-------#
        for blue_cnt in range (send.color_mask_subject_cnts[self.target.color]):
            
            if send.color_mask_subject_size[self.target.color][blue_cnt] > 10:
                self.new_target_xmax = send.color_mask_subject_XMax[self.target.color][blue_cnt]
                self.new_target_xmin = send.color_mask_subject_XMin[self.target.color][blue_cnt]
                self.new_target_ymax = send.color_mask_subject_YMax[self.target.color][blue_cnt]
                self.new_target_ymin = send.color_mask_subject_YMin[self.target.color][blue_cnt]
                
                if (self.lower_blue_ymax > self.new_target_ymax) and (self.lower_blue_xmax > self.new_target_xmax):
                    self.lower_blue_ymax = self.new_target_ymax
                    self.lower_blue_xmax = self.new_target_xmax
                    self.blue_x_middle = (self.new_target_xmax + self.new_target_xmin) / 2
                    self.blue_y_middle = (self.new_target_ymax + self.new_target_ymin) / 2

                    # rospy.logwarn(f"lower blue ymax: {self.lower_blue_ymax}\033[K")
        #self.lower_blue_ymax, self.blue_x_middle, self.new_target_xmax, self.new_target_xmin = self.ladder.get_object_ymax

    def keep_head(self):
        
        # if self.blue_y_middle - TARGET_LINE > 5:
            # self.now_head_Vertical = self.now_head_Vertical+5
        # elif self.blue_y_middle - TARGET_LINE < -5:            
            # self.now_head_Vertical = self.now_head_Vertical-5

        # if abs(self.blue_y_middle - TARGET_LINE) > 5:
        #     self.now_head_Vertical = int(self.now_head_Vertical+(self.blue_y_middle - TARGET_LINE)*-3)
        # send.sendHeadMotor(2, self.now_head_Vertical, 100)#垂直

        y_difference =  self.blue_y_middle - TARGET_LINE               #目標與中心y差距            
        y_degree = y_difference * (38 / 240)         #目標與中心y角度        

        send.sendHeadMotor(2,self.now_head_Vertical - round(y_degree * 4096 / 360 *0.5), 35)   #352
        self.now_head_Vertical = self.now_head_Vertical - round(y_degree * 4096 / 360 *0.5)
        if abs(self.now_head_Vertical - 2048) > 880 :
                if (self.now_head_Vertical - 2048 ) > 0 :
                    self.now_head_Vertical = 2048 + 880
                elif (self.now_head_Vertical - 2048) < 0 :
                    self.now_head_Vertical = 2048 - 880



    
    def walkinggait(self,motion):
    #步態函數
        if motion == 'ready_to_cw':
            rospy.loginfo("對正梯子\033[K")
            send.sendBodyAuto(0,0,0,0,1,0)           #停止步態
            send.sendSensorReset(1,1,1)                   #IMU reset 避免機器人步態修正錯誤
            rospy.sleep(3)                           #穩定停止後的搖晃
            send.sendBodySector(29)                  #這是基本站姿的磁區
            while not send.execute:
                rospy.logdebug("站立姿勢\033[K")
            send.execute = False
            rospy.sleep(3) 

            send.sendHeadMotor(2, self.head_Vertical, 100)#垂直
            # self.squat_times = (self.now_head_Vertical - self.head_Vertical)//10
            # while self.squat_times > 0:
            #     send.sendBodySector(10) #9極限
            #     rospy.sleep(1)
            #     self.squat_times -= 1
            # #-爬梯磁區-#
            send.sendBodySector(871)
            rospy.sleep(11.5)
            send.sendBodySector(872)
            rospy.sleep(12)
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
            # if send.imu_value_Yaw > 1:
            #     self.now_theta = -THETA_NORMAL
            # elif send.imu_value_Yaw < -1:
            #     self.now_theta = THETA_NORMAL
            # else:
            #     self.now_theta = 0

            self.now_theta = self.imu_angle()

            #速度調整
            # self.moveTotal += self.now_forward
            send.sendContinuousValue(self.now_forward, self.now_translation, 0, self.now_theta, 0)

    def edge_judge(self,strategy):
    #邊緣判斷,回傳機器人走路速度與走路模式
        if (self.moveTotal >= UP_LADDER_DISTANCE) and (self.blue_x_middle >= BLUE_MID-BLUE_ERROR) and (self.blue_x_middle <= BLUE_MID+BLUE_ERROR) and abs(send.imu_value_Yaw) < 1.2:
            self.state = "爬梯"
            return "ready_to_cw"
        
        else:
            if (self.moveTotal > 125000):
                self.theta       = send.imu_value_Yaw/4
                self.forward     = BACK_MIN + FORWARD_CORRECTION
                self.state       = "!!!小心採到梯子,後退!!!"

            elif (self.moveTotal >= UP_LADDER_DISTANCE) and (self.blue_x_middle < BLUE_MID):
                self.forward     = BACK_MIN+ FORWARD_CORRECTION
                self.theta       =  0
                self.translation = LEFT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION + 300
                self.state       = "左平移"

            elif (self.moveTotal >= UP_LADDER_DISTANCE) and (self.blue_x_middle > BLUE_MID):
                self.forward     = BACK_MIN+ FORWARD_CORRECTION
                self.theta       =  0
                self.translation = RIGHT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                self.state       = "右平移"
            
            else:
                # if self.moveTotal < SECOND_FORWORD_CHANGE_LINE:
                    # if self.blue_x_middle < BLUE_MID-BLUE_ERROR: #左移
                    #     self.translation = LEFT_THETA * TRANSLATION_BIG+ TRANSLATION_CORRECTION + 400
                    #     self.state       = "大左平移  "
                    
                    # elif self.blue_x_middle > BLUE_MID+BLUE_ERROR: #右移
                    #     self.translation = RIGHT_THETA * TRANSLATION_BIG + TRANSLATION_CORRECTION
                    #     self.state       = "大右平移  "
                    # else:
                    #     self.translation = TRANSLATION_CORRECTION
                if self.moveTotal > FIRST_FORWORD_CHANGE_LINE:
                    if self.blue_x_middle < BLUE_MID-BLUE_ERROR: #左移
                        self.translation = LEFT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION + 500
                        self.state       = "左平移  "
                    
                    elif self.blue_x_middle > BLUE_MID+BLUE_ERROR: #右移
                        self.translation = RIGHT_THETA * TRANSLATION_NORMAL + TRANSLATION_CORRECTION
                        self.state       = "右平移  "
                    else:
                        self.translation = TRANSLATION_CORRECTION 
                else:
                        self.translation = TRANSLATION_CORRECTION                           
                

                if self.moveTotal < THIRD_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_BIG + FORWARD_CORRECTION
                    self.state      = '大前進'                   
                elif self.moveTotal < SECOND_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_NORMAL + FORWARD_CORRECTION
                    self.state      = '前進'             
                elif self.moveTotal < FIRST_FORWORD_CHANGE_LINE:
                    self.forward     = FORWARD_MIN + FORWARD_CORRECTION
                    self.state      = '小前進'
                else:
                    self.theta      = THETA_CORRECTION
                    self.forward    = FORWARD_BIG + FORWARD_CORRECTION
                    self.state     = 'no'
                    # self.head = False
                    
                    # self.now_head_Vertical += 50
                    # send.sendHeadMotor(2,self.now_head_Vertical, 35)   #352
            return 'walking'

    def imu_angle(self):      #一般 imu修正角度
        imu_ranges = [  (180,  -3),
                        (90,  -3), 
                        (60,  -3), 
                        (45,  -3), 
                        (20,  -3), 
                        (10,  -2), 
                        (5,   -2), 
                        (2,   -1), 
                        (0,    0),
                        (-2,    1),
                        (-5,    2),
                        (-10,   2),
                        (-20,   3),
                        (-45,   3),
                        (-60,   3),
                        (-90,   3),
                        (-180,   3)]
        for imu_range in imu_ranges:           
            if send.imu_value_Yaw >= imu_range[0]:
                return imu_range[1]
        return 0
    def draw_function(self):
    #畫面顯示繪畫資訊    
        send.drawImageFunction(1, 0, BLUE_MID, BLUE_MID, 0, 240, 255, 0, 0)   #基準線X
        send.drawImageFunction(2, 0, int(self.blue_x_middle), int(self.blue_x_middle), 0,240, 255, 255, 0) #物件中心X
        send.drawImageFunction(3, 0, 0, 320, TARGET_LINE,TARGET_LINE, 255, 0, 0) #基準線Y
        send.drawImageFunction(4, 0, 0, 320,int(self.blue_y_middle),int(self.blue_y_middle), 255, 255, 0) #物件中心Y

    def route_plan(self):
    #路徑規劃
        if ROUTE_PLAN_FLAG:
            for t in range(len(ROUTE_PLAN)//4):
                start = rospy.get_time()
                end   = 99999
                rospy.sleep(1)       #啟動步態後穩定時間
                sys.stdout.write("\033[H\033[J")

                while (end-start) < ROUTE_PLAN[3+4*t]:
                    end = rospy.get_time()
                    rospy.loginfo("\033[K\033[H")
                    rospy.loginfo(f"RemainingTime:{end-start}\033[K")                    
                    rospy.loginfo(f"{ROUTE_PLAN[0+4*t]},{ROUTE_PLAN[1+4*t]},{ROUTE_PLAN[2+4*t]},{ROUTE_PLAN[3+4*t]}\033[K")
                    
                    self.forward     = ROUTE_PLAN[0+4*t]+FORWARD_CORRECTION
                    self.translation = ROUTE_PLAN[1+4*t]+TRANSLATION_CORRECTION
                    self.theta       = ROUTE_PLAN[2+4*t]+THETA_CORRECTION
                    
                    send.sendContinuousValue(self.forward,self.translation,0,self.theta,0)

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
                            'Target': self.get_object,
                            'Ball' : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_object(self):
        max_object_size = max(send.color_mask_subject_size[self.color])
        max_object_idx  = send.color_mask_subject_size[self.color].index(max_object_size)
        # return max_object_idx if max_object_size > 10000 else None
        return max_object_idx if max_object_size > 100 else None

    def get_object_ymax(self):
        lower_ymax      = 0
        new_target_xmax = 0
        new_target_xmin = 0
        new_target_ymax = 0
        x_middle = 160

        #-------距離判斷-------#
        for blue_cnt in range (send.color_mask_subject_cnts[2]):
            if send.color_mask_subject_size[2][blue_cnt]:
                new_target_xmax = send.color_mask_subject_XMax[2][blue_cnt]
                new_target_xmin = send.color_mask_subject_XMin[2][blue_cnt]
                new_target_ymax = send.color_mask_subject_YMin[2][blue_cnt]
                
                if lower_ymax < new_target_ymax:
                    lower_ymax = new_target_ymax
                    x_middle = (new_target_xmax + new_target_xmin) / 2
        
        return lower_ymax, x_middle, new_target_xmax, new_target_xmin
        
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

'''                                                                                                                                                                                                      
                                                                                                                             .::-========--:.                                                         
                                                                                                                        :-=+++==---:::::---=++=-.                                                     
                                                                                                                    .-++=-:...              ..:=++:                                                   
                                                                                                                 .=++-:.  ....................  .:=+-                                                 
                                                                                                               -++=:. .............................:++:                                               
                                                                                                            :=+=:. ..................................:*+                                              
                                                                                                          :++-. .......................................-*:                                            
                                                                                                        -*+:........................................... :*-                                           
                                                                                                      -*+:................................................#-                                          
                                                                                                    :*+. ..................................................#-                                         
                                                                                                  :**:.................................................... :#:                                        
                                                                                                .+*-....................................................... =#.                                       
                                                                                               =#=...........................................................*+                                       
                                                                                             :*+..............................................................#-                                      
                                                                                            =*-.............................................................. =#                                      
                                                                                          :*+..................................................................#=                                     
                                                                                         =*-.................................................................. -#.                                    
                                                                                       .*+......................................................................#=                                    
                                                                                      -#=...................................................................... =#.                                   
                                                                                     +*: .......................................................................:%-                                   
                                                                                   .*+.......................................................................... *+                                   
                                                                                  :#=........................................................................... =#.                                  
                                                                                 -#- ............................................................................:#-                                  
                                                                                =#: ..............................................................................#=                                  
                                                                               =#: .............................................................................. +*                                  
                                                                              +#:................................................................................ -#.                                 
                                                                             +*...................................................................................:%:                                 
                                                                            +#.....................................................................................#-                                 
                                                                           +#......................................................................................*=                                 
                                                                          =#...................................................................................... ++                                 
                                                                         -#:...................................................................................... +*                                 
                                                                        :%- ...................................................................................... =*                                 
                                                                       .#- ....................................................................................... =#.                                
                                                                       *+ ........................................................................................ =#.                                
                                                                      =#.......................................................................................... =#.                                
                                                                     :%: .....................................       ............................................. =#.                                
                                                                     #%................................... ..-==+++==-............................................ =*                                 
                                                                    =%#:.............................. ..-+**++======+**-......................................... =*                                 
                                                                   .%=*= ........................... .-+*+=------------=#+........................................ +*                                 
                                                                   +#:+#......................... .-+*+=-----------------*+ ...................................... +*                                 
                                                                  .%=:-%- ..................... .=**=---------------------*+ ......................................*+                                 
                                                                  +#:-:+#.................... .=*+-------------------------#- .....................................*=                                 
                                                                 .%=:---#+ ................ .=*+---------------------------=#. ....................................#-                                 
                                                                 =%:-----#+................-*+-----------------------------:+#: .......-=+++++++=: ................#-                                 
                                                                 *+:------+*-...........:=*+---------------------------------=*+-::-==++=-------+*=...............:#:                                 
                                                                :%-:--------+++=----=++++=-------------------------------------=++++=-------------*+ ............ -#.                                 
                                                                -#-------------=++++==-------------------------------------------------------------*+ ........... =#.                                 
                                      .:-=======-:.             +*:-------------------------------+-------------------------------------------------*+ .......... +*                                  
                                   :=+=-::......:=++=:          *+:------------------------------*=--------------------------------------------------*+.......... *+                                  
                                 -+=:.             .:=++:      .#=:----------------------------:+%----------------------------------------------------**..........#=                                  
                               :*=.                    :=*=.   :%-:-----------------------------%+:---------------------------------------------------:+*: .... .#@-                                  
                              +*:                        .-+=. -%------------------------------+@----------------==-------------------------------------=*=:..:-**#.                                  
                            .*=.                            -+++#:----------------------------:##:---------------*----------------------------------------=++++=-+#                                   
                           -*-            ..................  -%*:-----------------------------%+:--------------+#----------------------------------------------:**                                   
                          -#:          ...................... .#*:-----------------------------@+:--------------**------------------------------*=--------------:*+                                   
                         -*:        ................. ....... .%*:-----------------------------+----------------*+-----------------------------=#----------------#-                                   
                        :#:        ................ .   ..... .%*:-----------------------------------------------------------------------------*+:-------------:-%:                                   
                       .#-       ................          .. .%*:----------------------------------------------------------------------------=#---------------:=#.                                   
                       *= .     ........  ........             ##:----------==================================--------------------------------+=---------------:**                                    
                      =*...    ...................             +#----=====+++++*****####################******++++======----------------------------------------#=                                    
                     .#: .     ...........                     :%=-=++++**#######***********************#@++*****######**+++=====-------------------------------%:                                    
                     ++ .     .........                         *#-+++****+++++++++++++++++++++++++++++++%=:::::----=%#*#######**++====-----------------------:+#.                                    
                    .#: .    .....                              .#*=+++++++++++++++++++++++++++++++++++++%=:-------:+%++++++++**######*++===------------------:*+                                     
                    =*.     .                                    .+#*++++++++++++++++++++++++++++++++++++%+:--------#%++++++++++++++**#####*++====-------------#-                                     
                    *= .   ..                                      .-=****+++++++++++++++++++++++++++++++%+::::::::-%*++++++++++++++++++++*####*++====--------=#.                                     
                   .#: .  .                                            .:-=*****++=++++++++++++++++++++++%+.....::.=@+++++++++++++++++++++++++**##**++===----:**                                      
                   :#. .                                                    ..:-=#%*****+++++++++++++++++%=        +%+++++++++++++++++++++++++++++*##*++++=--+#.                                      
                   =*  ..                                                        .*+.::-=++********++++++%-        ##+++++++++++++++++++++++++++++++++++++++**.                                       
                   =*                                              .              .*=        ..:-==++***#@:       .%#+++++++++++++++++++++++++++++++****++=-.                                         
                   =*                          .-----:.        .-+++++=-.          .#-                  -%.       -%*********************+**+++====--:..                                              
                   =*                       .=+++===+++=-.  .:+*=------=*+.         :#:                 -#        =*         ..........                    ....:::..                                  
                   =*                      =*=-::::::::-++++++-::::::::::=*=.        +#.                ++        +=                               .:-========--===++++=:                             
                   -#.                   .*+::::--::---:::--:::-::-::----:-=+=:.    -+**                #-        *-                          .:-+++=--:..           ..-=*=.                          
                   :#.                  .*=:----:-::-:---::::-::-::--:-::-:::=+++==++:-#-              .#:       .#:                       :=++=-:.                      .-*=                         
                   .#:                  ++:-::--::::---:--:----::::-:--:--:--:::----:::+#              -#        :#.                   .-++=-.                             .**                        
                    *=                 .#-:::-::-:----:::--:--:-=:-----:--------::::-:::#=             *+        -*.                .=++-:                                  .**                       
                    =*                 +*:-------:--:--::-::--:-#=:--::-:-::---:-::-:-:.=*            .%:        -*              :=++-.       ....                           .#=                      
                    :#.               :%-:--::--::::::::-:::-:-:=%=:-::---::::--::----::-#:           -#.        -*           .=*+-.   .   ....                       .       =#                      
                     #=               *+:---:-:---::::-:::-----::+*:-::::::::::::::::::::*+           *+         =*         :**-.  ....... ...         ......           ..... :#.                     
                     =*              =#::-:::-::=-:-:--:---::::::::::-----------------:::+#          .#.         -*       -*#=  .......  ...             .....           .    .#:                     
                     .#:            :#-:-:::::::+*::-::--:::::----=======+++++++++++==--:=#.         +*          -*     =*++:   ........       ....           .                *-                     
                      =+           .#=:-::-::--::#+::::::----===+++**############****++=--#:         #-          :*.  :*+-*: ..  .......       ....                            #-                     
                      .#=.        :#=:::::-::::-:-*-:---===++*####@#**++++++++++++++++++=-#-        =#           .*. +*-:*- ....   ....  ...           ...  .              .   #-                     
                       =%+=-::::-=*-:--:--::---:::---==++*##**+=--%*=+++++++++++++++++++==%-        #-            *+*=::++   ..     ...                    ...    .......   . .#-                     
                        *+:-==++=-::-:--:----:::--==+*##%%+--::--:+@++++++++++++++++++++=*#.       -#             *%-::-#. ..                                      ...        .%:                     
                        :#-.:::::::::::-:-:::--==++###*+=*%=:-----:#%++++++++++++++++++=*#:        #=            +*::::+* .                                                   :%.                     
                         =*:::::-:::---:-::--==+*#%*++++++#%-:-:::::%#=++++++++++++++++*#:        -#            +*-::::++ .                                              .    =#                      
                          +*::::---:::-::--=++*%#*+++++++++##:::... :%*++++++++++++++*#=          %-           =#-:::::-#.                                                    ++                      
                           *+::::-::-:::-=++*%#+++++++++++++%*..     :%*+++++++++++*+-           +#           :#-:::::::+*.                                                  .#-                      
                            *+:::----:--=++#%*+++++++++++++++%+       :%*+++++++**=.            .%:          .#=:-:::--::=*=:.....                                           -#.                      
                            .*+:::-:::-=++%%+++++++++++++++++*%-       .##++**+=:               *+           =#:::::-:::-::-======+++==:                                     +*                       
                             .*+:::-:-=++*%=++++++++++++++++++*%:        +%*-.                 =#            +*::::-::::---::::::::::-=*=                                   .#-                       
                               ++::::-=++*#+++++++++++++++++++=##.        -*-                 .%:            +*::--::::--::-::----:--:::*=                                  -#                        
                                +*-::-=+++++++++++++++++++++****%*         .=+-               #=             -%-:--::-----::--:::-:--:-:-#.                                 *+                        
                                 :+*+=-====+++++++++*******++-:..#-          .++-            ++               *#::---:::-:-+*-:--:-:-:-::#-                                :#.                        
                                   .-=++++****+++++==-::.        -#            .=+-.        =*                .**::--::-:=#=:::------:-::+=                                *+                         
                                                                  #-             .-+=:   .-+%+====:.           .**::-:--:--:::::-:-::::-:-*:                    .--==+=-  -#.                         
                                                                  ++                :**===-:.....:=+++:          +#-::-:-::--::--:----:--:=*:                .-++=-----*+.*=                          
                                                                  =*               :++-.             :+*=         -#+::----:-----:--:--::::=*-.          .:-++=-:::::-:-*+*                           
                                                                  -* .:--====-:. .++:                  .=*-        .+*=:::::::--:--::-:---::-+*+=-----==+++=-:::---:-::-#%:                           
                                                                  =%++=-:::::-=+*#-                      .*+.... ..:-*%*+++=--:::-::::=#=::-:::-===+===--::::--::----::-%=                            
                                                                .++-.          :#:                         *+..:=++-:.....:-=++=::--:-#=::--:--::::::::::--:::::-:-::::*+                             
                                                               =+:             *+                          .#=+*-             .=*=:-:--:-----:::::--------::::-::--:::*+                              
                                                              =+               #-                           *@=                 :%+::::-:-:--::----::-::---:--:---::=#+                               
                                                             :#:               %-                           ++                   :%#+=-::::::::::::---::-:::::::::=**:                                
                                                             .#.               #+                          .#:                    =%=++**+==------:::::::-----==+*+:                                  
                                                              *-               :*.                         =*                     :%:.=+=:--=++++++++++++++++=--.                                     
                                                             :+%:               .:                        :#.                     =#=*=.        .:-----:.                                             
                                                           :+=.-#-               .:                      :+.                     .%%*+---==:.-++==----==+=:                                           
                                                          -+.   .=:.                                                            .*+.      .+%=..        .:++.                                         
                                                         .#-       ...                                                         :::          =*...........  :*:                                        
                                                         :#:                                                                  .              -............. -#.                                       
                                                          *-                                                                                  .............. *=                                       
                                                          -#.                                                                                 .............. ++                                       
                                                           -*:                                                                                .............. #=                                       
                                                            :*=.                                                                             .............. =#                                        
                                                              :++                                                                           ............. .+*.                                        
                                                              .*=                                                                              .....   ..=*=                                          
                                                             -*:                                                                            .::....::-=++-                                            
                                                            =*.                                                                              +%+====-:.                                               
                                                           =*.                                                                                =+                                                      
                                                          =*.                                                                                 .#-                                                     
                                                         -#.                  .:.                                                              =*                                                     
                                                        .#-                 +#%%%#:                          ...                               .#:                                                    
                                                        ++                :#%####%%-                       -*###*-                              *=                                                    
                                                       :#:               :%%######%#.                    :#%####%@+                             +*                                                    
                                                       ++          ....  %%########@=                   -%%######%@:                            =#                                                    
                                                      .#-        ...... =@#########%*                  :%%########%+                            =%                                                    
                                                      :#:       ........#%#########%#.                 #%#########%#. ...                       =#                                                    
                                                      -#.       .......:@%#########%%.                :@%#########%%:......                     =#                                                    
                                                      -#.       .......:@%#########%%.                =@##########%%:.......                    +*                                                    
                                                      -%.        .......%%#########%#.                =@##########%%:.......                    #=                                                    
                                                      :#:         ..... +@#########@*                 =@##########%#........                   -#.                                                    
                                                      .#=               .%%#######%@-                 :@%#########@+ ......                    *+                                                     
                                                       **                -@#######@*                   *@########%#. ....                     -%.                                                     
                                                       :#:                =%%####%*.                   .%%######%%:                          .#=                                                      
                                                        +*                 :+###*=                      :#%####%#:                          .#+                                                       
                                                         *=                   ..                          =*##*=.                          .#+                                                        
                                                         .*=                                                                              :#=                                                         
                                                          .*+                                                                            =*-                                                          
                                                            =*:                                                                        :*+.                                                           
                                                             :*+:...                                                                 :+*:                                                             
                                                               -*+:......                                                         .:++:                                                               
                                                                 -*+-.........                                              ....:=*=:                                                                 
                                                                   :+*=:...:::..........                         ...........:-+*+:                                                                    
                                                                     .-+*+-:...:::::.............................:::::...:=++=:                                                                       
                                                                         :=+++-::...:::::::::::::::::::::::::::::..::-=+++-.                                                                          
                                                                            .:=++++=-:::....::::::::::.....::::--==++=-.                                                                              
                                                                                 .:-=++++=---:.::.::::-==++++++=-:.                                                                                   
                                                                                        .:--+#-....::**-:..                                                                                           
                                                                                             ++.:::.-#.                                                                                               
                                                                                             ==.:::.#=                                                                                                
                                                                                             *-.::.-#.                                                                                                
                                                                                            .#:....*=                                                                                                 
                                                                                            -*    :%.                                                                                                 
                                                                                            +=    +*                                                                                                  
                                                                                           .#:    #-                                                                                                  
                                                                                           -*    :#.                                                                                                  
                                                                                           *-    =*                                                                                                   
                                                                                          :#     #=                                                                                                   
                                                                                          +=    .%:                                                                                                   
                                                                                         :#.    :%                                                                                                    
                                                                                         +* ..  :%                                                                                                    
                                                                                        :%:.....:%.                                                                                                   
                                                                                        *+ ......#-                                                                                                   
                                                                                       +*........=#                .-=+=                                                                              
                                                                                      =#:........-**:           .-+++=-#+                                                                             
                                                                                     =#:........==-=**+=-:::-=++++====-**                                                                             
                                                                                    =#:....:... **-====+++++++=======-=%-                                                                             
                                                                                   +*:....*#... =@+==================+#-                                                                              
                                                                                 .**.....*+*= ..:%**+==============+*+.                                                                               
                                                                                -*=....-*= :#... -*:-+***++==+++**+=.                                                                                 
                                                      .:--::.                .=+=.....=*:   +*... -*-  .:-====--:.                                                                                    
                                                    -+=-::--==+====---:---===+-.....-*=      **.....=+-                                                                                               
                                                    ++ ...    ...:::-----::.......-++.        +#- ....=+-.                                                                                            
                                                    .++:.......................:=+=.           :++: ....:=+=:.            .:--.                                                                       
                                                      :++-:................:-=+=-                :++-......:-=++==----=====-:-*=                                                                      
                                                        .-=+==--:--:--=====-:.                     :+*-.........:::---:..     *=                                                                      
                                                            .::--==---:..                            .=++-:.................-*-                                                                       
                                                                                                        .-=++=--::::::::-===-.                                                                        
                                                                                                            .::-=====---:..                                                                           
'''                                                                                                                                                                                                      
                                                                                                                                                                                                      
                                                                                                                                                                                                      
                                                                                                                                                                                                      