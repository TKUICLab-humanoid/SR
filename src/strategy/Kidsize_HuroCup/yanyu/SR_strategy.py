#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
from SR_API_test import Send_distance
from Ladder_API import Send_Climb
import time
imgdata = [[None for high in range(240)]for width in range (320)]
if __name__ == '__main__':
    try:
        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
        distance = Send_distance()#建立名稱,順便歸零
        climb = Send_Climb()
        r = rospy.Rate(5)
        send.sendBodySector(29)
        while not rospy.is_shutdown():
            #判斷Humanoid Interface的按鈕
            # if send.Web == True:
            if send.is_start ==True: 
                if send.DIOValue == 25 or send.DIOValue == 26 or send.DIOValue == 27 or send.DIOValue == 17 or send.DIOValue == 18 or send.DIOValue == 19:
                    distance.print_state()
                    send.drawImageFunction(1,0,0,320,distance.knee,distance.knee,255,0,0)#膝蓋的橫線
                    send.drawImageFunction(2,0,distance.f_lr,distance.f_lr,0,240,255,0,0)#lr的線
                    # send.drawImageFunction(3,0,132,132,0,240,255,0,0)#lm的線
                    send.drawImageFunction(4,0,distance.f_ll,distance.f_ll,0,240,255,0,0)#ll的線
                    send.drawImageFunction(5,0,distance.f_rl,distance.f_rl,0,240,255,0,0)#rl的線
                    # send.drawImageFunction(6,0,182,182,0,240,255,0,0)#rm的線
                    send.drawImageFunction(7,0,distance.f_rr,distance.f_rr,0,240,255,0,0)#rr的線
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
                    start=time.time()
                    #機器人暫停且不是在做上板
                    if distance.stop_flag == 1 and distance.up_board_flag == 0:
                        send.sendBodyAuto(700,0,0,0,1,0)
                        distance.stop_flag = 0
                    elif distance.stop_flag == 1 and distance.up_board_flag == 1:
                        send.sendBodyAuto(0,0,0,0,1,0)
                        distance.stop_flag = 0
                        distance.up_board_flag = 0
                    
                    #還沒找到板子（找板 上板）
                    elif distance.stop_flag == 0 :
                        if distance.direction==0:
                            distance.find_up_board()
                            distance.up_board()
                        elif distance.direction==1:
                            distance.find_down_board()
                            distance.down_board()
                    #distance.print_state()
                    end=time.time()
                    print(end-start)
                # elif send.Web == False:
                elif send.DIOValue == 28:
                    #還沒找梯距
                    # if ladder.find_ladder_flag == 0:
                    #     send.drawImageFunction(1,0,0,320,ladder.eyeline_y,ladder.eyeline_y,255,0,0)#y基準線
                    #     send.drawImageFunction(2,0,ladder.eyeline_x,ladder.eyeline_x,0,240,255,0,0)#x基準線
                    #     if ladder.read_ladder_p < ladder.ladder_n:
                    #         ladder.find_ladder_theta()
                    #         ladder.down_head()
                    #         ladder.print_state()
                    #     elif ladder.read_ladder_p == ladder.ladder_n:
                    #         ladder.calculate_hight()
                    #         ladder.find_ladder_flag = 1
                    #         send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
                    #     else:
                    #         send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
                    send.drawImageFunction(1,0,0,320,distance.knee,distance.knee,255,0,0)#膝蓋的橫線
                    send.drawImageFunction(2,0,distance.f_ll,distance.f_ll,0,240,255,0,0)#ll的線
                    send.drawImageFunction(3,0,distance.f_lr,distance.f_lr,0,240,255,0,0)#lr的線
                    send.drawImageFunction(4,0,distance.f_rl,distance.f_rl,0,240,255,0,0)#rl的線
                    send.drawImageFunction(5,0,distance.f_rr,distance.f_rr,0,240,255,0,0)#rr的線
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,distance.head_Vertical,100)#垂直

                    if climb.stop_flag == 1 and climb.up_ladder_flag == 0:
                        send.sendBodyAuto(500,0,0,0,1,0)
                        climb.stop_flag = 0
                    elif climb.stop_flag == 1 and climb.up_ladder_flag == 1:
                        #send.sendBodyAuto(0,0,0,0,1,0)
                        #climb.stop_flag = 0
                        #climb.up_ladder_flag = 033 
                        send.sendBodySector(29)
                    elif climb.stop_flag == 0 :
                        climb.find_ladder()
                        climb.up_ladder()
            elif send.is_start ==False:
                # print('web',send.Web)
                send.sendSensorReset()
                if send.DIOValue == 9 or send.DIOValue == 10 or send.DIOValue == 11 :
                    print("turn off")
                    if distance.stop_flag == 0 or distance.up_board_flag == 1:
                        
                        distance.theta = 0
                        distance.speed = 0
                        distance.yspeed=0
                        if distance.stop_flag == 0:
                            send.sendBodyAuto(0,0,0,0,1,0)
                        # send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
                        distance = Send_distance()#建立名稱,順便歸零
                        distance.layer_n= 1
                        distance.stop_flag = 1
                        time.sleep(2)
                        send.sendBodySector(29)
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
                    time.sleep(1)
                elif send.DIOValue == 0 or send.DIOValue == 8 : 
                    # print("ladder turn off")
                    # ladder.head_init = ladder.head_highest
                    # ladder.head_now  = ladder.head_init
                    # #ladder.find_ladder_flag = 0
                    # ladder.read_ladder_p=0
                    # ladder.read_ladder_f=0
                    # ladder.ladder_dis=[999,999]
                    # ladder.head_theta=[0 for i in range(ladder.ladder_n)]                   #0數量要等於階數
                    # ladder.head_360=[0 for i in range(ladder.ladder_n)]
                    # ladder.ladder_hight=[0 for i in range(ladder.ladder_n)]
                    if climb.stop_flag == 0:
                        send.sendBodyAuto(0,0,0,0,1,0)
                    print("climb turn off")
                    climb.theta = 0
                    climb.speed = 0
                    climb.yspeed=0
                    # send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
                    climb = Send_Climb()#建立名稱,順便歸零
                    climb.stop_flag = 1
                    climb.up_ladder_flag = 0
                    time.sleep(2)
                    send.sendBodySector(29)
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
                    time.sleep(0.5)

            r.sleep()    

    except rospy.ROSInterruptException:
        pass

