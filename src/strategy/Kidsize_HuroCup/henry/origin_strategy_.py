#!/usr/bin/env python
#coding=utf-8
from dis import dis
from types import LambdaType
import rospy
import numpy as np
from Python_API import Sendmessage
from SR_API import Send_distance
from SR_API_2 import Ladder_send_distance

from SR_API_ladder  import Climb_ladder

import time

imgdata = [[None for high in range(240)]for width in range (320)]

if __name__ == '__main__':

    try:        
        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
        distance = Send_distance()#建立名稱,順便歸零
        Ladder = Ladder_send_distance()

        Climb = Climb_ladder()

        send.is_start = False
        while not rospy.is_shutdown():
            if send.is_start == True:
#=====================鏡頭上畫線(name,line or square,xmin,xmax,ymin,ymax,r,g,b)=====================
                send.drawImageFunction(1,0,0,320,230,230,255,0,0)#膝蓋的橫線
                send.drawImageFunction(2,0,150,150,0,240,255,0,0)#lr的線                      #lr代表〝左腳〞的〝右邊〞那條線
                send.drawImageFunction(3,0,115,115,0,240,255,0,0)#ll的線                      #以此類推
                send.drawImageFunction(4,0,165,165,0,240,255,0,0)#rl的線                      #以此類推
                send.drawImageFunction(5,0,200,200,0,240,255,0,0)#rr的線                      #以此類推

                send.drawImageFunction(6,0,15,15,0,240,255,0,0)#lll的線                          #左邊界
                send.drawImageFunction(7,0,300,300,0,240,255,0,0)#rrr的線                   #右邊界

                send.drawImageFunction(9,0,0,320,Climb.eyeline_y,Climb.eyeline_y,255,255,255)   #眼睛的橫線
                send.drawImageFunction(10,0,Climb.eyeline_x,Climb.eyeline_x,0,240,255,255,255)   #垂直線
                
                send.sendHeadMotor(1,2048,100)                                                                          #設定頭部水平位置
                send.sendHeadMotor(2,1405,100)                                                                          #設定頭部垂直位置
#==============================================================================================

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            #判斷上/下板（一般調整）
            #     if distance.walk_flag == False and distance.up_board_flag == False:
            #         send.sendBodyAuto(400,0,0,0,1,0)                                                                    #走路速度（進退,平移,用不到,左右轉角度,0/1（走一步/持續走））
            #         time.sleep(2)
            #         distance.walk_flag = True
            #         distance.up_board_flag = True
            #         distance.Times += 1

            #     elif distance.walk_flag == True and distance.up_board_flag == True:
            #         if distance.Times ==1:
            #             distance.speed_revision()
            #             distance.find_Rboard()                                                                                            #呼叫SR_API的find_board
            #             distance.revision()

            #         elif distance.Times == 2:
            #             distance.speed_revision()
            #             distance.find_Yboard()
            #             distance.revision()

            #         elif distance.Times == 3:
            #             distance.speed_revision()
            #             distance.find_Bboard()
            #             distance.revision()
                    
            #         elif distance.Times ==4 :
            #             distance.speed_revision()
            #             distance.find_Yboard()
            #             distance.revision_down()

            #         elif distance.Times == 5:
            #             distance.speed_revision()
            #             distance.find_Rboard()
            #             distance.revision_down()

            #         elif distance.Times == 6:
            #             distance.speed_revision()
            #             distance.find_Gboard()
            #             distance.revision_down()

            #         elif distance.Times == 7:
            #             send.sendBodyAuto(200,0,0,0,1,0)  
            # #板前原地踏步微調
            #     elif distance.walk_flag == False and distance.up_board_flag == True:
            #         if distance.Times ==1:
            #             distance.speed_revision()
            #             distance.find_Rboard()
            #             distance.revision_stop()

            #         elif distance.Times == 2:
            #             distance.speed_revision()
            #             distance.find_Yboard()
            #             distance.revision_stop()

            #         elif distance.Times == 3:
            #             distance.speed_revision()
            #             distance.find_Bboard()
            #             distance.revision_stop()
                    
            #         elif distance.Times ==4 :
            #             distance.speed_revision()
            #             distance.find_Yboard()
            #             distance.revision_stop()

            #         elif distance.Times == 5:
            #             distance.speed_revision()
            #             distance.find_Rboard()
            #             distance.revision_stop()

            #         elif distance.Times == 6:
            #             distance.speed_revision()
            #             distance.find_Gboard()
            #             distance.revision_stop()

            #         elif distance.Times == 7:
            #             send.sendBodyAuto(200,0,0,0,1,0)  

            # if send.Web == False:
            #     if distance.walk_flag == True:
            #         send.sendBodyAuto(200,0,0,0,1,0)
            #         distance.walk_flag = False
            #         distance.up_board_flag = False
            #         distance.Times = 0
            #     # else:
            #     #     send.sendBodySector(29)
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                if Ladder.walk_flag == False and Ladder.up_ladder_flag == False:
                    send.sendBodyAuto(200,0,0,0,1,0)                                                                    #走路速度（進退,平移,用不到,左右轉角度,0/1（走一步/持續走））
                    time.sleep(1)
                    Ladder.walk_flag = True
                    Ladder.up_ladder_flag = True
                    Ladder.Times += 1

                elif Ladder.walk_flag == True and Ladder.up_ladder_flag == True:
                    if Ladder.Times ==1:

                        if Climb.read_ladder_p < Climb.ladder_n:
                            Climb.find_up_ladder()
                            Climb.rise_head()
                            Climb.print_state()
                        elif Climb.read_ladder_p == Climb.ladder_n:
                            Climb.find_ladder_hight()
                        else:
                            send.sendHeadMotor(2,1300,100)#頭回到原位

                        Ladder.Find_ladder_henry()  
                        
                        Climb.ladder_distance()
                        Climb.find_up_ladder()
                        Climb.rise_head()
                        Climb.find_ladder_hight()
                        Climb.robot_high_cal
                        Climb.print_state

                        Ladder.Ladder_revision()
                        Ladder.Direction_Deviation()

                elif Ladder.walk_flag == False and Ladder.up_ladder_flag == True:
                        Ladder.Find_ladder_henry

            elif send.is_start == False:
                if Ladder.walk_flag == True:
                    send.sendHeadMotor(2,1300,100)#頭回到原位

                    send.sendBodyAuto(200,0,0,0,1,0)
                    Ladder.walk_flag = False
                    Ladder.up_ladder_flag = False
                    Ladder.Times = 0
                # else:
                #     send.sendBodySector(29)
                #     time.sleep(1)

    except rospy.ROSInterruptException:
        pass
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
