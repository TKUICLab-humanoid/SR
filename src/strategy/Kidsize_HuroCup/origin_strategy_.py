#!/usr/bin/env python
#coding=utf-8
from dis import dis
from types import LambdaType
# from winreg import DisableReflectionKey
import rospy
import numpy as np
from Python_API import Sendmessage
from SR_API import Send_distance
# from SR_API_2 import Ladder_send_distance
import time

imgdata = [[None for high in range(240)]for width in range (320)]

if __name__ == '__main__':

    try:        
        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
        distance = Send_distance()#建立名稱,順便歸零
        # Ladder=Ladder_send_distance()

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

                # send.drawImageFunction(8,0,0,320,150,150,0,0,0)#Cup橫線
                # send.drawImageFunction(9,0,0,320,155,155,0,0,0)#Cdown橫線

                send.sendHeadMotor(1,2048,100)                                                                          #設定頭部水平位置
                send.sendHeadMotor(2,1405,100)                                                                          #設定頭部垂直位置
#==============================================================================================

                if distance.Walk == False and distance.Progress == False:
                    send.sendBodyAuto(400,0,0,0,1,0)                                                                    #走路速度（進退,平移,用不到,左右轉角度,0/1（走一步/持續走））
                    time.sleep(0.5)
                    distance.Walk = True
                    distance.Progress = True
                    distance.Times += 1

                elif distance.Walk == True and distance.Progress == True:
                    if distance.Times ==1:
                        distance.find_board()                                                                                            #呼叫SR_API的find_board
                        distance.revision()

                    elif distance.Times == 2:
                        distance.Sec_board()
                        distance.revision()

                    elif distance.Times == 3:
                        distance.Thi_board()
                        distance.revision()
                    
                    elif distance.Times ==4 :
                        distance.Sec_board()
                        distance.revision_down()

                    elif distance.Times == 5:
                        distance.find_board()
                        distance.revision_down()

                    elif distance.Times == 6:
                        distance.find_Gboard()
                        distance.revision_down()

                    elif distance.Times == 7:
                        send.sendBodyAuto(200,0,0,0,1,0)  

                elif distance.Walk == False and distance.Progress == True:
                    if distance.Times ==1:
                        distance.find_board()                                                                                            #呼叫SR_API的find_board
                        distance.revision_stop()

                    elif distance.Times == 2:
                        distance.Sec_board()
                        distance.revision_stop()

                    elif distance.Times == 3:
                        distance.Thi_board()
                        distance.revision_stop()
                    
                    elif distance.Times ==4 :
                        distance.Sec_board()
                        distance.revision_stop()

                    elif distance.Times == 5:
                        distance.find_board()
                        distance.revision_stop()

                    elif distance.Times == 6:
                        distance.find_Gboard()
                        distance.revision_stop()

                    elif distance.Times == 7:
                        send.sendBodyAuto(200,0,0,0,1,0)  

            if send.is_start == False:
                if distance.Walk == True:
                    send.sendBodyAuto(200,0,0,0,1,0)
                    distance.Walk = False
                    distance.Progress = False
                    distance.Times = 0
                else:
                    send.sendBodySector(29)
                    time.sleep(5)
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            #     if Ladder.Walk == False and Ladder.Sum == False:
            #         send.sendBodyAuto(200,0,0,0,1,0)                                                                    #走路速度（進退,平移,用不到,左右轉角度,0/1（走一步/持續走））
            #         Ladder.Walk = True
            #         Ladder.Sum = True
            #         Ladder.Times += 1

            #     else:
            #         if Ladder.Times ==1:
            #             Ladder.Find_ladder()                                                                                            #呼叫SR_API的find_board
            #             Ladder.Ladder_revision()
            #             Ladder.Find_ladder_distance()

            # if send.Web == False:
            #     if Ladder.Walk == True:
            #         send.sendBodyAuto(200,0,0,0,1,0)
            #         Ladder.Walk = False
            #         Ladder.Sum = False
            #         Ladder.Times -= 1
            #     else:
            #         send.sendBodySector(29)
            #         time.sleep(1)

    except rospy.ROSInterruptException:
        pass
