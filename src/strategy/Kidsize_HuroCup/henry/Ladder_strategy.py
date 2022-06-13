#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
from Ladder_API import Send_Climb
import time

#imgdata = [[None for high in range(240)]for width in range (320)]
# lrdistance = 0
# lldistance = 0
# rldistance = 0
# rrdistance = 0

if __name__ == '__main__':
    try:
        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
        climb = Send_Climb()#建立名稱,順便歸零

        while not rospy.is_shutdown():
            #判斷Humanoid Interface的按鈕
            #if send.Web == True:
            if send.is_start ==True:
                send.drawImageFunction(1,0,0,320,215,215,255,0,0)#膝蓋的橫線
                send.drawImageFunction(2,0,98,98,0,240,255,0,0)#ll的線
                send.drawImageFunction(3,0,150,150,0,240,255,0,0)#lr的線
                send.drawImageFunction(4,0,188,188,0,240,255,0,0)#rl的線
                send.drawImageFunction(5,0,240,240,0,240,255,0,0)#rr的線
                send.sendHeadMotor(1,2048,100)#水平
                send.sendHeadMotor(2,1425,100)#垂直

                if climb.stop_flag == 1 and climb.up_ladder_flag == 0:
                    send.sendBodyAuto(500,0,0,0,1,0)
                    climb.stop_flag = 0
                elif climb.stop_flag == 1 and climb.up_ladder_flag == 1:
                    send.sendBodyAuto(0,0,0,0,1,0)
                    climb.stop_flag = 0
                    climb.up_ladder_flag = 0


                elif climb.stop_flag == 0 :
                    climb.find_ladder()
                    climb.up_ladder()
                

            #elif send.Web == False:
            elif send.is_start ==False:
                if climb.stop_flag == 0:
                    print("turn off")
                    climb.theta = 0
                    climb.speed = 0
                    climb.yspeed=0
                    send.sendBodyAuto(0,0,0,0,1,0)
                    send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
                    climb = Send_Climb()#建立名稱,順便歸零
                    climb.stop_flag = 1
                    time.sleep(0.5)
                    send.sendBodySector(29)
                send.sendHeadMotor(1,2048,100)#水平
                send.sendHeadMotor(2,1375,100)#垂直
                time.sleep(0.5)
                    

    except rospy.ROSInterruptException:
        pass