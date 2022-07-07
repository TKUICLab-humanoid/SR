#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
from find_ladder_API import Find_ladder
from SR_API import Send_distance



imgdata = [[None for high in range(240)]for width in range (320)]


if __name__ == '__main__':
    try:
        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
        ladder = Find_ladder()
        distance = Send_distance()

        send.drawImageFunction(1,0,0,320,ladder.eyeline_y,ladder.eyeline_y,255,255,255)   #眼睛的橫線
        send.drawImageFunction(2,0,ladder.eyeline_x,ladder.eyeline_x,0,240,255,255,255)   #垂直線
        # send.drawImageFunction(3,0,ladder.eye_r,ladder.eye_r,0,240,255,255,255)   #右線
        send.sendHeadMotor(2,ladder.head_highest,100)#頭回到原位

        while not rospy.is_shutdown():
            if send.is_start == True:
                print('start')
            #     print('start')
            #     ladder.find_all()
                
            elif send.is_start == False:
                 print('turn off')
            #     send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
            #     send.sendHeadMotor(2,distance.head_Vertical,100)#垂直


            


    except rospy.ROSInterruptException:
        pass