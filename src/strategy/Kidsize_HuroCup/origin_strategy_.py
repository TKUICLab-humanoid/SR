#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
from SR_API_ladder import Climb_ladder

imgdata = [[None for high in range(240)]for width in range (320)]


if __name__ == '__main__':
    try:
        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
        ladder = Climb_ladder()
        while not rospy.is_shutdown():
            send.drawImageFunction(1,0,0,320,ladder.eyeline_y,ladder.eyeline_y,255,255,255)   #眼睛的橫線
            send.drawImageFunction(2,0,ladder.eyeline_x,ladder.eyeline_x,0,240,255,255,255)   #垂直線
            # send.drawImageFunction(3,0,ladder.eye_r,ladder.eye_r,0,240,255,255,255)   #右線
            if send.Web == True:
                if ladder.read_ladder_p < ladder.ladder_n:
                    ladder.find_up_ladder()
                    ladder.rise_head()
                    ladder.print_state()
                elif ladder.read_ladder_p == ladder.ladder_n:
                    ladder.find_ladder_hight()
                else:
                    send.sendHeadMotor(2,1300,100)#頭回到原位
                
                

            elif send.Web == False:
                send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
                ladder = Climb_ladder()
                send.sendHeadMotor(2,1300,100)#頭回到原位
                


            


    except rospy.ROSInterruptException:
        pass