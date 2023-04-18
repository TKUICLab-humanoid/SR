#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
from SR_API_wei_re import Lift_and_Carry,Wall_Climb
# from Ladder_API import Send_Climb
import time
imgdata = [[None for high in range(240)]for width in range (320)]
if __name__ == '__main__':
    try:
        //aa
        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
        LC = Lift_and_Carry()#建立名稱,順便歸零
        CW = Wall_Climb()
        r = rospy.Rate(30)
        send.sendBodySector(29)
        while not rospy.is_shutdown():
            #判斷Humanoid Interface的按鈕
            # if send.Web == True:
            if send.is_start ==True:
                #開啟小指撥1,2(同時或單獨都可)
                if send.DIOValue == 17 or send.DIOValue == 18 or  send.DIOValue == 19 or  send.DIOValue == 25 or  send.DIOValue == 26 or  send.DIOValue == 27:
                    LC.printf()
                    LC.draw_function()
                    send.sendHeadMotor(1,LC.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,LC.head_Vertical,100)#垂直
                    start=time.time()
                    #機器人暫停且不是在做上板
                    if LC.layer == 7:
                        #完成LC策略             
                        pass
                    elif LC.walkinggait_stop == True and LC.walkinggait_LC == False:
                        #起步
                        send.sendBodyAuto(700,0,0,0,1,0)
                        LC.walkinggait_stop = False
                    elif LC.walkinggait_stop == True and LC.walkinggait_LC == True:
                        #剛上板或下板,重新開啟步態
                        send.sendBodyAuto(0,0,0,0,1,0)
                        LC.walkinggait_stop = 0
                        LC.walkinggait_LC   = 0
                    elif LC.walkinggait_stop == False:
                        #還沒找到板子（找板 上板）
                        LC.find_board(LC.layer)
                        LC.walkinggait(LC.layer)
                    end=time.time()
                    print("策略計算總時間:",end-start)
                elif send.DIOValue == 20 or send.DIOValue == 28:
                    CW.printf()
                    send.sendHeadMotor(1,CW.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,CW.head_Vertical,100)#垂直
                    if CW.walkinggait_stop == True and CW.Climb_ladder == False:
                        send.sendBodyAuto(500,0,0,0,1,0)
                        CW.walkinggait_stop = False 
                    elif CW.walkinggait_stop == True and CW.Climb_ladder == True:
                        print("==========")
                        print("∥完成爬梯∥")
                        print("==========")
                        time.sleep(3)
                        send.sendBodySector(29)
                    elif CW.walkinggait_stop == False:
                        CW.find_ladder()
                        CW.walkinggait()
            # elif send.Web == False:
            elif send.is_start ==False:
                if send.DIOValue == 1 or send.DIOValue == 2 or send.DIOValue == 3 or  send.DIOValue == 9 or send.DIOValue == 10 or send.DIOValue == 11 :
                    LC.draw_function()
                    send.sendHeadMotor(1,LC.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,LC.head_Vertical,100)#垂直
                    if LC.walkinggait_stop == False or LC.walkinggait_LC == True:
                        print("🔊parameter reset")
                        send.sendHeadMotor(1,LC.head_Horizontal,100)  #水平
                        send.sendHeadMotor(2,LC.head_Vertical,100)    #垂直
                        if LC.walkinggait_stop == 0:
                            send.sendBodyAuto(0,0,0,0,1,0)
                        send = Sendmessage()                #建立名稱,順便歸零,就是底線底線init
                        LC = Lift_and_Carry()          #建立名稱,順便歸零
                        LC.walkinggait_stop = True
                        LC.walkinggait_LC   = False
                        send.sendSensorReset()              #IMUreset
                        time.sleep(2)
                        send.sendBodySector(29)             #基礎站姿磁區
                        time.sleep(1.5)
                        if LC.stand_correct == True:
                            send.sendBodySector(30)             #基礎站姿調整磁區
                        time.sleep(1)
                        print("🆗🆗🆗")
                    print("LC turn off")
                    time.sleep(1)
                elif send.DIOValue == 4 or send.DIOValue == 12 :
                    if CW.walkinggait_stop == False:
                        print("🔊parameter reset")
                        send = Sendmessage()    #建立名稱,順便歸零,就是底線底線init
                        CW = Wall_Climb()    #建立名稱,順便歸零
                        send.sendBodyAuto(0,0,0,0,1,0)
                        time.sleep(2)
                        send.sendSensorReset()              #IMUreset
                        time.sleep(1)
                        send.sendBodySector(29)
                        time.sleep(1.5)
                        if CW.stand_correct == True:
                            send.sendBodySector(33)             #基礎站姿調整磁區
                    print("CW turn off")
                    CW.theta       = 0
                    CW.forward     = 0
                    CW.translation = 0
                    
                    CW.walkinggait_stop = True
                    CW.Climb_ladder     = False
                    send.sendHeadMotor(1,LC.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,LC.head_Vertical,100)#垂直
                    time.sleep(1)
                elif send.DIOValue == 0 or send.DIOValue == 8:
                    print("⣿⣿⣿⣿⣿⣿⡿⠿⠛⠛⠛⢻⡻⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿")
                    print("⣿⣿⣿⣿⣿⣿⣟⣫⡾⠛⠛⠛⠛⠛⠛⠿⣾⣽⡻⣿⣿⣿⣿⣿⣿⣿")
                    print("⣿⣿⣿⣿⣿⡟⣼⠏⠀⠀⠀⠀⠀⠀⣀⣀⡀⣙⣿⣎⢿⣿⣿⣿⣿⣿")
                    print("⣿⣿⣿⣿⣿⢹⡟⠀⠀⠀⣰⡾⠟⠛⠛⠛⠛⠛⠛⠿⣮⡻⣿⣿⣿⣿")
                    print("⣿⡿⢟⣻⣟⣽⠇⠀⠀⠀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⡹⣿⣿⣿")
                    print("⡟⣼⡟⠉⠉⣿⠀⠀⠀⠀⢿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⢟⣿⣿⣿")
                    print("⣇⣿⠁⠀⠀⣿⠀⠀⠀⠀⠘⢿⣦⣄⣀⣀⣀⣀⣤⡴⣾⣏⣾⣿⣿⣿")
                    print("⡇⣿⠀⠀⠀⣿⠀⠀⠀⠀⠀⠀⠈⠉⠛⠋⠉⠉⠀⠀⢻⣿⣿⣿⣿⣿")
                    print("⢃⣿⠀⠀⠀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣧⣿⣿⣿⣿")
                    print("⡻⣿⠀⠀⠀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣧⣿⣿⣿⣿")
                    print("⡇⣿⠀⠀⠀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⢹⣿⣿⣿⣿")
                    print("⣿⡸⢷⣤⣤⣿⡀⠀⠀⠀⠀⢠⣤⣄⣀⣀⣀⠀⠀⢠⣿⣿⣿⣿⣿⣿")
                    print("⣿⣿⣷⣿⣷⣿⡇⠀⠀⠀⠀⢸⡏⡍⣿⡏⠀⠀⠀⢸⡏⣿⣿⣿⣿⣿")
                    print("⣿⣿⣿⣿⣿⢼⡇⠀⠀⠀⠀⣸⡇⣷⣻⣆⣀⣀⣀⣼⣻⣿⣿⣿⣿⣿")
                    print("⣿⣿⣿⣿⣿⣜⠿⢦⣤⣤⡾⢟⣰⣿⣷⣭⣯⣭⣯⣥⣿⣿⣿⣿⣿⣿")
                    print("░░░░░░░░░▄░░░░░░░░░░░░░░▄░░░░░░░")
                    print("░░░░░░░░▌▒█░░░░░░░░░░░▄▀▒▌░░░░░░")
                    print("░░░░░░░░▌▒▒█░░░░░░░░▄▀▒▒▒▐░░░░░░")
                    print("░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐░░░░░░")
                    print("░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░░░░░░")
                    print("░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░░░░░░")
                    print("░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌░░░░░")
                    print("░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐░░░░░")
                    print("░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌░░░░")
                    print("░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌░░░░")
                    print("▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐░░░░")
                    print("▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌░░░")
                    print("▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐░░░░")
                    print("░▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌░░░░")
                    print("░▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐░░░░░")
                    print("░░▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌░░░░░")
            r.sleep()    

    except rospy.ROSInterruptException:
        pass
