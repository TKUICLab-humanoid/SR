#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
from SR_API_wei_re import Send_distance
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
            # print(send.DIOValue)
            if send.is_start ==True:
                if send.DIOValue == 1 or  send.DIOValue == 25 or send.DIOValue == 9 or send.DIOValue == 17 or send.DIOValue == 3 or  send.DIOValue == 27 or send.DIOValue == 11 or  send.DIOValue == 19:
                    distance.printf()
                    distance.draw_function()
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
                    start=time.time()
                    #機器人暫停且不是在做上板
                    if distance.layer == 7:
                        pass
                    elif distance.walkinggait_stop == True and distance.walkinggait_LC == False:
                        send.sendBodyAuto(700,0,0,0,1,0)
                        distance.walkinggait_stop = False
                    elif distance.walkinggait_stop == True and distance.walkinggait_LC == True:
                        send.sendBodyAuto(0,0,0,0,1,0)
                        distance.walkinggait_stop = 0
                        distance.walkinggait_LC   = 0
                    #還沒找到板子（找板 上板）
                    elif distance.walkinggait_stop == False:
                        distance.find_board(distance.layer)
                        distance.walkinggait(distance.layer)
                    
                    #distance.print_state()
                    end=time.time()
                    print(end-start)
                # elif send.Web == False:
                # elif send.DIOValue == 4 or  send.DIOValue == 28 or send.DIOValue == 12 or  send.DIOValue == 20:
                #     send.drawImageFunction(1,0,0,320,distance.knee,distance.knee,255,0,0)#膝蓋的橫線
                #     send.drawImageFunction(2,0,distance.f_ll,distance.f_ll,0,240,255,0,0)#ll的線
                #     send.drawImageFunction(3,0,distance.f_lr,distance.f_lr,0,240,255,0,0)#lr的線
                #     send.drawImageFunction(4,0,distance.f_rl,distance.f_rl,0,240,255,0,0)#rl的線
                #     send.drawImageFunction(5,0,distance.f_rr,distance.f_rr,0,240,255,0,0)#rr的線
                #     send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                #     send.sendHeadMotor(2,distance.head_Vertical,100)#垂直

                #     if climb.stop_flag == 1 and climb.up_ladder_flag == 0:
                #         send.sendBodyAuto(500,0,0,0,1,0)
                #         climb.stop_flag = 0
                #     elif climb.stop_flag == 1 and climb.up_ladder_flag == 1:
                #         send.sendBodySector(29)
                #     elif climb.stop_flag == 0 :
                #         climb.find_ladder()
                #         climb.up_ladder()
            elif send.is_start ==False:
                # print('web',send.Web)
                if send.DIOValue == 1 or  send.DIOValue == 25 or send.DIOValue == 9 or send.DIOValue == 17 or send.DIOValue == 3 or  send.DIOValue == 27 or send.DIOValue == 11 or  send.DIOValue == 19:
                    distance.draw_function()
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
                    if distance.walkinggait_stop == False or distance.walkinggait_LC == True:
                        print("turn off")
                        if distance.walkinggait_stop == 0:
                            send.sendBodyAuto(0,0,0,0,1,0)
                        send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
                        distance = Send_distance()#建立名稱,順便歸零
                        # distance.layer            = 1
                        distance.walkinggait_stop = True
                        distance.walkinggait_LC   = False
                        send.sendSensorReset()
                        time.sleep(2)
                        send.sendBodySector(29)
                        time.sleep(1)
                        send.sendBodySector(299)
                        time.sleep(1)
                        send.sendBodySector(22)
                    print("LC turn off")
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    # send.sendHeadMotor(2,2698,100)#垂直
                    time.sleep(1)
                elif send.DIOValue == 4 or  send.DIOValue == 28 or send.DIOValue == 12 or  send.DIOValue == 20:
                    # if climb.stop_flag == 0:
                    #     send = Sendmessage() #建立名稱,順便歸零,就是底線底線init
                    #     send.sendBodyAuto(0,0,0,0,1,0)
                    #     time.sleep(2)
                    #     send.sendBodySector(29)
                    # print("CW turn off")
                    # climb.theta = 0
                    # climb.speed = 0
                    # climb.yspeed=0
                    # climb = Send_Climb()#建立名稱,順便歸零
                    # climb.stop_flag = 1
                    # climb.up_ladder_flag = 0
                    send.sendHeadMotor(1,distance.head_Horizontal,100)#水平
                    send.sendHeadMotor(2,distance.head_Vertical,100)#垂直
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
