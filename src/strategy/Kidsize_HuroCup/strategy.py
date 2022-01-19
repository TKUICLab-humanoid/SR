#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from API import Sendmessage
from IMU import IMU_Yaw_ini
import time
from Class import RightVirtualFoot,LeftVirtualFoot,Select,Image_Process,CalDistance,MovementDecide,WalkingMethod,ClimbStair

class INI ():
    def __init__(self):
        self.WalkingFlag = False
        self.UpFlag = False
        self.SureUpDistance = 15
        self.FootFlag = [False,False,False,False]
        self.Distance = [9999,9999,9999,9999,9999]#LeftOutside,LeftInside,Middle,RightInside,RightOutside
        self.BoardDistance = [9999 , 9999 , 9999]
        self.BodyState = 'INITIAL '
        self.Stair = 0
        self.WhichStrategy = 'ClimbStair'
        self.WhichLabel = 0x01
        self.NextLabel = 0x02
        self.ClimbFlag = False
        self.CS_Distance = [9999,9999]


    def DrawVirtualFoot(self):
        #FrontVirtualFoot
        send.drawImageFunction(1, 1, LeftFoot.XMin, LeftFoot.XMax, LeftFoot.YMin, LeftFoot.YMax, 0, 0, 0)
        send.drawImageFunction(2, 1, RightFoot.XMin, RightFoot.XMax, RightFoot.YMin, RightFoot.YMax, 0, 0, 5)
        send.drawImageFunction(3, 0, LeftFoot.XMax, LeftFoot.XMax, 0, 240, 0, 0, 0)
        send.drawImageFunction(4, 0, RightFoot.XMin, RightFoot.XMin, 0, 240, 0, 0, 0)
        send.drawImageFunction(5, 0, 0, 320,  RightFoot.YMax - self.SureUpDistance,  RightFoot.YMax - self.SureUpDistance, 0, 0, 0)
        #OutSideVirtualFoot
        send.drawImageFunction(6, 1, LeftFoot.XMin, LeftFoot.XMin - 50 , LeftFoot.YMin, LeftFoot.YMax, 0, 0, 0)
        send.drawImageFunction(7, 1, RightFoot.XMax, RightFoot.XMax + 50, RightFoot.YMin, RightFoot.YMax, 0, 0, 0)
        #UpLine
        # send.drawImageFunction(8, 0, LeftFoot.XMin, LeftFoot.XMiddle , LeftFoot.YMax - self.Distance[0], LeftFoot.YMax - self.Distance[1], 0, 0, 0)
        # send.drawImageFunction(9, 0, LeftFoot.XMiddle, StrategyCalculate.Middle , LeftFoot.YMax - self.Distance[1], StrategyCalculate.Middle - self.Distance[2], 0, 0, 0)
        # send.drawImageFunction(10, 0, StrategyCalculate.Middle, RightFoot.XMin , StrategyCalculate.Middle - self.Distance[2], RightFoot.YMax - self.Distance[3], 0, 0, 0)
        # send.drawImageFunction(11, 0, RightFoot.XMin, RightFoot.XMax , RightFoot.YMax - self.Distance[3], RightFoot.YMax - self.Distance[4] , 0, 0, 0)

    def DataPrinting(self):
        print()
        print('=================================================')
        print('WhichStrategy   = ',self.WhichStrategy)
        print('WhichStair      = ',initial.Stair)
        if self.WhichLabel == 0x01 :
            print('FindWhichLabel  =  RedLabel')
        elif self.WhichLabel == 0x02:
            print('FindWhichLabel  =  YellowLabel')
        elif self.WhichLabel == 0x04:
            print('FindWhichLabel  =  BlueLabel')
        elif self.WhichLabel == 0x08:
            print('FindWhichLabel  =  GreenLabel')
        elif self.WhichLabel == 0x10:
            print('FindWhichLabel  =  BlackLabel')
        elif self.WhichLabel == 0x20:
            print('FindWhichLabel  =  RedLabel')
        elif self.WhichLabel == 0x40:
            print('FindWhichLabel  =  WhiteLabel')   
        else :
            print('FindWhichLabel   = OtherLabel')
        if self.WhichStrategy == 'LiftandCarry' :
            print('VirtualFootFlag = ',self.FootFlag)
            print('FootDistance    = ',self.Distance)
            print('BoardDistance   = ',self.BoardDistance)
            print('BodyState       = ',self.BodyState)
            print('SureUpDistance  = ',self.SureUpDistance)
            print('Walking,UPFlag  = ',self.WalkingFlag , self.UpFlag)
        elif self.WhichStrategy == 'ClimbStair':
            print('CS_FootDistance = ',self.CS_Distance)
            print('BodyState       = ',self.BodyState)
            print('SureUpDistance  = ',self.SureUpDistance)
            print('Walking,ClimbFlag  = ',self.WalkingFlag , self.ClimbFlag)
        print('=================================================')
        print()
    def StrategyClassify(self) :
        if send.DIOValue == 24:
            self.WhichStrategy = 'LiftandCarry'
        else :
            self.WhichStrategy = 'ClimbStair'

    def UPStrategy(self) :
        self.WhichLabel,self.NextLabel= StrategySelect.WhichStair(self.Stair,self.WhichStrategy)
        self.FootFlag = StrategyImage.FindWoodUP(self.WhichLabel,send.Label_Model)
        self.Distance = StrategyCalculate.CalculateUP(self.WhichLabel,send.Label_Model)
        self.BoardDistance = StrategyCalculate.CalBoardDistance(self.NextLabel,send.Label_Model,self.Distance)
        self.BodyState = StrategyMove.DecideUP(self.Distance,self.FootFlag,self.SureUpDistance)
        self.DataPrinting()
        self.WalkingFlag , self.UpFlag = StrategyMethod.Method(self.BodyState,self.WalkingFlag,self.ClimbFlag)
    
    def DownStrategy(self) :
        self.WhichLabel,self.NextLabel = StrategySelect.WhichStair(self.Stair,self.WhichStrategy)
        self.FootFlag = StrategyImage.FindWoodDown(self.WhichLabel,send.Label_Model)
        self.Distance = StrategyCalculate.CalculateDown(self.WhichLabel,send.Label_Model)
        self.BodyState = StrategyMove.DecideDown(self.Distance,self.FootFlag,self.SureUpDistance)
        self.DataPrinting()
        self.WalkingFlag , self.UpFlag = StrategyMethod.Method(self.BodyState,self.WalkingFlag,self.ClimbFlag)

    def BeforeClimb(self) :
        if self.WalkingFlag == False :
            send.sendBodyAuto(200,0,0,0,1,0)
            time.sleep(1)
            self.WalkingFlag = True
        else :
            self.WhichLabel,self.NextLabel= StrategySelect.WhichStair(self.Stair,self.WhichStrategy)
            self.CS_Distance = CS_Strategy.CS_DistanceCal(self.WhichLabel,send.Label_Model,self.NextLabel)
            self.BodyState,self.ClimbFlag = CS_Strategy.CS_MovementDecide(self.CS_Distance,self.SureUpDistance)
            self.DataPrinting()
            self.WalkingFlag , self.UpFlag = StrategyMethod.Method(self.BodyState,self.WalkingFlag,self.ClimbFlag)
    
    def StartClimb(self):
        self.ClimbFlag = True
        self.BodyState = CS_Strategy.CS_StartClimb()
        self.DataPrinting()

if __name__ == '__main__':
    try:
        send = Sendmessage()
        initial = INI ()
        RightFoot = RightVirtualFoot()
        LeftFoot = LeftVirtualFoot()
        StrategySelect = Select()
        StrategyImage = Image_Process()
        StrategyCalculate = CalDistance()
        StrategyMove = MovementDecide()
        StrategyMethod = WalkingMethod()
        CS_Strategy = ClimbStair()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            initial.StrategyClassify()
            send.sendSensorReset()
            send.sendHeadMotor(2,1370,100)
            send.sendHeadMotor(1,2048,100)
            initial.DrawVirtualFoot()
            if send.Web == True:
                if initial.Stair == 0 :
                    if initial.WhichStrategy == 'LiftandCarry' :
                        if initial.WalkingFlag == False :
                            send.sendBodyAuto(200,0,0,0,1,0)
                            time.sleep(1)
                            initial.WalkingFlag = True
                        else:
                            if initial.UpFlag == False :
                                ST = time.time()
                                initial.UPStrategy()
                                ED = time.time()
                                print('Time Used = ',ED-ST)
                            elif initial.UpFlag == True :
                                initial.FootFlag = [False,False,False,False]
                                initial.Distance = [9999,9999,9999,9999,9999]#LeftOutside,LeftInside,Middle,RightInside,RightOutside
                                initial.BoardDistance = [9999 , 9999 , 9999]
                                initial.Stair = initial.Stair + 1
                                initial.UpFlag = False
                    else :
                        if initial.ClimbFlag == False :
                            initial.BeforeClimb()
                        else :
                            if initial.BodyState == 'Stair_Top' :
                                initial.DataPrinting()
                            else :
                                initial.StartClimb()

                elif initial.Stair == 1 :
                    if initial.WalkingFlag == False :
                        send.sendBodyAuto(200,0,0,0,1,0)
                        time.sleep(1)
                        initial.WalkingFlag = True
                    else:
                        if initial.UpFlag == False :
                            ST = time.time()
                            initial.UPStrategy()
                            ED = time.time()
                            print('Time Used = ',ED-ST)
                        elif initial.UpFlag == True :
                            initial.FootFlag = [False,False,False,False]
                            initial.Distance = [9999,9999,9999,9999,9999]#LeftOutside,LeftInside,Middle,RightInside,RightOutside
                            initial.BoardDistance = [9999 , 9999 , 9999]
                            initial.Stair = initial.Stair + 1
                            initial.UpFlag = False
                            
                elif initial.Stair == 2 :
                    if initial.WalkingFlag == False :
                        send.sendBodyAuto(200,0,0,0,1,0)
                        time.sleep(1)
                        initial.WalkingFlag = True
                    else:
                        if initial.UpFlag == False :
                            ST = time.time()
                            initial.UPStrategy()
                            ED = time.time()
                            print('Time Used = ',ED-ST)
                        elif initial.UpFlag == True :
                            initial.FootFlag = [False,False,False,False]
                            initial.Distance = [9999,9999,9999,9999,9999]#LeftOutside,LeftInside,Middle,RightInside,RightOutside
                            initial.BoardDistance = [9999 , 9999 , 9999]
                            initial.Stair = initial.Stair + 1
                            initial.UpFlag = False

                elif initial.Stair == 3 :
                    if initial.WalkingFlag == False :
                        send.sendBodyAuto(200,0,0,0,1,0)
                        time.sleep(1)
                        initial.WalkingFlag = True
                    else:
                        if initial.UpFlag == False :
                            initial.DownStrategy()
                        elif initial.UpFlag == True :
                            initial.FootFlag = [False,False,False,False]
                            initial.Distance = [9999,9999,9999,9999,9999]#LeftOutside,LeftInside,Middle,RightInside,RightOutside
                            initial.BoardDistance = [9999 , 9999 , 9999]
                            initial.Stair = initial.Stair + 1
                            initial.UpFlag = False

                elif initial.Stair == 4 :
                    if initial.WalkingFlag == False :
                        send.sendBodyAuto(200,0,0,0,1,0)
                        time.sleep(1)
                        initial.WalkingFlag = True
                    else:
                        if initial.UpFlag == False :
                            initial.DownStrategy()
                        elif initial.UpFlag == True :
                            initial.FootFlag = [False,False,False,False]
                            initial.Distance = [9999,9999,9999,9999,9999]#LeftOutside,LeftInside,Middle,RightInside,RightOutside
                            initial.BoardDistance = [9999 , 9999 , 9999]
                            initial.Stair = initial.Stair + 1
                            initial.UpFlag = False

                elif initial.Stair == 5 :
                    if initial.WalkingFlag == False :
                        send.sendBodyAuto(200,0,0,0,1,0)
                        time.sleep(1)
                        initial.WalkingFlag = True
                    else:
                        if initial.UpFlag == False :
                            initial.DownStrategy()
                        elif initial.UpFlag == True :
                            initial.FootFlag = [False,False,False,False]
                            initial.Distance = [9999,9999,9999,9999,9999]#LeftOutside,LeftInside,Middle,RightInside,RightOutside
                            initial.BoardDistance = [9999 , 9999 , 9999]
                            initial.Stair = initial.Stair + 1
                            initial.UpFlag = False
                elif initial.Stair == 6 :
                    if initial.WalkingFlag == True :
                        send.sendBodyAuto(0,0,0,0,1,0)
                        time.sleep(0.5)
                        initial.WalkingFlag = False
                    else :
                        initial.BodyState = 'Finish'
                        #initial.WalkingFlag = False
                        initial.UpFlag = False
                        initial.DataPrinting()

            if send.Web == False:
                if initial.WalkingFlag == True:
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.05) 
                    send.sendContinuousValue(0,0,0,0,0)
                    time.sleep(0.03)
                    send.sendBodySector(29)
                    initial.WalkingFlag = False
                #initial.Stair = 0
                initial = INI ()
                initial.DataPrinting()
                #print(WhichLabel)
            r.sleep()
    except rospy.ROSInterruptException:
        pass

    #UP#
    # initial.WhichLabel,initial.NextLabel = StrategySelect.WhichStair(initial.Stair,initial.Strategy)
    # initial.FootFlag = StrategyImage.FindWoodUP(initial.WhichLabel,send.Label_Model)
    # initial.Distance = StrategyCalculate.CalculateUP(initial.WhichLabel,send.Label_Model)
    # initial.BoardDistance = StrategyCalculate.CalBoardDistance(initial.NextLabel,send.Label_Model,initial.Distance)
    # initial.BodyState = StrategyMove.DecideUP(initial.Distance,initial.FootFlag,initial.SureUpDistance)
    # initial.DataPrinting()
    # initial.WalkingFlag , initial.UpFlag = StrategyMethod.Method(initial.BodyState,initial.WalkingFlag)
    
    #Down#
    # initial.WhichLabel,initial.NextLabel = StrategySelect.WhichStair(initial.Stair,initial.Strategy)
    # initial.FootFlag = StrategyImage.FindWoodDown(initial.WhichLabel,send.Label_Model)
    # initial.Distance = StrategyCalculate.CalculateDown(initial.WhichLabel,send.Label_Model)
    # initial.BodyState = StrategyMove.DecideDown(initial.Distance,initial.FootFlag,initial.SureUpDistance)
    # initial.DataPrinting()
    # initial.WalkingFlag , initial.UpFlag = StrategyMethod.Method(initial.BodyState,initial.WalkingFlag)