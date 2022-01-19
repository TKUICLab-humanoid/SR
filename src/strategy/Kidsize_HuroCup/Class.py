import rospy
import numpy as np
from API import Sendmessage
import time
send = Sendmessage()
class LeftVirtualFoot():
    
    def __init__(self):
        self.XMin = 105
        self.XMax = self.XMin + 50
        self.YMin = 135
        self.YMax = self.YMin + 80
        self.XMiddle = (self.XMin + self.XMax)/2

class RightVirtualFoot():

    def __init__(self):
        self.XMin = 165
        self.XMax = self.XMin + 50
        self.YMin = 135
        self.YMax = self.YMin + 80
        self.XMiddle = (self.XMin + self.XMax)/2

class ColorNum ():
    def __init__(self):
        self.OrangeLabel = 0x01
        self.YellowLabel = 0x02
        self.BlueLabel   = 0x04
        self.GreenLabel  = 0x08
        self.BlackLabel  = 0x10
        self.RedLabel    = 0x20
        self.WhiteLabel  = 0x40
        
class Image_Process():

    def __init__(self):
        self.ImageData = [0]*320*240
        self.ImageWidth = 320
        self.Label = [0]*320*240
        self.FootFlag = [False,False,False,False] #Left,LeftFront,RightFront,Right

    def Exchange(self,InComingData):
        self.ImageData = [0]*320*240
        self.Label = [0]*320*240
        self.ImageData = InComingData
        for i in range(len(self.ImageData)):
            if self.ImageData[i] == 0x01 :
                self.Label[i] = 'OrangeLabel'
            elif self.ImageData[i] == 0x02:
                self.Label[i] = 'YellowLabel'
            elif self.ImageData[i] == 0x04:
                self.Label[i] = 'BlueLabel'
            elif self.ImageData[i] == 0x08:
                self.Label[i] = 'GreenLabel'
            elif self.ImageData[i] == 0x10:
                self.Label[i] = 'BlackLabel'
            elif self.ImageData[i] == 0x20:
                self.Label[i] = 'RedLabel'
            elif self.ImageData[i] == 0x40:
                self.Label[i] = 'WhiteLabel'   
            else :
                self.Label[i] = 'OtherLabel'
        return self.Label

    def FindWoodUP(self,ColorLabel,Label):
        self.FootFlag = [False,False,False,False]

        for i in range( LeftVirtualFoot().YMax , LeftVirtualFoot().YMin ,-1) : 
            if Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMiddle) ] == ColorLabel :
                self.FootFlag[1] = True
                break

        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) : 
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMiddle) ] == ColorLabel :
                self.FootFlag[2] = True
                break   

        if self.FootFlag[1] == False and self.FootFlag[2] == False :
            for i in range( LeftVirtualFoot().XMin, LeftVirtualFoot().XMin - 50 ,-1) :
                if Label[ int( self.ImageWidth * LeftVirtualFoot().YMin + i ) ] == ColorLabel :
                    self.FootFlag[0] = True
                    break

            for i in range( RightVirtualFoot().XMax , RightVirtualFoot().XMax + 50 , 1) :
                if Label[ int(self.ImageWidth * RightVirtualFoot().YMin + i) ] == ColorLabel :
                    self.FootFlag[3] = True
                    break
            
        elif self.FootFlag[1] == False and self.FootFlag[2] == True :
            for i in range( RightVirtualFoot().XMax , RightVirtualFoot().XMax + 50 , 1) :
                if Label[ int(self.ImageWidth * RightVirtualFoot().YMin + i) ] == ColorLabel :
                    self.FootFlag[3] = True
                    break

        elif self.FootFlag[1] == True and self.FootFlag[2] == False :
            for i in range( LeftVirtualFoot().XMin, LeftVirtualFoot().XMin - 50 ,-1) :
                if Label[ int( self.ImageWidth * LeftVirtualFoot().YMin + i ) ] == ColorLabel :
                    self.FootFlag[0] = True
                    break
        else :
            pass
        return  self.FootFlag

    def FindWoodDown(self,ColorLabel,Label):
        self.FootFlag = [False,False,False,False]
        for i in range( LeftVirtualFoot().YMax , LeftVirtualFoot().YMin ,-1) : 
            if Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMiddle) ] != ColorLabel :
                self.FootFlag[1] = True
                break

        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) : 
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMiddle) ] != ColorLabel :
                self.FootFlag[2] = True
                break   

        if self.FootFlag[1] == False and self.FootFlag[2] == False :
            for i in range( LeftVirtualFoot().XMin, LeftVirtualFoot().XMin - 50 ,-1) :
                if Label[ int( self.ImageWidth * LeftVirtualFoot().YMin + i ) ] != ColorLabel :
                    self.FootFlag[0] = True
                    break

            for i in range( RightVirtualFoot().XMax , RightVirtualFoot().XMax + 50 , 1) :
                if Label[ int(self.ImageWidth * RightVirtualFoot().YMin + i) ] != ColorLabel :
                    self.FootFlag[3] = True
                    break
            
        elif self.FootFlag[1] == False and self.FootFlag[2] == True :
            for i in range( RightVirtualFoot().XMax , RightVirtualFoot().XMax + 50 , 1) :
                if Label[ int(self.ImageWidth * RightVirtualFoot().YMin + i) ] != ColorLabel :
                    self.FootFlag[3] = True
                    break

        elif self.FootFlag[1] == True and self.FootFlag[2] == False :
            for i in range( LeftVirtualFoot().XMin, LeftVirtualFoot().XMin - 50 ,-1) :
                if Label[ int( self.ImageWidth * LeftVirtualFoot().YMin + i ) ] != ColorLabel :
                    self.FootFlag[0] = True
                    break
        else :
            pass
        return  self.FootFlag

class Select():

    def __init__(self):
        self.WhichStrategy = 'LiftandCarry'
        self.WhichLabel = 0x20 #RedLabel
        self.NextLabel = 0x04 #BlueLabel

    def WhichStair(self,Stair,Strategy):
        if Strategy == 'LiftandCarry' :
            if Stair == 0 :
                self.WhichLabel = 0x20 #RedLabel
                self.NextLabel = 0x04  #BlueLabel
            elif Stair == 1 :
                self.WhichLabel = 0x04 #BlueLabel
                self.NextLabel = 0x02  #YellowLabel
            elif Stair == 2 :
                self.WhichLabel = 0x02 #YellowLabel
                self.NextLabel = 0x04  #BlueLabel
            elif Stair == 3 :
                self.WhichLabel = 0x02 #YellowLabel
                self.NextLabel = 0x04  #BlueLabel
            elif Stair == 4 :
                self.WhichLabel = 0x04 #BlueLabel
                self.NextLabel = 0x20  #RedLabel
            elif Stair == 5 :
                self.WhichLabel = 0x20 #RedLabel
                self.NextLabel = 0x08  #GreenLabel
        elif Strategy == 'ClimbStair' :
            self.WhichLabel = 0x20 #RedLabel
            self.NextLabel = 0x02 #RedLabel
        return self.WhichLabel ,self.NextLabel

class CalDistance():

    def __init__(self):
        self.ImageWidth = 320
        self.Distance = [9999 , 9999 , 9999 , 9999 , 9999] #LeftOutside,LeftInside,Middle,RightInside,RightOutside
        self.BoardDistance = [9999 , 9999 , 9999]
        self.Middle = (LeftVirtualFoot().XMax + RightVirtualFoot().XMin)/2

    def CalculateUP(self,ColorLabel,Label):
        self.Distance = [9999 , 9999 , 9999 , 9999 , 9999]
        for i in range( LeftVirtualFoot().YMax , LeftVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMin) ] == ColorLabel :
                self.Distance[0] = LeftVirtualFoot().YMax - i
                break

        for i in range( LeftVirtualFoot().YMax , LeftVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMax) ] == ColorLabel :
                self.Distance[1] = LeftVirtualFoot().YMax - i
                break

        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + self.Middle) ] == ColorLabel :
                self.Distance[2] = LeftVirtualFoot().YMax - i
                break
        
        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMin) ] == ColorLabel :
                self.Distance[3] = RightVirtualFoot().YMax - i
                break

        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMax) ] == ColorLabel :
                self.Distance[4] = RightVirtualFoot().YMax - i
                break
        return self.Distance

    def CalculateDown(self,ColorLabel,Label):
        self.Distance = [9999 , 9999 , 9999 , 9999 , 9999]
        for i in range( LeftVirtualFoot().YMax , LeftVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMin) ] != ColorLabel :
                self.Distance[0] = LeftVirtualFoot().YMax - i
                break

        for i in range( LeftVirtualFoot().YMax , LeftVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMax) ] != ColorLabel :
                self.Distance[1] = LeftVirtualFoot().YMax - i
                break

        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + self.Middle) ] != ColorLabel :
                self.Distance[2] = LeftVirtualFoot().YMax - i
                break
        
        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMin) ] != ColorLabel :
                self.Distance[3] = RightVirtualFoot().YMax - i
                break

        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMax) ] != ColorLabel :
                self.Distance[4] = RightVirtualFoot().YMax - i
                break
        return self.Distance

    def CalBoardDistance (self,NextLabel,Label,Distance) :
        self.BoardDistance = [9999 , 9999 , 9999]
        for i in range( LeftVirtualFoot().YMin , LeftVirtualFoot().YMin - 100 ,-1) :
            if Label[ int(self.ImageWidth * i + (LeftVirtualFoot().XMin - 50)) ] == NextLabel :
                self.BoardDistance[0] = abs(LeftVirtualFoot().YMax - i - Distance[0])
                break
        for i in range( RightVirtualFoot().YMin , RightVirtualFoot().YMin - 100 ,-1) :
            if Label[ int(self.ImageWidth * i + self.Middle) ] == NextLabel :
                self.BoardDistance[1] = abs(LeftVirtualFoot().YMax - i - Distance[2])
                break
        for i in range( RightVirtualFoot().YMin , RightVirtualFoot().YMin - 100 ,-1) :
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMax) ] == NextLabel :
                self.BoardDistance[2] = abs(LeftVirtualFoot().YMax - i - Distance[4])
                break
        return self.BoardDistance

class MovementDecide() :
    
    def __init__(self):
        self.BodyState = 'INITIAL'

    def DecideUP(self,Distance,FootFlag,SureUpDistance):
        if FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == False :
            self.BodyState = 'BigFront'
        elif FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == True :
            self.BodyState = 'BigRightRotation'
        elif FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == True and FootFlag[3] == False :
            if Distance[3] < SureUpDistance and Distance[4] < SureUpDistance :
                if Distance[3] > Distance[4] :
                    self.BodyState = 'RightRotation'
                elif Distance[3] < Distance[4] :
                    self.BodyState = 'LeftRotation'
                else :
                    self.BodyState = 'SmallFront'
            else :
                self.BodyState = 'SmallFront'
        elif FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == True and FootFlag[3] == True :
            self.BodyState = 'BigRightRotation'
        elif FootFlag[0] == False and FootFlag[1] == True and FootFlag[2] == False and FootFlag[3] == False :
            if Distance[0] < SureUpDistance and Distance[1] < SureUpDistance :
                if Distance[0] > Distance[1] :
                    self.BodyState = 'RightRotation'
                elif Distance[0] < Distance[1] :
                    self.BodyState = 'LeftRotation'
                else :
                    self.BodyState = 'SmallFront'
            else :
                self.BodyState = 'SmallFront'
        elif FootFlag[0] == False and FootFlag[1] == True and FootFlag[2] == True and FootFlag[3] == False :
            if Distance[1] <= SureUpDistance and Distance[2] <= SureUpDistance and Distance[3] <= SureUpDistance : 
                self.BodyState = 'UP'
            elif Distance[2] <= SureUpDistance :
                if Distance[1] <= SureUpDistance and Distance[3] <= SureUpDistance:
                    self.BodyState = 'UP'
                elif Distance[1] < Distance[3] :
                    self.BodyState = 'LeftRoatation'
                elif Distance[1] > Distance[3] :
                    self.BodyState = 'RightRoatation'
                elif Distance[1] + 5 < Distance[3] :
                    self.BodyState = 'UP'
                elif Distance[1] > Distance[3] + 5 :
                    self.BodyState = 'UP'    
            elif Distance[0] <= SureUpDistance and Distance[4] <= SureUpDistance : 
                if Distance[1] <= SureUpDistance + 5 and Distance[3] <= SureUpDistance + 5 :
                    self.BodyState = 'UP'
                elif Distance[2] <= SureUpDistance and ((Distance[0] <= SureUpDistance and Distance[1] <= SureUpDistance) or (Distance[3] <= SureUpDistance and Distance[4] <= SureUpDistance)):
                    self.BodyState = 'UP'
                elif Distance[2] > SureUpDistance and  (Distance[1] <= SureUpDistance and Distance[3] <= SureUpDistance) : 
                    self.BodyState  = 'UP'
                else:
                    self.BodyState = 'SmallFront'
            elif Distance[1] > 60 and Distance[2] > 60 and Distance[3] > 60  :
                self.BodyState = 'BigFront'
            elif Distance[1] > SureUpDistance + 30 and Distance[2] > SureUpDistance + 30 and Distance[3] > SureUpDistance + 30 :
                self.BodyState = 'SmallFront'
            elif Distance[0] <= SureUpDistance or Distance[1] <= SureUpDistance :
                if Distance[4] <= SureUpDistance or Distance[3] <= SureUpDistance :
                    self.BodyState = 'UP' 
                elif Distance[4] > SureUpDistance or Distance[3] > SureUpDistance :
                    self.BodyState = 'LeftRotation'
                elif Distance[0] < Distance[4] :
                    self.BodyState = 'RightRotation'
                elif Distance[0] > Distance[4] :
                    self.BodyState = 'LeftRotation'
            elif Distance[3] <= SureUpDistance or Distance[4] <= SureUpDistance :
                if Distance[0] <= SureUpDistance or Distance[1] <= SureUpDistance :
                    self.BodyState = 'UP'
                elif Distance[0] > SureUpDistance or Distance[1] > SureUpDistance :
                    self.BodyState = 'RightRotation'
                elif Distance[0] < Distance[4] :
                    self.BodyState = 'RightRotation'
                elif Distance[0] > Distance[4] :
                    self.BodyState = 'LeftRotation'
            elif Distance[0] > Distance[4] :
                self.BodyState = 'RightRoatation'
            elif Distance[0] < Distance[4]  :
                self.BodyState = 'LeftRotation' 
        elif FootFlag[0] == False and FootFlag[1] == True and FootFlag[2] == True and FootFlag[3] == True :
            self.BodyState = 'BigRightRotation'
        elif FootFlag[0] == True and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == False :
            self.BodyState = 'BigLeftRotation'
        elif FootFlag[0] == True and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == True :
            self.BodyState = 'BigRightRotation'
        elif FootFlag[0] == True and FootFlag[1] == True and FootFlag[2] == False and FootFlag[3] == False :
            self.BodyState = 'BigLeftRotation'
        else :
            self.BodyState = 'SmallFront'
        return self.BodyState

    
    def DecideDown(self,Distance,FootFlag,SureUpDistance):
        if FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == False :
            self.BodyState = 'BigFront'
        elif FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == True :
            self.BodyState = 'BigLeftRotation'
        elif FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == True and FootFlag[3] == False :
            if Distance[3] < SureUpDistance and Distance[4] < SureUpDistance :
                if Distance[3] > Distance[4] :
                    self.BodyState = 'LeftRotation'
                elif Distance[3] < Distance[4] :
                    self.BodyState = 'RightRotation'
                else :
                    self.BodyState = 'SmallFront'
            else :
                self.BodyState = 'SmallFront'
        elif FootFlag[0] == False and FootFlag[1] == False and FootFlag[2] == True and FootFlag[3] == True :
            self.BodyState = 'BigLeftRotation'
        elif FootFlag[0] == False and FootFlag[1] == True and FootFlag[2] == False and FootFlag[3] == False :
            if Distance[0] < SureUpDistance and Distance[1] < SureUpDistance :
                if Distance[0] > Distance[1] :
                    self.BodyState = 'LeftRotation'
                elif Distance[0] < Distance[1] :
                    self.BodyState = 'RightRotation'
                else :
                    self.BodyState = 'SmallFront'
            else :
                self.BodyState = 'SmallFront'
        elif FootFlag[0] == False and FootFlag[1] == True and FootFlag[2] == True and FootFlag[3] == False :
            if Distance[1] <= SureUpDistance and Distance[2] <= SureUpDistance and Distance[3] <= SureUpDistance : 
                self.BodyState = 'Down'
            elif Distance[2] <= SureUpDistance - 5 :
                if Distance[1] <= SureUpDistance and Distance[3] <= SureUpDistance:
                    self.BodyState = 'Down'
                elif Distance[1] < Distance[3] :
                    self.BodyState = 'RightRoatation'
                elif Distance[1] > Distance[3] :
                    self.BodyState = 'LeftRoatation'
                elif Distance[1] + 5 < Distance[3] :
                    self.BodyState = 'Down'
                elif Distance[1] > Distance[3] + 5 :
                    self.BodyState = 'Down'              
            elif Distance[0] <= SureUpDistance and Distance[4] <= SureUpDistance :
                if Distance[0] == 0:
                    if Distance[4] <= SureUpDistance - 5 :
                        self.BodyState = 'Down'
                    else    :
                        self.BodyState = 'LeftRotation'
                else :
                    if Distance[1] <= SureUpDistance + 5 and Distance[3] <= SureUpDistance + 5 :
                        self.BodyState = 'Down'
                    elif Distance[2] <= SureUpDistance and ((Distance[0] <= SureUpDistance and Distance[1] <= SureUpDistance) or (Distance[3] <= SureUpDistance and Distance[4] <= SureUpDistance)):
                        self.BodyState = 'Down'
                    elif Distance[2] > SureUpDistance and  (Distance[1] <= SureUpDistance and Distance[3] <= SureUpDistance) : 
                        self.BodyState = 'Down'
                    else:
                        self.BodyState = 'SmallFront'
            elif Distance[1] > 60 and Distance[2] > 60 and Distance[3] > 60  :
                self.BodyState = 'BigFront'
            elif Distance[1] > SureUpDistance + 30 and Distance[2] > SureUpDistance + 30 and Distance[3] > SureUpDistance + 30 :
                self.BodyState = 'SmallFront'
            elif Distance[0] <= SureUpDistance or Distance[1] <= SureUpDistance :
                if Distance[0] == 0:
                    if Distance[4] <= SureUpDistance - 5 :
                        self.BodyState = 'Down'
                    else    :
                        self.BodyState = 'LeftRotation'
                else :
                    if Distance[4] <= SureUpDistance and Distance[3] <= SureUpDistance :
                        self.BodyState = 'Down' 
                    elif Distance[4] > SureUpDistance  or Distance[3] > SureUpDistance  :
                        self.BodyState = 'LeftRotation'
                    elif Distance[0] < Distance[4] :
                        self.BodyState = 'LeftRotation'
                    elif Distance[0] > Distance[4] :
                        self.BodyState = 'RightRotation'
            elif Distance[3] <= SureUpDistance or Distance[4] <= SureUpDistance :
                if Distance[0] <= SureUpDistance or Distance[1] <= SureUpDistance :
                    self.BodyState = 'Down'
                elif Distance[0] > SureUpDistance or Distance[1] > SureUpDistance :
                    self.BodyState = 'LeftRotation'
                elif Distance[0] < Distance[4] :
                    self.BodyState = 'LeftRotation'
                elif Distance[0] > Distance[4] :
                    self.BodyState = 'RightRotation'
            elif Distance[0] > Distance[4] :
                self.BodyState = 'LeftRoatation'
            elif Distance[0] < Distance[4]  :
                self.BodyState = 'RightRotation' 
        elif FootFlag[0] == False and FootFlag[1] == True and FootFlag[2] == True and FootFlag[3] == True :
            self.BodyState = 'BigLeftRotation'
        elif FootFlag[0] == True and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == False :
            self.BodyState = 'BigRightRotation'
        elif FootFlag[0] == True and FootFlag[1] == False and FootFlag[2] == False and FootFlag[3] == True :
            self.BodyState = 'BigLeftRotation'
        elif FootFlag[0] == True and FootFlag[1] == True and FootFlag[2] == False and FootFlag[3] == False :
            self.BodyState = 'BigRightRotation'
        else :
            self.BodyState = 'SmallFront'
        return self.BodyState

class ClimbStair ():
    def __init__(self):
        self.StairDistance = [9999]
        self.BodyState = 'INITIAL'
        self.Distance = [9999 , 9999]
        self.ClimbFlag = False
        self.ImageWidth = 320

    def CS_DistanceCal(self,ColorLabel,Label,NextLabel):
        self.Distance = [9999 , 9999]
        for i in range( LeftVirtualFoot().YMax , LeftVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMin) ] == ColorLabel or Label[ int(self.ImageWidth * i + LeftVirtualFoot().XMin) ] == NextLabel:
                self.Distance[0] = LeftVirtualFoot().YMax - i
                break
        for i in range( RightVirtualFoot().YMax , RightVirtualFoot().YMin ,-1) :
            if Label[ int(self.ImageWidth * i + RightVirtualFoot().XMax) ] == ColorLabel or Label[ int(self.ImageWidth * i + RightVirtualFoot().XMax) ] == NextLabel :
                self.Distance[1] = RightVirtualFoot().YMax - i
                break
        return self.Distance

    def CS_MovementDecide(self,Distance,SureUpDistance):
        if Distance[0] <= SureUpDistance and Distance[1] <= SureUpDistance :
            self.BodyState = 'Climb_First_Stair'
            self.ClimbFlag = True
        elif Distance[0] > 60  and Distance[1] > 60 :
            if Distance[0] > Distance[1]:
                self.BodyState = 'BigRightRotation'
            elif Distance[0] < Distance[1] :
                self.BodyState = 'BigLeftRotation'
            else :
                self.BodyState = 'BigFront'
        elif Distance[0] > SureUpDistance + 30  and Distance[1] > SureUpDistance + 30:
            if Distance[0] > Distance[1]:
                self.BodyState = 'RightRotation'
            elif Distance[0] < Distance[1] :
                self.BodyState = 'LeftRotation'
            else :
                self.BodyState = 'SmallFront'
        elif Distance[0] > Distance[1] :
            if Distance[0] <= SureUpDistance and Distance[1] <= SureUpDistance + 5:
                self.BodyState = 'Climb_First_Stair'
                self.ClimbFlag = True
            elif Distance[1] <= SureUpDistance and Distance[0] <= SureUpDistance + 5:
                self.BodyState = 'Climb_First_Stair'
                self.ClimbFlag = True
            else :
                self.BodyState = 'RightRotation'
        elif Distance[0] < Distance[1] :
            if Distance[0] <= SureUpDistance and Distance[1] <= SureUpDistance + 5:
                self.BodyState = 'Climb_First_Stair'
                self.ClimbFlag = True
            elif Distance[1] <= SureUpDistance and Distance[0] <= SureUpDistance + 5:
                self.BodyState = 'Climb_First_Stair'
                self.ClimbFlag = True
            else :
                self.BodyState = 'LeftRotation'
            
        return self.BodyState,self.ClimbFlag

    def CS_StartClimb(self) : 
        if self.ClimbFlag == True :
            if self.BodyState == 'Climb_First_Stair' :
                send.sendBodySector(101)
                time.sleep(5)
                self.BodyState = 'Climb_Second_Stair'
            elif self.BodyState == 'Climb_Second_Stair' :
                send.sendBodySector(102)
                time.sleep(5)
                self.BodyState = 'Climb_Third_Stair'
            elif self.BodyState == 'Climb_Third_Stair' :
                send.sendBodySector(103)
                time.sleep(5)
                self.BodyState = 'Climb_Forth_Stair'
            elif self.BodyState == 'Climb_Forth_Stair' :
                send.sendBodySector(104)
                time.sleep(5)
                self.BodyState = 'Stair_Top'
                self.ClimbFlag = False
        return self.BodyState

class WalkingMethod():
    def __init__(self):
        self.UpFlag = False
    def Method(self,BodyState,WalkingFlag,ClimbFlag) :
        if BodyState == 'BigFront' :
            send.sendContinuousValue(500,0,0,0,0)
            WalkingFlag = True
            self.UpFlag = False
            ClimbFlag = False
        elif BodyState == 'SmallFront' :
            send.sendContinuousValue(300,0,0,0,0)
            WalkingFlag = True
            self.UpFlag = False
            ClimbFlag = False
        elif BodyState == 'BigLeftRotation' :
            send.sendContinuousValue(-100,0,0,5,0)
            WalkingFlag = True
            self.UpFlag = False
            ClimbFlag = False
        elif BodyState == 'BigRightRotation' :
            send.sendContinuousValue(-100,0,0,-5,0)
            WalkingFlag = True
            self.UpFlag = False
            ClimbFlag = False
        elif BodyState == 'LeftRotation' :
            send.sendContinuousValue(-100,0,0,4,0)
            WalkingFlag = True
            self.UpFlag = False
            ClimbFlag = False
        elif BodyState == 'RightRotation' :
            send.sendContinuousValue(-100,0,0,-4,0)
            WalkingFlag = True
            self.UpFlag = False
            ClimbFlag = False
        elif BodyState == 'UP' :
            send.sendBodyAuto(300,0,0,0,1,0)
            time.sleep(2.5)
            WalkingFlag = False
            self.UpFlag = True
            ClimbFlag = False
            send.sendBodyAuto(8500,0,0,0,2,0)
            time.sleep(5)
        elif BodyState == 'Down' :
            send.sendBodyAuto(300,0,0,0,1,0)
            time.sleep(2.5)
            WalkingFlag = False
            self.UpFlag = True
            ClimbFlag = False
            send.sendBodyAuto(8500,0,0,0,3,0)
            time.sleep(5)
        elif BodyState == 'Climb_First_Stair' :
            send.sendBodyAuto(300,0,0,0,1,0)
            time.sleep(1)
            ClimbFlag = True
            WalkingFlag = False
        return WalkingFlag , self.UpFlag
                