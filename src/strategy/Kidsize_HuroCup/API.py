#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from rospy import Publisher
from tku_msgs.msg import Interface,HeadPackage,SandHandSpeed,DrawImage,SingleMotorData,\
SensorSet,ObjectList,LabelModelObjectList,RobotPos,SetGoalPoint,SoccerDataList,SensorPackage
from std_msgs.msg import Int16,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class Sendmessage:     
    def __init__(self):
        self.walkingGait_pub = rospy.Publisher('SendBodyAuto_Topic', Interface, queue_size=100)
        self.head_motor_pub = rospy.Publisher("/package/HeadMotor",HeadPackage, queue_size=10)
        self.sector_pub = rospy.Publisher("/package/Sector",Int16, queue_size=100)
        self.hand_speed_pub = rospy.Publisher("/package/motorspeed",SandHandSpeed, queue_size=100)
        self.draw_image_pub = rospy.Publisher("/strategy/drawimage",DrawImage, queue_size=100)
        self.continuous_value_pub = rospy.Publisher("/ChangeContinuousValue_Topic",Interface, queue_size=100)
        self.single_motor_data_pub = rospy.Publisher("/package/SingleMotorData",SingleMotorData, queue_size=100)
        self.sensor_pub = rospy.Publisher("sensorset",SensorSet, queue_size=100)
        
        self.Web = False
        self.Label_Model = np.empty((320*240))#[0 for i in range(320*240)]
        self.bridge = CvBridge()
        self.color_mask_subject_cnts = [0 for i in range(8)]
        self.color_mask_subject_X = [[0]*320 for i in range(8)]
        self.color_mask_subject_Y = [[0]*320 for i in range(8)]
        self.color_mask_subject_XMin = [[0]*320 for i in range(8)]
        self.color_mask_subject_XMax = [[0]*320 for i in range(8)]
        self.color_mask_subject_YMax = [[0]*320 for i in range(8)]
        self.color_mask_subject_YMin = [[0]*320 for i in range(8)]
        self.color_mask_subject_Width = [[0]*320 for i in range(8)]
        self.color_mask_subject_Height = [[0]*320 for i in range(8)]
        self.color_mask_subject_size = [[0]*320 for i in range(8)]
        self.imu_value_Roll = 0
        self.imu_value_Yaw = 0
        self.imu_value_Pitch = 0
        self.DIOValue = 0x00
        self.is_start = False

        aaaa = rospy.init_node('talker', anonymous=True)
        object_list_sub = rospy.Subscriber("/Object/List",ObjectList, self.getObject)
        label_model_sub = rospy.Subscriber("/LabelModel/List",LabelModelObjectList, self.getLabelModel)
        
        start_sub = rospy.Subscriber("/web/start",Bool, self.startFunction)
        DIO_ack_sub = rospy.Subscriber("/package/FPGAack",Int16, self.DIOackFunction)
        sensor_sub = rospy.Subscriber("/package/sensorpackage",SensorPackage, self.sensorPackageFunction)

        
    def sendBodyAuto(self,x,y,z,theta,mode,sensor):	#步態啟動   (0->單步 1->連續 2->上板  依walking interface排序)
        walkdata = Interface()
        walkdata.x = x
        walkdata.y = y
        walkdata.z = z
        walkdata.theta = theta
        walkdata.walking_mode = mode 
        walkdata.sensor_mode = sensor
        self.walkingGait_pub.publish(walkdata)

    def sendHeadMotor(self,ID,Position,Speed):	#頭部馬達   (Position 1->水平 2->垂直)
        HeadData = HeadPackage()
        HeadData.ID = ID
        HeadData.Position = Position
        HeadData.Speed = Speed
        self.head_motor_pub.publish(HeadData)

    def sendBodySector(self,Sector):	#磁區
        SectorData = Int16()
        SectorData.data = int(Sector)
        self.sector_pub.publish(SectorData)

    def sendHandSpeed(self,Sector,Speed):   #馬達速度放進磁區for投籃
        HandSpeedData = SandHandSpeed()
        HandSpeedData.sector = Sector
        HandSpeedData.speed = Speed
        self.hand_speed_pub.publish(HandSpeedData)

    def drawImageFunction(self,cnt,mode,xmin,xmax,ymin,ymax,r,g,b):
        ImageData = DrawImage()
        ImageData.cnt = cnt
        ImageData.XMax = xmax
        ImageData.XMin = xmin
        ImageData.YMax = ymax
        ImageData.YMin = ymin
        ImageData.rValue = r
        ImageData.gValue = g
        ImageData.bValue = b
        ImageData.Mode = mode
        self.draw_image_pub.publish(ImageData)

    def sendContinuousValue(self,x,y,z,theta,sensor):
        walkdata = Interface()
        walkdata.x = x
        walkdata.y = y
        walkdata.z = z
        walkdata.theta = theta
        walkdata.sensor_mode = sensor
        self.continuous_value_pub.publish(walkdata)

    def sendSingleMotor(self,ID,Position,Speed):
        MotorData = SingleMotorData()
        MotorData.ID = ID
        MotorData.Position = Position
        MotorData.Speed = Speed
        self.single_motor_data_pub.publish(MotorData)

    def sendSensorSet(self,R,P,Y,DesireSet,IMUReset,ForceState,GainSet):
        msg = SensorSet()
        msg.Roll  = R
        msg.Pitch = P
        msg.Yaw   = Y
        msg.DesireSet = DesireSet
        msg.IMUReset = IMUReset
        msg.ForceState = ForceState
        msg.GainSet = GainSet
        self.sensor_pub.publish(msg)

    def sendSensorReset(self):
        msg = SensorSet()
        msg.Roll  = 0
        msg.Pitch = 0
        msg.Yaw   = 0
        msg.DesireSet = False
        msg.IMUReset = True
        msg.ForceState = False
        msg.GainSet = False
        self.sensor_pub.publish(msg)

    def strategy(self):
        send = Sendmessage()
        while not rospy.is_shutdown():
            send.drawImageFunction(1,0,0,320,120,120,152,245,255)
            print(self.Web)
            time.sleep(0.3)
            send.sendBodySector(1010)
            time.sleep(4)
            time.sleep(0.3)
            send.sendBodySector(1011)
            time.sleep(4)
            time.sleep(0.3)
            send.sendBodySector(1020)
            time.sleep(4)
            time.sleep(0.3)
            send.sendBodySector(1021)
            time.sleep(4)
            print("1")
            
            

    def startFunction(self,msg):
        self.Web = msg.data
    def getLabelModel(self,msg):
        # start = time.time()
        # for i in range (320*240):
        #     self.Label_Model[i] = msg.LabelModel[i]
        # end = time.time()
        # print("Time Used = ",end - start)
        #print('MSG = ',msg.LabelModel)
        self.Label_Model = msg.LabelModel
        #print('SELF = ',self.Label_Model)
    def getObject(self,msg):
        for i in range (8):
            self.color_mask_subject_cnts[i] = msg.Objectlist[i].cnt
            for j in range (self.color_mask_subject_cnts[i]):

                self.color_mask_subject_X[i][j] = msg.Objectlist[i].Colorarray[j].X
                self.color_mask_subject_Y[i][j] = msg.Objectlist[i].Colorarray[j].Y
                self.color_mask_subject_XMin[i][j] = msg.Objectlist[i].Colorarray[j].XMin
                self.color_mask_subject_YMin[i][j] = msg.Objectlist[i].Colorarray[j].YMin
                self.color_mask_subject_XMax[i][j] = msg.Objectlist[i].Colorarray[j].XMax
                self.color_mask_subject_YMax[i][j] = msg.Objectlist[i].Colorarray[j].YMax
                self.color_mask_subject_Width[i][j] = msg.Objectlist[i].Colorarray[j].Width
                self.color_mask_subject_Height[i][j] = msg.Objectlist[i].Colorarray[j].Height
                self.color_mask_subject_size[i][j] = msg.Objectlist[i].Colorarray[j].size
    def sensorPackageFunction(self,msg):        
        self.imu_value_Roll  = msg.IMUData[0]
        self.imu_value_Pitch = msg.IMUData[1]
        self.imu_value_Yaw   = msg.IMUData[2]
    def DIOackFunction(self,msg):
        if msg.data & 0x10:
            self.is_start = True
        else:
            self.is_start = False
        self.DIOValue = msg.data
    
        
    
if __name__ == '__main__':
    try:
        aa = Sendmessage()
        aa.strategy()
    except rospy.ROSInterruptException:
        pass
