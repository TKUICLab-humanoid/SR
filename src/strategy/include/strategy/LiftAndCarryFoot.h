#include <stdio.h>
#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "tku_msgs/LabelModelObjectList.h"
#include "tku_msgs/DrawImage.h"
#include "strategy/HoleData.h"
#include "strategy/mainData.h"
#include "strategy/FootData.h"
#include "strategy/LiftAndCarryInfo.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"

#define noise_pixel_distancce 10
#define noise_pixel_virtual_foot 5

StrategyInfoInstance *strategy_info;
ToolInstance *tool;
RosCommunicationInstance *ros_com;
ros::Publisher FootData_Publish;
ros::Subscriber mainData_subscribe;
ros::Subscriber LabelModel_subscribe;
ros::Publisher DrawImage_Publish;
int countdistance[50];
static int ccount = 0;