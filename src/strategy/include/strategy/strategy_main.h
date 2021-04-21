#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "fMatrix.h"
//#include "strategy/FootData.h"
//#include "strategy/HoleData.h"
//#include "strategy/mainData.h"
//#include "LiftAndCarryInfo.h"
#include <iostream>
#include <string>
#include "tku_msgs/PointData.h"

using namespace std;

#define Scale2Deg 0.087890625
#define Deg2Rad 0.017453292 // pi/180

class KidsizeStrategy
{
	public:

		KidsizeStrategy(ros::NodeHandle &nh)
		{
			strategy_info = StrategyInfoInstance::getInstance();
			tool = ToolInstance::getInstance();
			ros_com = RosCommunicationInstance::getInstance();
			Endpoint_Publish = nh.advertise<tku_msgs::PointData>("/package/EndPoint", 1000);

		};
		~KidsizeStrategy()
		{
			StrategyInfoInstance::deleteInstance();
			ToolInstance::deleteInstance();
			RosCommunicationInstance::deleteInstance();		
		};
		StrategyInfoInstance *strategy_info;
		ToolInstance *tool;
		RosCommunicationInstance *ros_com;

		ros::Publisher Endpoint_Publish;

		void strategymain();
		void systemtransform(int x, int y, double headangle,double distance);
		void MoveHead(HeadMotorID ID, int Position, int Speed);

		int HorizontalHeadPosition;
		int VerticalHeadPosition;

		bool first_flag = true;


		tku_msgs::PointData endpoint_temp;
};
