#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "strategy/FootData.h"
#include "strategy/HoleData.h"
#include "strategy/mainData.h"
#include "LiftAndCarryInfo.h"
#include <iostream>
#include <string>

using namespace std;

class KidsizeStrategy
{
	public:
		ros::Subscriber HoleData_subscribe;
		ros::Subscriber FootData_subscribe;	
		ros::Publisher mainData_Publish;
		KidsizeStrategy(ros::NodeHandle &nh)
		{
			strategy_info = StrategyInfoInstance::getInstance();
			tool = ToolInstance::getInstance();
			ros_com = RosCommunicationInstance::getInstance();
			HoleData_subscribe = nh.subscribe("Hole_Topic", 1,&KidsizeStrategy::GetHoleData,this);
			FootData_subscribe = nh.subscribe("Distance_Topic", 1, &KidsizeStrategy::GetFootData,this);
			mainData_Publish = nh.advertise<strategy::mainData>("mainData_Topic", 10);
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
		void initparameterpath();
		void strategymain();
		void GetHoleData(const strategy::HoleData &msg);
		void GetFootData(const strategy::FootData &msg);
		void SendmainData(int WhichStair);
		void StrategyInitial();
		void StrategyBody();	
		void CW_distance();
		void CW_StrategyBody();
		void CW_StrategyClassify();
		void ShowMainData();
		void StrategyClassify();
		void Loadparameter();
		void Strategy_Select();
		void ShiftDanger();
		void Initial_IMUdata();
		void Theta_Offset();
		double timeuse,timeuse_stragegy;
		struct timeval tstart, tend;
		struct timeval tstart_stragegy,tend_stragegy;
		struct timeval tstart2, tend2;
		double initial_IMUdata,current_IMUdata;
		double IMUoffset;
		double leftslope,rightslope;
		bool shiftdanger;
		int Theta_offset;
		int countdistance;
		string parameter_path="N";
		//for LoadParameter
		int st0_SmallFrontX , st0_SmallFrontY , st0_SmallFrontZ , st0_SmallFrontTha, st0_SmallFrontimu;
		int st0_BigFrontX , st0_BigFrontY , st0_BigFrontZ , st0_BigFrontTha, st0_BigFrontimu;
		int st0_SprX , st0_SprY , st0_SprZ , st0_SprTha, st0_Sprimu;
		int SmallFrontX , SmallFrontY , SmallFrontZ , SmallFrontTha, SmallFrontimu, SmallFrontDelay;
		int BigFrontX , BigFrontY , BigFrontZ , BigFrontTha, BigFrontimu, BigFrontDelay;
		int SprX , SprY , SprZ , SprTha, Sprimu, SprDelay;
		int LeftShiftX , LeftShiftY , LeftShiftZ , LeftShiftTha, LeftShiftimu, LeftShiftDelay;
		int RightShiftX , RightShiftY , RightShiftZ , RightShiftTha, RightShiftimu, RightShiftDelay;
		int SmallLeftShiftX , SmallLeftShiftY , SmallLeftShiftZ , SmallLeftShiftTha, SmallLeftShiftimu, SmallLeftShiftDelay;
		int SmallRightShiftX , SmallRightShiftY , SmallRightShiftZ , SmallRightShiftTha, SmallRightShiftimu, SmallRightShiftDelay;
		int SmallLeftRotationX , SmallLeftRotationY , SmallLeftRotationZ , SmallLeftRotationTha, SmallLeftRotationimu, SmallLeftRotationDelay;
		int SmallRightRotationX , SmallRightRotationY , SmallRightRotationZ , SmallRightRotationTha, SmallRightRotationimu, SmallRightRotationDelay;
		int BigLeftRotationX , BigLeftRotationY , BigLeftRotationZ , BigLeftRotationTha, BigLeftRotationimu, BigLeftRotationDelay;
		int BigRightRotationX , BigRightRotationY , BigRightRotationZ , BigRightRotationTha, BigRightRotationimu, BigRightRotationDelay;
		int Hole_LeftX , Hole_LeftY , Hole_LeftZ , Hole_LeftTha, Hole_Leftimu;
		int Hole_RightX , Hole_RightY , Hole_RightZ , Hole_RightTha, Hole_Rightimu, Hole_Delay;
		int LC_StepX , LC_StepY , LC_StepZ , LC_StepTha, LC_Stepimu, LC_StepDelay;
		int LC_DownX , LC_DownY , LC_DownZ , LC_DownTha, LC_Downimu, LC_DownDelay;
		int HeadpositionX , HeadpositionY;
		int CW_top_stair;
		int CW_first_hand_delay , CW_first_foot_delay , CW_hand_delay , CW_foot_delay;
		int CW_Up_distance;
		int SureUpDistance,ErrorDownDistance;
		//for LoadParameter
		//CW
		bool CW_step_flag;
		bool CW_finish;
		bool CW_handcheck;
		bool CW_Leftfoot_flag, CW_Rightfoot_flag;
		int CW_Stair;
		int CW_Leftfoot_distance;
		int CW_Rightfoot_distance;
		int CW_Slope_Leftdistance;
		int CW_Slope_Rightdistance;
		int CW_Count;// = 0;
		float CW_Slope;
		int CW_stairdistance[20]={0};
		//CW
};
