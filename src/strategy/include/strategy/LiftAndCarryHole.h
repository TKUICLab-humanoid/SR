#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "strategy/FootData.h"
#include "strategy/HoleData.h"
#include "strategy/mainData.h"
#include "LiftAndCarryInfo.h"

using namespace std;
#define PI 3.14159265358979323846

ros::Publisher HoleData_Publish;
ros::Subscriber mainData_subscribe;
ros::Subscriber LabelModel_subscribe;

class KidsizeStrategy : public TKU_KidSizeStrategyBase
{
	public:
		KidsizeStrategy(ros::NodeHandle &nh) : TKU_KidSizeStrategyBase(nh)
		{
			HoleData_Publish = nh.advertise<strategy::HoleData>("Hole_Topic",1000);
			mainData_subscribe = nh.subscribe("mainData_Topic", 1000, &KidsizeStrategy::GetmainData,this);
		};
		~KidsizeStrategy(){};
		void strategymain();
		void Loadparameter();
		void GetmainData(const strategy::mainData &msg);
		void SendHoleData(bool Hole_x0Flag, bool Hole_x1Flag, bool Hole_y0Flag, bool Hole_y1Flag, bool AviodHole, int Hole_Leftx1, int Hole_Rightx1);
		void ShowHoleData();
		void HoleDataInitial();
		int SmallFrontX , SmallFrontY , SmallFrontZ , SmallFrontTha;
		int BigFrontX , BigFrontY , BigFrontZ , BigFrontTha;
		int SprX , SprY , SprZ , SprTha;
		int LeftShiftX , LeftShiftY , LeftShiftZ , LeftShiftTha;
		int RightShiftX , RightShiftY , RightShiftZ , RightShiftTha;
		int SmallLeftShiftX , SmallLeftShiftY , SmallLeftShiftZ , SmallLeftShiftTha;
		int SmallRightShiftX , SmallRightShiftY , SmallRightShiftZ , SmallRightShiftTha;
		int SmallLeftRotationX , SmallLeftRotationY , SmallLeftRotationZ , SmallLeftRotationTha;
		int SmallRightRotationX , SmallRightRotationY , SmallRightRotationZ , SmallRightRotationTha;
		int BigLeftRotationX , BigLeftRotationY , BigLeftRotationZ , BigLeftRotationTha;
		int BigRightRotationX , BigRightRotationY , BigRightRotationZ , BigRightRotationTha;
		int Hole_LeftX , Hole_LeftY , Hole_LeftZ , Hole_LeftTha;
		int Hole_RightX , Hole_RightY , Hole_RightZ , Hole_RightTha;
		int LC_StepX , LC_StepY , LC_StepZ , LC_StepTha;
		int LC_DownX , LC_DownY , LC_DownZ , LC_DownTha;
		int HeadpositionX , HeadpositionY;
		int CW_top_stair;
		int CW_first_hand_delay , CW_first_foot_delay , CW_hand_delay , CW_foot_delay;
		int CW_Up_distance;
};
