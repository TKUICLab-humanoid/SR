#include "strategy/LiftAndCarryInfo.h"

LiftAndCarryInfo* liftandcarryinfo = new LiftAndCarryInfo();
LiftAndCarryInfo::LiftAndCarryInfo(void)
{
    etTopColor = LabelModel::Yellow;
    etSecColor = LabelModel::Blue;
    etTrdColor = LabelModel::Red;
    etFileColor = LabelModel::Green;
    etEndColor = LabelModel::Other; 
    StrategyState = CalSlope;
    WhichStair = Stair_0;
    BodyState = Up;
    HoleState = Left;
//---------------For climbing wall----------------------------------------------------------------
	CW_Strategy = Go_to_stair;
//------------------------------------------------------------------------------------------------
    RobotUp = true;
    AviodHole = false;
    InitialFlag = true;
    RightFoot.XMin = 170 + 5;//165
    RightFoot.XMax = RightFoot.XMin + 50;//6.5
    RightFoot.YMin = 130;//83
    RightFoot.YMax = RightFoot.YMin + 80;//10
    LeftFoot.XMin = 131 - 25;
    LeftFoot.XMax = LeftFoot.XMin + 50;
    LeftFoot.YMin = 130;
    LeftFoot.YMax = LeftFoot.YMin + 80;
}
