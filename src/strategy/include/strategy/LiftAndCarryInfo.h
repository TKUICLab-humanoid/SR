
#pragma once

#ifndef LiftAndCarryInfoH
#define LiftAndCarryInfoH

#include <math.h>
#include "tku_libs/strategy_info.h"

#define TopColor (int)LabelMark::YellowLabel
#define SecColor (int)LabelMark::BlueLabel
#define TrdColor (int)LabelMark::RedLabel
#define FileColor (int)LabelMark::GreenLabel
#define EndColor (int)LabelMark::OthersLabel
#define SampleNum 100
#define ParticleNum 48
#define PI 3.14159265358979323846
#define ImageWidth 320

enum CW_Strategy_STATE   //CW
{
	Go_to_stair, Climb_to_first_stair, Climb_to_second_stair,Climb_to_third_stair, Climb_next, On_stair_top
};
enum Strategy_STATE
{
     CalSlope,Finish
};

enum ACTION_STATE
{
    Spr, Up, Zero, SmallFront, SmallLeftRotation, SmallRightRotation, BigLeftRotation, BigRightRotation, RightShift, LeftShift,SmallLeftShift,SmallRightShift, BigFront, Find_Wood
};

enum ACTION_Hole
{
    Left, Right
};

enum Stair
{
    Stair_0, Stair_1, Stair_2, Stair_3, Stair_4, Stair_5, Stair_6
};

enum Strategy
{
    strategy_liftandcarry, strategy_climbingwall
};

struct Info
{
    int XMax, XMin, YMax, YMin, X, Y;
};

class LiftAndCarryInfo
{
    public:
        LiftAndCarryInfo(void);
        Strategy_STATE StrategyState;
        ACTION_STATE BodyState;
        ACTION_STATE Laststate;
        ACTION_Hole HoleState;
        CW_Strategy_STATE CW_Strategy;
        Info TopStair;
        Info SecStair;
        Info TrdStair;
        Info FileStair;
        Info RightFoot;
        Info LeftFoot;
        int Delaytime;
        int datum;
        int GreatFeetCount;
        int WhichStair;
        int WhichStrategy;//add
        LabelModel etTopColor;
        LabelModel etSecColor;
        LabelModel etTrdColor;
        LabelModel etFothColor;
        LabelModel etFileColor;
        LabelModel etEndColor;   
        int LeftForSlope[2];
        int RightForSlope[2];
        int CenterForSlope[2];
        int RightCenterForSlope[1];
        int LeftCenterForSlope[1];
        int Hole_Leftx0;
        int Hole_Leftx1;
        int Hole_Rightx0;
        int Hole_Rightx1;
        int Hole_Lefty0;
        int Hole_Lefty1;
        int Hole_Righty0;
        int Hole_Righty1;
        int HoleShift;
        int LeftFootDistance;
        int RightFootDistance;
        int CenterFootDistance;
        int RightCenterFootDistance;
        int LeftCenterFootDistance;
        int rightrotatcount;
        int leftrotatcount;
        bool IMUenable;
        bool Start;
        bool Hole_x0Flag;
        bool Hole_x1Flag;
        bool Hole_y0Flag;
        bool Hole_y1Flag;
        bool AviodHole;
        bool LeftSlopeFlag;
        bool RightSlopeFlag;
        bool CenterSlopeFlag;
        bool RobotUp;			//true上坡模式，false下坡模式
        bool InitialFlag;
};
extern LiftAndCarryInfo* liftandcarryinfo;
#endif