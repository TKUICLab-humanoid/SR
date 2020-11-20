#include "strategy/LiftAndCarryHole.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kidsize123");
	ros::NodeHandle nh;
	KidsizeStrategy KidsizeStrategy(nh);

	ros::Rate loop_rate(30);

	while (nh.ok()) 
	{
		KidsizeStrategy.strategymain();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void KidsizeStrategy::strategymain()
{
    tool->Delay(1000);
    HoleDataInitial();
    if((liftandcarryinfo->WhichStair == Stair_1) || (liftandcarryinfo->WhichStair == Stair_5))
    {
        for(int w = 160; w >= 0; w--)
        {
            if(strategyinfo->Label_Model[ ImageWidth * (liftandcarryinfo->LeftFoot.YMin) + w] == FileColor)
            {
                if (liftandcarryinfo->Hole_Leftx1 == 0)
                {
                    liftandcarryinfo->Hole_Leftx0 = w;
                    liftandcarryinfo->Hole_Leftx1 = w;
                }
                else
                    liftandcarryinfo->Hole_Leftx1 = w;
                if (liftandcarryinfo->Hole_Leftx0 >= (liftandcarryinfo->LeftFoot.XMin -5))
                    liftandcarryinfo->Hole_x0Flag = true;
            }
        }
        for(int w =160; w < ImageWidth ; w++)
        {
            if( strategyinfo->Label_Model[ ImageWidth * (liftandcarryinfo->LeftFoot.YMin)+w] == FileColor)
            {
                if (liftandcarryinfo->Hole_Rightx1 == 320)
                {
                    liftandcarryinfo->Hole_Rightx0 = w;
                    liftandcarryinfo->Hole_Rightx1 = w;
                }
                else
                    liftandcarryinfo->Hole_Rightx1 = w;
                if (liftandcarryinfo->Hole_Rightx0 <= (liftandcarryinfo->RightFoot.XMax + 5))
                    liftandcarryinfo->Hole_x1Flag = true;
            }
        }
        if((liftandcarryinfo->Hole_x0Flag || liftandcarryinfo->Hole_x1Flag) && liftandcarryinfo->WhichStair == Stair_1)
            liftandcarryinfo->AviodHole = true;
        for(int w = 160; w >= 0 ; w--)
        {
            if(liftandcarryinfo->AviodHole)
            {
                break;
            }
            ROS_INFO("middle line");
            if( strategyinfo->Label_Model[ ImageWidth * (liftandcarryinfo->LeftFoot.YMin+30) + w] == FileColor)
            {
                if (liftandcarryinfo->Hole_Leftx1 == 0)
                {
                    liftandcarryinfo->Hole_Leftx0 = w;
                    liftandcarryinfo->Hole_Leftx1 = w;
                }
                else
                    liftandcarryinfo->Hole_Leftx1 = w;
                if (liftandcarryinfo->Hole_Leftx0 >= (liftandcarryinfo->LeftFoot.XMin -5))
                    liftandcarryinfo->Hole_x0Flag = true;
            }
        }
        for(int w = 160; w < ImageWidth ; w++)
        {
            if(liftandcarryinfo->AviodHole)
            {
                break;
            }
            if( strategyinfo->Label_Model[ ImageWidth * (liftandcarryinfo->LeftFoot.YMin+30) + w] == FileColor)
            {
                if (liftandcarryinfo->Hole_Rightx1 == 320)
                {
                    liftandcarryinfo->Hole_Rightx0 = w;
                    liftandcarryinfo->Hole_Rightx1 = w;
                }
                else
                    liftandcarryinfo->Hole_Rightx1 = w;
                if (liftandcarryinfo->Hole_Rightx0 <= (liftandcarryinfo->RightFoot.XMax + 5))
                    liftandcarryinfo->Hole_x1Flag = true;
            }
        }
        if((liftandcarryinfo->Hole_x0Flag || liftandcarryinfo->Hole_x1Flag) && liftandcarryinfo->WhichStair == Stair_1)
            liftandcarryinfo->AviodHole = true;
        for(int w = 160; w >= 0; w--)
        {
            if(liftandcarryinfo->AviodHole)
            {
                break;
            }
            ROS_INFO("bottom line");
            if( strategyinfo->Label_Model[ ImageWidth * (liftandcarryinfo->LeftFoot.YMax) + w] == FileColor)
            {
                if (liftandcarryinfo->Hole_Leftx1 == 0)
                {
                    liftandcarryinfo->Hole_Leftx0 = w;
                    liftandcarryinfo->Hole_Leftx1 = w;
                }
                else
                    liftandcarryinfo->Hole_Leftx1 = w;
                if (liftandcarryinfo->Hole_Leftx0 >= (liftandcarryinfo->LeftFoot.XMin -5))
                    liftandcarryinfo->Hole_x0Flag = true;
            }
        }
        for(int w = 160; w < ImageWidth; w++)
        {
            if(liftandcarryinfo->AviodHole)
            {
                break;
            }
            if( strategyinfo->Label_Model[ ImageWidth * (liftandcarryinfo->LeftFoot.YMax) + w] == FileColor)
            {
                if (liftandcarryinfo->Hole_Rightx1 == 320)
                {
                    liftandcarryinfo->Hole_Rightx0 = w;
                    liftandcarryinfo->Hole_Rightx1 = w;
                }
                else
                    liftandcarryinfo->Hole_Rightx1 = w;
                if (liftandcarryinfo->Hole_Rightx0 <= (liftandcarryinfo->RightFoot.XMax + 5))
                    liftandcarryinfo->Hole_x1Flag = true;
            }
            if((liftandcarryinfo->Hole_x0Flag || liftandcarryinfo->Hole_x1Flag) && liftandcarryinfo->WhichStair == Stair_1)
                liftandcarryinfo->AviodHole = true;
        }
        if(liftandcarryinfo->WhichStair == Stair_5)
        {
            if(liftandcarryinfo->Hole_x0Flag)
            {
                for(int w = (liftandcarryinfo->LeftFoot.YMin); w >= 0; w--)
                {
                    if( strategyinfo->Label_Model[ ImageWidth * w + liftandcarryinfo->LeftFoot.XMin] == TrdColor)
                        liftandcarryinfo->Hole_y0Flag = true;
                }
                if(!liftandcarryinfo->Hole_y0Flag)
                {
                    for(int w = 160; w >= 0; w--)
                    {
                        if( strategyinfo->Label_Model[w] == TrdColor)
                        {
                            if(w > 11)
                                liftandcarryinfo->Hole_y0Flag = true;
                        }
                    }
                }
            }
            if(liftandcarryinfo->Hole_x1Flag)
            {
                for(int w = (liftandcarryinfo->LeftFoot.YMin); w >= 0 ;w--)
                {
                    if( strategyinfo->Label_Model[ ImageWidth * w + liftandcarryinfo->RightFoot.XMax] == TrdColor)
                        liftandcarryinfo->Hole_y1Flag = true;
                }
                if(!liftandcarryinfo->Hole_y1Flag)
                {
                    for(int w = 160; w <= 320; w++)
                    {
                        if( strategyinfo->Label_Model[w] == TrdColor)
                        {
                            if(w < 311)
                                liftandcarryinfo->Hole_y1Flag = true;
                        }
                    }
                }
            }
            if (liftandcarryinfo->Hole_x0Flag || liftandcarryinfo->Hole_x1Flag)
                liftandcarryinfo->AviodHole = true;
        }
    }
    SendHoleData(liftandcarryinfo->Hole_x0Flag, liftandcarryinfo->Hole_x1Flag, liftandcarryinfo->Hole_y0Flag, liftandcarryinfo->Hole_y1Flag, liftandcarryinfo->AviodHole, liftandcarryinfo->Hole_Leftx1, liftandcarryinfo->Hole_Rightx1);
    ShowHoleData();
}

void KidsizeStrategy::GetmainData(const strategy::mainData &msg)
{
    liftandcarryinfo->WhichStair = msg.WhichStair;
}

void KidsizeStrategy::SendHoleData(bool Hole_x0Flag, bool Hole_x1Flag, bool Hole_y0Flag, bool Hole_y1Flag, bool AviodHole, int Hole_Leftx1, int Hole_Rightx1)
{
    strategy::HoleData msg;
    msg.Hole_x0Flag = Hole_x0Flag;
    msg.Hole_x1Flag = Hole_x1Flag;
    msg.Hole_y0Flag = Hole_y0Flag;
    msg.Hole_y1Flag = Hole_y1Flag;
    msg.AviodHole = AviodHole;
    msg.Hole_Leftx1 = Hole_Leftx1;
    msg.Hole_Rightx1 = Hole_Rightx1;
    HoleData_Publish.publish(msg);
}

void KidsizeStrategy::ShowHoleData()
{
    ROS_INFO("---------------------------");
    ROS_INFO("Hole_x1 = %d , %d , %d , %d",liftandcarryinfo->Hole_Leftx1,liftandcarryinfo->Hole_Leftx0,liftandcarryinfo->Hole_Rightx0,liftandcarryinfo->Hole_Rightx1);
    ROS_INFO("AvoidHole = %s",(liftandcarryinfo->AviodHole == true ? "true" : "false"));
    ROS_INFO("%x\n", strategyinfo->Label_Model[ImageWidth * (liftandcarryinfo->LeftFoot.YMin) + liftandcarryinfo->Hole_Leftx1]);
}

void KidsizeStrategy::HoleDataInitial()
{
    liftandcarryinfo->Hole_Leftx0 = 160;	//螢幕中間
    liftandcarryinfo->Hole_Leftx1 = 0;
    liftandcarryinfo->Hole_Rightx0 = 160;	//螢幕中間
    liftandcarryinfo->Hole_Rightx1 = 320;
    liftandcarryinfo->Hole_x0Flag = false;
    liftandcarryinfo->Hole_x1Flag = false;
    liftandcarryinfo->Hole_y0Flag = false;
    liftandcarryinfo->Hole_y1Flag = false;
    liftandcarryinfo->AviodHole = false;
}

void KidsizeStrategy::Loadparameter()
{
    fstream fin;
    char path[200];
    strcpy(path, PATH.c_str());
    strcat(path, "/SpartanRace_Parameter.ini");
    string sTmp;
    char line[100];
    fin.open(path, ios::in);
    try
    {
        SmallFrontX = tool->readvalue(fin, "SmallFrontX", 1);
        SmallFrontY = tool->readvalue(fin, "SmallFrontY", 1);
        SmallFrontZ = tool->readvalue(fin, "SmallFrontZ", 1);
        SmallFrontTha = tool->readvalue(fin, "SmallFrontTha", 1);
        BigFrontX = tool->readvalue(fin, "BigFrontX", 1);
        BigFrontY = tool->readvalue(fin, "BigFrontY", 1);
        BigFrontZ = tool->readvalue(fin, "BigFrontZ", 1);
        BigFrontTha = tool->readvalue(fin, "BigFrontTha", 1);
        SprX = tool->readvalue(fin, "SprX", 1);
        SprY = tool->readvalue(fin, "SprY", 1);
        SprZ = tool->readvalue(fin, "SprZ", 1);
        SprTha = tool->readvalue(fin, "SprTha", 1);
        LeftShiftX = tool->readvalue(fin, "LeftShiftX", 1);
        LeftShiftY = tool->readvalue(fin, "LeftShiftY", 1);
        LeftShiftZ = tool->readvalue(fin, "LeftShiftZ", 1);
        LeftShiftTha = tool->readvalue(fin, "LeftShiftTha", 1);
        RightShiftX = tool->readvalue(fin, "RightShiftX", 1);
        RightShiftY = tool->readvalue(fin, "RightShiftY", 1);
        RightShiftZ = tool->readvalue(fin, "RightShiftZ", 1);
        RightShiftTha = tool->readvalue(fin, "RightShiftTha", 1);
        SmallLeftShiftX = tool->readvalue(fin, "SmallLeftShiftX", 1);
        SmallLeftShiftY = tool->readvalue(fin, "SmallLeftShiftY", 1);
        SmallLeftShiftZ = tool->readvalue(fin, "SmallLeftShiftZ", 1);
        SmallLeftShiftTha = tool->readvalue(fin, "SmallLeftShiftTha", 1);
        SmallRightShiftX = tool->readvalue(fin, "SmallRightShiftX", 1);
        SmallRightShiftY = tool->readvalue(fin, "SmallRightShiftY", 1);
        SmallRightShiftZ = tool->readvalue(fin, "SmallRightShiftZ", 1);
        SmallRightShiftTha = tool->readvalue(fin, "SmallRightShiftTha", 1);
        SmallLeftRotationX = tool->readvalue(fin, "SmallLeftRotationX", 1);
        SmallLeftRotationY = tool->readvalue(fin, "SmallLeftRotationY", 1);
        SmallLeftRotationZ = tool->readvalue(fin, "SmallLeftRotationZ", 1);
        SmallLeftRotationTha = tool->readvalue(fin, "SmallLeftRotationTha", 1);
        SmallRightRotationX = tool->readvalue(fin, "SmallRightRotationX", 1);
        SmallRightRotationY = tool->readvalue(fin, "SmallRightRotationY", 1);
        SmallRightRotationZ = tool->readvalue(fin, "SmallRightRotationZ", 1);
        SmallRightRotationTha = tool->readvalue(fin, "SmallRightRotationTha", 1);
        BigLeftRotationX = tool->readvalue(fin, "BigLeftRotationX", 1);
        BigLeftRotationY = tool->readvalue(fin, "BigLeftRotationY", 1);
        BigLeftRotationZ = tool->readvalue(fin, "BigLeftRotationZ", 1);
        BigLeftRotationTha = tool->readvalue(fin, "BigLeftRotationTha", 1);
        BigRightRotationX = tool->readvalue(fin, "BigRightRotationX", 1);
        BigRightRotationY = tool->readvalue(fin, "BigRightRotationY", 1);
        BigRightRotationZ = tool->readvalue(fin, "BigRightRotationZ", 1);
        BigRightRotationTha = tool->readvalue(fin, "BigRightRotationTha", 1);
        Hole_LeftX = tool->readvalue(fin, "Hole_LeftX", 1);
        Hole_LeftY = tool->readvalue(fin, "Hole_LeftY", 1);
        Hole_LeftZ = tool->readvalue(fin, "Hole_LeftZ", 1);
        Hole_LeftTha = tool->readvalue(fin, "Hole_LeftTha", 1);
        Hole_RightX = tool->readvalue(fin, "Hole_RightX", 1);
        Hole_RightY = tool->readvalue(fin, "Hole_RightY", 1);
        Hole_RightZ = tool->readvalue(fin, "Hole_RightZ", 1);
        Hole_RightTha = tool->readvalue(fin, "Hole_RightTha", 1);
        LC_StepX = tool->readvalue(fin, "LC_StepX", 1);
        LC_StepY = tool->readvalue(fin, "LC_StepY", 1);
        LC_StepZ = tool->readvalue(fin, "LC_StepZ", 1);
        LC_StepTha = tool->readvalue(fin, "LC_StepTha", 1);
        LC_DownX = tool->readvalue(fin, "LC_DownX", 1);
        LC_DownY = tool->readvalue(fin, "LC_DownY", 1);
        LC_DownZ = tool->readvalue(fin, "LC_DownZ", 1);
        LC_DownTha = tool->readvalue(fin, "LC_DownTha", 1);
        HeadpositionX = tool->readvalue(fin, "HeadpositionX", 1);
        HeadpositionY = tool->readvalue(fin, "HeadpositionY", 1);
        CW_top_stair = tool->readvalue(fin, "CW_top_stair", 1);
        CW_first_hand_delay = tool->readvalue(fin, "CW_first_hand_delay", 1);
        CW_first_foot_delay = tool->readvalue(fin, "CW_first_foot_delay", 1);
    	CW_hand_delay = tool->readvalue(fin, "CW_hand_delay", 1);
    	CW_foot_delay = tool->readvalue(fin, "CW_foot_delay", 1);
        CW_Up_distance = tool->readvalue(fin, "CW_Up_distance", 1);
        fin.close();
    }
    catch(exception e)
    {
    }
}
