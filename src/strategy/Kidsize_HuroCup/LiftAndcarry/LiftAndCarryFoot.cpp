#include "strategy/LiftAndCarryFoot.h"

void GetmainData(const strategy::mainData &msg)
{
    liftandcarryinfo->WhichStair = msg.WhichStair;
}

void GetLabelModel(const tku_msgs::LabelModelObjectList &msg)
{
    for(int i = 0; i < 320*240; i++)
        strategy_info->label_model[i] = msg.LabelModel[i];
}

void SendFootData(bool LeftSlopeFlag, bool CenterSlopeFlag, bool RightSlopeFlag, int LeftFootDistance, int CenterFootDistance, int RightFootDistance, int LeftCenterFootDistance, int RightCenterFootDistance,bool RightFindWoodFlag,bool LeftFindWoodFlag)
{
    strategy::FootData msg;
    msg.LeftSlopeFlag = LeftSlopeFlag;
    msg.CenterSlopeFlag = CenterSlopeFlag;
    msg.RightSlopeFlag = RightSlopeFlag;
    msg.LeftFootDistance = LeftFootDistance;
    msg.CenterFootDistance = CenterFootDistance;
    msg.RightFootDistance = RightFootDistance;
    msg.LeftCenterFootDistance = LeftCenterFootDistance;
    msg.RightCenterFootDistance = RightCenterFootDistance;
    msg.RightFindWoodFlag = RightFindWoodFlag;//4.21
    msg.LeftFindWoodFlag = LeftFindWoodFlag;//4.21
    FootData_Publish.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LiftAndCarryFoot");
	ros::NodeHandle nh;
	strategy_info = StrategyInfoInstance::getInstance();
	tool = ToolInstance::getInstance();
	ros_com = RosCommunicationInstance::getInstance();
    FootData_Publish = nh.advertise<strategy::FootData>("Distance_Topic", 10);
    mainData_subscribe = nh.subscribe("mainData_Topic", 10, GetmainData);
    LabelModel_subscribe = nh.subscribe("/LabelModel/List", 10, GetLabelModel);
    DrawImage_Publish = nh.advertise<tku_msgs::DrawImage>("/strategy/drawimage", 100);
    ros::Rate loop_rate(30);
    while(nh.ok())
    {
        ros::spinOnce();
        liftandcarryinfo->LeftFindWoodFlag = false;
        liftandcarryinfo->RightFindWoodFlag = false;
        liftandcarryinfo->LeftSlopeFlag = false;
        liftandcarryinfo->RightSlopeFlag = false;
        liftandcarryinfo->CenterSlopeFlag = false;
        if(liftandcarryinfo->WhichStair == Stair_1 || liftandcarryinfo->WhichStair == Stair_3)
        {
            for( int h = liftandcarryinfo->LeftFoot.YMax; h > liftandcarryinfo->LeftFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == SecColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->LeftSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->RightFoot.XMax] == SecColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->RightSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + ((liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2)] == SecColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->CenterForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->CenterSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->LeftFoot.XMin+liftandcarryinfo->LeftFoot.XMax)/2] == SecColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->RightFoot.XMax+(liftandcarryinfo->LeftFoot.XMin+liftandcarryinfo->RightFoot.XMax)/2)/2] == SecColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
        }
        else if((liftandcarryinfo->WhichStair == Stair_2))
        {
            for( int h = liftandcarryinfo->LeftFoot.YMax; h > liftandcarryinfo->LeftFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == TopColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->LeftSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->RightFoot.XMax ] == TopColor)
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->RightSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for	( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + ((liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2)] == TopColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->CenterForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->CenterSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->LeftFoot.XMin+liftandcarryinfo->LeftFoot.XMax)/2] == TopColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->RightFoot.XMin+liftandcarryinfo->RightFoot.XMax)/2] == TopColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
        }
        else if(liftandcarryinfo->WhichStair == Stair_0 || liftandcarryinfo->WhichStair == Stair_4)
        {
            for( int h = liftandcarryinfo->LeftFoot.YMax; h > liftandcarryinfo->LeftFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == TrdColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->LeftSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->RightFoot.XMax] == TrdColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->RightSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + ((liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2)] == TrdColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->CenterForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->CenterSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->LeftFoot.XMin+liftandcarryinfo->LeftFoot.XMax)/2] == TrdColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->RightFoot.XMin+liftandcarryinfo->RightFoot.XMax)/2] == TrdColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
        }
        else if(liftandcarryinfo->WhichStair == Stair_5)
        {
            for( int h = liftandcarryinfo->LeftFoot.YMax; h > liftandcarryinfo->LeftFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == FileColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->LeftSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->RightFoot.XMax ] == FileColor)
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->RightSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + ((liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2)] == FileColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->CenterForSlope[0] = h + (ccount - 1);
                            liftandcarryinfo->CenterSlopeFlag = true;
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->LeftFoot.XMin+liftandcarryinfo->LeftFoot.XMax)/2] == FileColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->LeftCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
            for( int h = liftandcarryinfo->RightFoot.YMax; h > liftandcarryinfo->RightFoot.YMin-noise_pixel_virtual_foot ; h-- )
            {
                if( strategy_info->label_model[ ImageWidth * h + (liftandcarryinfo->RightFoot.XMin+liftandcarryinfo->RightFoot.XMax)/2] == FileColor )
                {
                    countdistance[ccount] = h;
                    if (countdistance[ccount] != countdistance[ccount-1]-1 && ccount != 0)
                    {
                        for(int i = 0; i <= ccount; i++)
                            countdistance[i] = 0;
                        ccount = 0;
                    }
                    else
                    {
                        ccount++;
                        if(ccount == noise_pixel_virtual_foot)
                        {
                            liftandcarryinfo->RightCenterForSlope[0] = h + (ccount - 1);
                            for(int i = 0; i < ccount; i++)
                                countdistance[i] = 0;
                            ccount = 0;
                            break;
                        }
                    }
                }
            }
        }
        int i;
        for(i=liftandcarryinfo->LeftFoot.YMin;i>0;i--)//4.21 計算是否看到板子
		{
            if(liftandcarryinfo->WhichStair == Stair_0)
			{
				if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == TrdColor )
				{
                   // NextStairDistanceL=liftandcarryinfo->LeftFoot.YMin-i;
                    liftandcarryinfo->LeftFindWoodFlag = true;
					break;
				}
			}
			if(liftandcarryinfo->WhichStair == Stair_1)
			{
				if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == SecColor )
				{
                    //NextStairDistanceL=liftandcarryinfo->LeftFoot.YMin-i;
                    liftandcarryinfo->LeftFindWoodFlag = true;
					break;
				}
			}
			else if(liftandcarryinfo->WhichStair == Stair_2)
			{
				if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == TopColor )
				{
                    //NextStairDistanceL=liftandcarryinfo->LeftFoot.YMin-i;
                    liftandcarryinfo->LeftFindWoodFlag = true;
					break;
				}
			}
		}
        for(i=liftandcarryinfo->LeftFoot.YMin;i>0;i--)//4.21
		{
            if(liftandcarryinfo->WhichStair == Stair_0)
			{
				if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax] == TrdColor )
				{
                    //NextStairDistanceR=liftandcarryinfo->LeftFoot.YMin-i;
                    liftandcarryinfo->RightFindWoodFlag = true;
					break;
				}
			}
			if(liftandcarryinfo->WhichStair == Stair_1)
			{
				if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax] == SecColor )
				{
                    //NextStairDistanceR=liftandcarryinfo->LeftFoot.YMin-i;
                    liftandcarryinfo->RightFindWoodFlag = true;
					break;
				}
			}
			else if(liftandcarryinfo->WhichStair == Stair_2)
			{
				if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax] == TopColor )
				{
                    //NextStairDistanceR=liftandcarryinfo->LeftFoot.YMin-i;
                    liftandcarryinfo->RightFindWoodFlag = true;
					break;
				}
			}
		}
        liftandcarryinfo->LeftFootDistance = (liftandcarryinfo->LeftFoot.YMax) - liftandcarryinfo->LeftForSlope[0];
        liftandcarryinfo->RightFootDistance = (liftandcarryinfo->RightFoot.YMax) -liftandcarryinfo->RightForSlope[0];
        liftandcarryinfo->CenterFootDistance = (liftandcarryinfo->RightFoot.YMax) -liftandcarryinfo->CenterForSlope[0];
        liftandcarryinfo->LeftCenterFootDistance = liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftCenterForSlope[0];
        liftandcarryinfo->RightCenterFootDistance = liftandcarryinfo->RightFoot.YMax - liftandcarryinfo->RightCenterForSlope[0];
        liftandcarryinfo->LeftForSlope[0] = -9999 + liftandcarryinfo->LeftFoot.YMax;//!!!
        liftandcarryinfo->RightForSlope[0] = -9999 + liftandcarryinfo->LeftFoot.YMax;//初始
        liftandcarryinfo->CenterForSlope[0] = -9999 + liftandcarryinfo->LeftFoot.YMax;
        liftandcarryinfo->LeftCenterForSlope[0] = -9999 + liftandcarryinfo->LeftFoot.YMax;
        liftandcarryinfo->RightCenterForSlope[0] = -9999 + liftandcarryinfo->LeftFoot.YMax;
        SendFootData(liftandcarryinfo->LeftSlopeFlag, liftandcarryinfo->CenterSlopeFlag, liftandcarryinfo->RightSlopeFlag, liftandcarryinfo->LeftFootDistance, liftandcarryinfo->CenterFootDistance, liftandcarryinfo->RightFootDistance, liftandcarryinfo->LeftCenterFootDistance, liftandcarryinfo->RightCenterFootDistance,liftandcarryinfo->RightFindWoodFlag,liftandcarryinfo->LeftFindWoodFlag);
        ROS_INFO("FootWitchstar = %d",liftandcarryinfo->WhichStair);
        ROS_INFO("LeftFindWoodFlag = %s, RightFindWoodFlag = %s",(liftandcarryinfo->LeftFindWoodFlag == true ? "true" : "false"),(liftandcarryinfo->RightFindWoodFlag == true ? "true" : "false"));
        ROS_INFO("FootDistance = %d %d %d %d %d\n",liftandcarryinfo->LeftFootDistance,liftandcarryinfo->LeftCenterFootDistance,liftandcarryinfo->CenterFootDistance,liftandcarryinfo->RightCenterFootDistance,liftandcarryinfo->RightFootDistance);
        ros_com->drawImageFunction(5,DrawMode::DrawObject,liftandcarryinfo->LeftFoot.XMin,liftandcarryinfo->LeftFoot.XMax,liftandcarryinfo->LeftFoot.YMin,liftandcarryinfo->LeftFoot.YMax,0,255,0);
        ros_com->drawImageFunction(6,DrawMode::DrawObject,liftandcarryinfo->RightFoot.XMin,liftandcarryinfo->RightFoot.XMax,liftandcarryinfo->RightFoot.YMin,liftandcarryinfo->RightFoot.YMax,0,255,0);
        ros_com->drawImageFunction(7,DrawMode::DrawObject, liftandcarryinfo->LeftFoot.XMin - 1, liftandcarryinfo->LeftFoot.XMin + 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftFootDistance - 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftFootDistance + 1,255,255,255);
        ros_com->drawImageFunction(8,DrawMode::DrawObject, liftandcarryinfo->RightFoot.XMax - 1, liftandcarryinfo->RightFoot.XMax + 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->RightFootDistance - 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->RightFootDistance + 1,255,255,255);
        ros_com->drawImageFunction(9,DrawMode::DrawObject, (liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2 - 1, (liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2 + 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->CenterFootDistance - 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->CenterFootDistance + 1,255,255,255);
        ros_com->drawImageFunction(10,DrawMode::DrawObject, (liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->LeftFoot.XMax)/2 - 1, (liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->LeftFoot.XMax)/2 + 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftCenterFootDistance - 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftCenterFootDistance + 1,255,255,255);
        ros_com->drawImageFunction(11,DrawMode::DrawObject, (liftandcarryinfo->RightFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2 - 1, (liftandcarryinfo->RightFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2 + 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->RightCenterFootDistance - 1, liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->RightCenterFootDistance + 1,255,255,255);
        ros_com->drawImageFunction(12,DrawMode::DrawLine,liftandcarryinfo->LeftFoot.XMin,(liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->LeftFoot.XMax)/2,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftFootDistance,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftCenterFootDistance,0,0,0);
        ros_com->drawImageFunction(13,DrawMode::DrawLine,(liftandcarryinfo->LeftFoot.XMin + liftandcarryinfo->LeftFoot.XMax)/2,(liftandcarryinfo->LeftFoot.XMin+liftandcarryinfo->RightFoot.XMax)/2,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->LeftCenterFootDistance,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->CenterFootDistance,0,0,0);  
        ros_com->drawImageFunction(14,DrawMode::DrawLine,(liftandcarryinfo->LeftFoot.XMin+liftandcarryinfo->RightFoot.XMax)/2,(liftandcarryinfo->RightFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->CenterFootDistance,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->RightCenterFootDistance,0,0,0);
        ros_com->drawImageFunction(15,DrawMode::DrawLine,(liftandcarryinfo->RightFoot.XMin + liftandcarryinfo->RightFoot.XMax)/2,liftandcarryinfo->RightFoot.XMax,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->RightCenterFootDistance,liftandcarryinfo->LeftFoot.YMax - liftandcarryinfo->RightFootDistance,0,0,0);  
        loop_rate.sleep();
    }
    return 0;
}