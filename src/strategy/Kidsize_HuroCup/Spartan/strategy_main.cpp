#include "strategy/strategy_main.h"

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "kidsize");
	ros::NodeHandle nh;
	KidsizeStrategy kidsizeStrategy(nh);
	ros::Rate loop_rate(30);
	kidsizeStrategy.initparameterpath();
	while (nh.ok()) 
	{
		kidsizeStrategy.Loadparameter();	//讀取參數
		kidsizeStrategy.Strategy_Select();	//策略選擇
		kidsizeStrategy.strategymain();		//主策略
		ros::spinOnce();
		kidsizeStrategy.Strategy_Select();	//策略選擇
		loop_rate.sleep();
	}
	return 0;
}
void KidsizeStrategy::strategymain()
{
	if(!strategy_info->getStrategyStart())	//策略指撥開關沒開啟
	{		
		StrategyInitial();//策略初始化
		gettimeofday(&tend_stragegy, NULL);
	}
	else//策略指撥開關開啟
	{
		gettimeofday(&tstart_stragegy, NULL);
		timeuse_stragegy = (1000000*(tstart_stragegy.tv_sec - tend_stragegy.tv_sec) + (tstart_stragegy.tv_usec - tend_stragegy.tv_usec))/1000;
		if(timeuse_stragegy >= 500)
		{	
			if(liftandcarryinfo->Start)
			{
				ros_com->sendBodySector(41);
				tool->Delay(500);
				Initial_IMUdata();
				tool->Delay(1000);
				ros_com->sendBodyAuto(st0_SprX,st0_SprY,0,st0_SprTha,WalkingMode::ContinuousStep,SensorMode(st0_Sprimu));
				liftandcarryinfo->Start=false;
			}
			gettimeofday(&tend, NULL);
			timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
			if(liftandcarryinfo->WhichStrategy == strategy_liftandcarry)	//項目策略為LC
			{
				if(timeuse >= liftandcarryinfo->Delaytime)//身體動作時間到
				{
					StrategyBody();//傳送身體策略
					gettimeofday(&tstart, NULL);
				}
				StrategyClassify();//策略判斷
			}
			else if(liftandcarryinfo->WhichStrategy == strategy_climbingwall)//項目策略為CW
			{
				if(timeuse >= liftandcarryinfo->Delaytime)//身體動作時間到
				{
					CW_StrategyBody();//傳送CW身體策略
					gettimeofday(&tstart, NULL);
				}
				CW_StrategyClassify();//CW策略判斷
			}
			liftandcarryinfo->InitialFlag = true;//初始化旗標
		}
		else
		{
			ros_com->sendBodySector(29);//站立
			tool->Delay(500);
		}	
	}
	if(liftandcarryinfo->WhichStrategy == strategy_liftandcarry)//項目策略為LC
	{
		SendmainData(liftandcarryinfo->WhichStair);
		//----------虛擬腳----------//
		ros_com->drawImageFunction(5,DrawMode::DrawObject,liftandcarryinfo->LeftFoot.XMin,liftandcarryinfo->LeftFoot.XMax,liftandcarryinfo->LeftFoot.YMin,liftandcarryinfo->LeftFoot.YMax,0,255,0);
		ros_com->drawImageFunction(6,DrawMode::DrawObject,liftandcarryinfo->RightFoot.XMin,liftandcarryinfo->RightFoot.XMax,liftandcarryinfo->RightFoot.YMin,liftandcarryinfo->RightFoot.YMax,0,255,0);
		ros_com->drawImageFunction(20,DrawMode::DrawLine,liftandcarryinfo->LeftFoot.XMax,liftandcarryinfo->LeftFoot.XMax,0,240,0,255,0);
		ros_com->drawImageFunction(21,DrawMode::DrawLine,liftandcarryinfo->RightFoot.XMin,liftandcarryinfo->RightFoot.XMin,0,240,0,255,0);
		//--------------------------//
		//----------平移判斷線-------//
		ros_com->drawImageFunction(22,DrawMode::DrawLine,liftandcarryinfo->LeftFoot.XMin-20,liftandcarryinfo->LeftFoot.XMin-20,0,240,0,255,0);
		ros_com->drawImageFunction(23,DrawMode::DrawLine,liftandcarryinfo->RightFoot.XMax+20,liftandcarryinfo->RightFoot.XMax+20,0,240,0,255,0);
		ros_com->drawImageFunction(24,DrawMode::DrawLine,0,320,liftandcarryinfo->LeftFoot.YMax-(SureUpDistance),liftandcarryinfo->LeftFoot.YMax-(SureUpDistance),0,255,0);
		ros_com->drawImageFunction(25,DrawMode::DrawLine,liftandcarryinfo->LeftFoot.XMin-50,liftandcarryinfo->LeftFoot.XMin-50,0,240,0,255,0);
		ros_com->drawImageFunction(26,DrawMode::DrawLine,liftandcarryinfo->RightFoot.XMax+50,liftandcarryinfo->RightFoot.XMax+50,0,240,0,255,0);
		//--------------------------//
	}
	ShowMainData();
}
//---------------------目前是關的---------------------//
void KidsizeStrategy::GetHoleData(const strategy::HoleData &msg)
{
	if(strategy_info->getStrategyStart())
	{
		//-------------讀入避洞旗標和參數-------------//
		liftandcarryinfo->Hole_x0Flag = msg.Hole_x0Flag;
		liftandcarryinfo->Hole_x1Flag = msg.Hole_x1Flag;
		liftandcarryinfo->Hole_y0Flag = msg.Hole_y0Flag;
		liftandcarryinfo->Hole_y1Flag = msg.Hole_y1Flag;
		liftandcarryinfo->AviodHole = msg.AviodHole;
		liftandcarryinfo->Hole_Leftx1 = msg.Hole_Leftx1;
		liftandcarryinfo->Hole_Rightx1 = msg.Hole_Rightx1;
		//-------------------------------------------//
	}
}
//---------------------------------------------------//
void KidsizeStrategy::GetFootData(const strategy::FootData &msg)
{
	if(strategy_info->getStrategyStart())
	{
		//-------------讀入腳距離參數和旗標-------------//
		liftandcarryinfo->LeftFootDistance = msg.LeftFootDistance;
		liftandcarryinfo->RightFootDistance = msg.RightFootDistance;
		liftandcarryinfo->CenterFootDistance = msg.CenterFootDistance;
		liftandcarryinfo->RightCenterFootDistance = msg.RightCenterFootDistance;
		liftandcarryinfo->LeftCenterFootDistance = msg.LeftCenterFootDistance;
		liftandcarryinfo->LeftSlopeFlag = msg.LeftSlopeFlag;
		liftandcarryinfo->RightSlopeFlag = msg.RightSlopeFlag;
		liftandcarryinfo->CenterSlopeFlag = msg.CenterSlopeFlag;
		//----------------------------------------------//
	}
}

void KidsizeStrategy::SendmainData(int WhichStair)
{
	strategy::mainData msg;//建立msg參數
	//-------------獲取msg變數之值-------------//
	msg.WhichStair = WhichStair;
	//-----------------------------------------//
	mainData_Publish.publish(msg);//將msg發布
}

void KidsizeStrategy::StrategyInitial()
{
	
	if(liftandcarryinfo->InitialFlag)
	{
		//ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
		tool->Delay(500);
		ros_com->sendBodySector(29);//站立
		tool->Delay(500);
		liftandcarryinfo->InitialFlag = false;
	}
	if(liftandcarryinfo->WhichStrategy == strategy_liftandcarry)//策略項目為LC
	{
		//-------------初始LC變數-------------//
		liftandcarryinfo->Delaytime = 4000;
		liftandcarryinfo->StrategyState = CalSlope;
		liftandcarryinfo->BodyState = Find_Wood;
		liftandcarryinfo->HoleState = Left;
		liftandcarryinfo->IMUenable = false;		//IMU
		liftandcarryinfo->AviodHole = false;
		liftandcarryinfo->LeftSlopeFlag = false;
		liftandcarryinfo->RightSlopeFlag = false;
		liftandcarryinfo->CenterSlopeFlag = false;
		liftandcarryinfo->Start = true;	//
		liftandcarryinfo->LeftFootDistance = (liftandcarryinfo->LeftFoot.YMax) - liftandcarryinfo->LeftForSlope[0];
		liftandcarryinfo->RightFootDistance = (liftandcarryinfo->RightFoot.YMax) -liftandcarryinfo->RightForSlope[0];
		liftandcarryinfo->CenterFootDistance = (liftandcarryinfo->RightFoot.YMax) -liftandcarryinfo->CenterForSlope[0];
		liftandcarryinfo->LeftCenterFootDistance = liftandcarryinfo->LeftFoot.YMax -liftandcarryinfo->LeftCenterForSlope[0];
		liftandcarryinfo->RightCenterFootDistance = liftandcarryinfo->LeftFoot.YMax -liftandcarryinfo->RightCenterForSlope[0];
		ros_com->sendHeadMotor(HeadMotorID::HorizontalID,HeadpositionX,400);
		tool->Delay(50);
		ros_com->sendHeadMotor(HeadMotorID::VerticalID,HeadpositionY,400);
		tool->Delay(50);
		Theta_offset=0;
		//---------------------------------------//
	}
	else if(liftandcarryinfo->WhichStrategy == strategy_climbingwall)//項目策略為CW
	{
		//-------------初始CW變數-------------//
		liftandcarryinfo->Delaytime = 2000;
		liftandcarryinfo->CW_Strategy = Go_to_stair;
		liftandcarryinfo->BodyState = Zero;
		liftandcarryinfo->Start = true;
		liftandcarryinfo->IMUenable = true;
		CW_step_flag = true;
		CW_handcheck = false;
		CW_finish = false;
		CW_Stair = 0;
		ros_com->sendHeadMotor(HeadMotorID::HorizontalID,HeadpositionX,400);
		tool->Delay(50);
		ros_com->sendHeadMotor(HeadMotorID::VerticalID,HeadpositionY,400);
		tool->Delay(50);
		//------------------------------------//
	}
}
bool avoiddrop;
void KidsizeStrategy::StrategyBody()
{
	int i,j,l;
	double woodshape[240];
	bool BodyMove=false;
	int leftmore=0,rightmore=0,foothalfdistance,rightoutdistance,leftoutdistance,rightcenteroutdistance,leftcenteroutdistance;
	int wood_area;
	double rightoutslope,leftoutslope;
	int leftdistance,rightdistance,centerdistance;
	switch(liftandcarryinfo->BodyState)//身體策略
	{
		case Find_Wood:
			if(liftandcarryinfo->WhichStair == Stair_0)//在第0層
			{
				//-------------初始-------------//
				liftandcarryinfo->rightrotatcount = 0;//++
				liftandcarryinfo->leftrotatcount = 0;
				//------------------------------//
			}
		break;
		case Up:
			//-------------初始-------------//
			liftandcarryinfo->Delaytime = LC_DownDelay;
			liftandcarryinfo->rightrotatcount = 0;
			liftandcarryinfo->leftrotatcount = 0;
			//------------------------------//
			if (liftandcarryinfo->RobotUp)//上坡階段
			{
				//-------------初始-------------//
				BodyMove=false;
				leftmore=0;
				rightmore=0;
				wood_area=0;
				for(i=0;i<240;i++)
					woodshape[i]=-1;
				//------------------------------//
				ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
				tool->Delay(500);

//				ros_com->sendBodyAuto(0,0,0,0,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
//				tool->Delay(1500);
				ros_com->sendBodySector(29);
				tool->Delay(1000);
				ros_com->sendBodySector(40);
				//ros_com->sendBodySector(30);
				tool->Delay(3000);
				ros_com->sendBodyAuto(LC_StepX,LC_StepY,0,LC_StepTha,WalkingMode::LC_Step,SensorMode(LC_Stepimu));//執行上坡動作
				tool->Delay(4000);
				//ros_com->sendBodySector(42);	//蹲下
				//tool->Delay(3000);
				//ros_com->sendBodySector(44);	//站起來
				//tool->Delay(4000);
				ros_com->sendBodySector(29);
				tool->Delay(1000);
				ros_com->sendBodySector(41);
				tool->Delay(500);
				liftandcarryinfo->WhichStair++;//目前層數++
				if(liftandcarryinfo->WhichStair == Stair_1 ||liftandcarryinfo->WhichStair == Stair_2)
				{
					if(liftandcarryinfo->WhichStair == Stair_1)
					{
						ros_com->sendHeadMotor(HeadMotorID::VerticalID,1400,400);
						tool->Delay(1500);
						ros::spinOnce();
					}
					for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
					{
						if(liftandcarryinfo->WhichStair == Stair_1)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == SecColor )
							{
								break;
							}
						}
						else if(liftandcarryinfo->WhichStair == Stair_2)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == TopColor )
							{
								break;
							}
						}
					}
					leftdistance=liftandcarryinfo->LeftFoot.YMax-i;
					for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
					{
						if(liftandcarryinfo->WhichStair == Stair_1)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax ] == SecColor )
							{
								break;
							}
						}
						else if(liftandcarryinfo->WhichStair == Stair_2)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax ] == TopColor )
							{
								break;
							}
						}
					}
					rightdistance=liftandcarryinfo->LeftFoot.YMax-i;
					for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
					{
						if(liftandcarryinfo->WhichStair == Stair_1)
						{
							if( strategy_info->label_model[ ImageWidth * i + (liftandcarryinfo->RightFoot.XMax+liftandcarryinfo->LeftFoot.XMin)/2 ] == SecColor )
							{
								break;
							}
						}
						else if(liftandcarryinfo->WhichStair == Stair_2)
						{
							if( strategy_info->label_model[ ImageWidth * i + (liftandcarryinfo->RightFoot.XMax+liftandcarryinfo->LeftFoot.XMin)/2 ] == TopColor )
							{
								break;
							}
						}
					}
					centerdistance=liftandcarryinfo->LeftFoot.YMax-i;
					if(leftdistance > centerdistance && centerdistance < rightdistance )	//提早避免010 
					{
						for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
						{
							if(liftandcarryinfo->WhichStair == Stair_1)
							{
								if( strategy_info->label_model[ ImageWidth * i +  314] == SecColor )
								{
									break;
								}
							}
							else if(liftandcarryinfo->WhichStair == Stair_2)
							{
								if( strategy_info->label_model[ ImageWidth * i +  314] == TopColor )
								{
									break;
								}
							}
						}
						rightoutdistance=liftandcarryinfo->LeftFoot.YMax-i;
						for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
						{
							if(liftandcarryinfo->WhichStair == Stair_1)
							{
								if( strategy_info->label_model[ ImageWidth * i +  ((314+liftandcarryinfo->RightFoot.XMax)/2) ] == SecColor )
								{
									break;
								}
							}
							else if(liftandcarryinfo->WhichStair == Stair_2)
							{
								if( strategy_info->label_model[ ImageWidth * i + ((314+liftandcarryinfo->RightFoot.XMax)/2)] == TopColor )
								{
									break;
								}
							}
						}
						rightcenteroutdistance=liftandcarryinfo->LeftFoot.YMax-i;
						for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
						{
							if(liftandcarryinfo->WhichStair == Stair_1)
							{
								if( strategy_info->label_model[ ImageWidth * i +  5 ] == SecColor )
								{
									break;
								}
							}
							else if(liftandcarryinfo->WhichStair == Stair_2)
							{
								if( strategy_info->label_model[ ImageWidth * i +  5] == TopColor )
								{
									break;
								}
							}
						}
						leftoutdistance=liftandcarryinfo->LeftFoot.YMax-i;
						for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
						{
							if(liftandcarryinfo->WhichStair == Stair_1)
							{
								if( strategy_info->label_model[ ImageWidth * i +  ((5+liftandcarryinfo->LeftFoot.XMin)/2) ] == SecColor )
								{
									break;
								}
							}
							else if(liftandcarryinfo->WhichStair == Stair_2)
							{
								if( strategy_info->label_model[ ImageWidth * i +  ((5+liftandcarryinfo->LeftFoot.XMin)/2)] == TopColor )
								{
									break;
								}
							}
						}
						leftcenteroutdistance=liftandcarryinfo->LeftFoot.YMax-i;
						rightoutslope=(((double)(rightoutdistance-rightcenteroutdistance)/(double)(314-(314+liftandcarryinfo->RightFoot.XMax)/2))-((double)(rightcenteroutdistance-rightdistance)/(double)((314+liftandcarryinfo->RightFoot.XMax)/2-liftandcarryinfo->RightFoot.XMax)));
						leftoutslope=(((double)(leftcenteroutdistance-leftoutdistance)/(double)((5+liftandcarryinfo->LeftFoot.XMin)/2-5))-(double)(leftdistance-leftcenteroutdistance)/(double)(liftandcarryinfo->LeftFoot.XMin-(5+liftandcarryinfo->LeftFoot.XMin)/2));
						if(abs(leftoutslope)>abs(rightoutslope))
							liftandcarryinfo->BodyState = RightShift;
						else if(abs(leftoutslope)<abs(rightoutslope))
							liftandcarryinfo->BodyState = LeftShift;
						else
						{
							if(leftdistance>rightdistance)
								liftandcarryinfo->BodyState = RightShift;
							else
								liftandcarryinfo->BodyState = LeftShift;		
						}
					}
					else
					{
						for(i=0;i<240;i++)	//垂直
						{
							//------算出下一層板的xmin------//
							for(j=1;j<319;j++)
							{
								if(liftandcarryinfo->WhichStair == Stair_1)
								{
									if( strategy_info->label_model[ ImageWidth * i + j ] == SecColor )
									{
										break;
									}
								}
								else if(liftandcarryinfo->WhichStair == Stair_2)
								{
									if( strategy_info->label_model[ ImageWidth * i + j ] == TopColor )
									{
										break;
									}
								}
							}
							//-----------------------------//
							if(j<319)
							{
								//------算出下一層板的xmax------//
								for(l=318;l>0;l--)
								{
									if(liftandcarryinfo->WhichStair == Stair_1)
									{
										if( strategy_info->label_model[ ImageWidth * i + l ] == SecColor )
										{
											break;
										}
									}
									else if(liftandcarryinfo->WhichStair == Stair_2)
									{
										if( strategy_info->label_model[ ImageWidth * i + l ] == TopColor )
										{
											break;
										}
									}
								}
								//-----------------------------//
								woodshape[i]=(j+l)/2;		//看下層板的中心點偏左或偏右
								//------算出下一層板的面積------//
								for(j;j<=l;j++)
								{
									if(liftandcarryinfo->WhichStair == Stair_1)
									{
										if( strategy_info->label_model[ ImageWidth * i + j ] == SecColor )
											wood_area++;			
									}
									else if(liftandcarryinfo->WhichStair == Stair_2)
									{
										if( strategy_info->label_model[ ImageWidth * i + j ] == TopColor )
											wood_area++;
									}
								}
								//-----------------------------//
							}
						}
						if(wood_area>=15000)
							liftandcarryinfo->BodyState =  SmallFront;
						else
						{
							for(i=0;i<240;i++)
							{
								if(woodshape[i]==-1)
									continue;
								else if(woodshape[i]<159.5)
									leftmore++;
								else if(woodshape[i]>159.5)
									rightmore++;
							}
							if(leftmore>rightmore && leftmore > 10)
							{
								if(wood_area>=8000)
								{
									liftandcarryinfo->BodyState =  SmallLeftRotation;
									BodyMove=true;
								}
								else
								{
									liftandcarryinfo->BodyState =  BigLeftRotation;
									BodyMove=true;
								}
							}
							else if(leftmore<rightmore && rightmore > 10)
							{
								if(wood_area>=8000)
								{
									liftandcarryinfo->BodyState =  SmallRightRotation;
									BodyMove=true;
								}
								else
								{
									liftandcarryinfo->BodyState =  BigRightRotation;
									BodyMove=true;
								}
							}	
							else
							{
								ros_com->sendHeadMotor(HeadMotorID::HorizontalID,HeadpositionX-600,400);
								tool->Delay(1500);
								ros::spinOnce();
								for(i=239;i>=0;i--)
								{
									if(liftandcarryinfo->WhichStair == Stair_1)
									{
										if( strategy_info->label_model[ ImageWidth * i + 314 ] == SecColor )
										{
											liftandcarryinfo->BodyState =  BigRightRotation;
											BodyMove=true;
											break;
										}
									}
									else if(liftandcarryinfo->WhichStair == Stair_2)
									{
										if( strategy_info->label_model[ ImageWidth * i + 314 ] == TopColor )
										{
											liftandcarryinfo->BodyState =  BigRightRotation;
											BodyMove=true;
											break;
										}
									}
								}
								if(!BodyMove)
								{
									ros_com->sendHeadMotor(HeadMotorID::HorizontalID,HeadpositionX+600,400);
									tool->Delay(1500);
									ros::spinOnce();
									for(i=239;i>=0;i--)
									{
										if(liftandcarryinfo->WhichStair == Stair_1)
										{
											if( strategy_info->label_model[ ImageWidth * i + 5 ] == SecColor )
											{
												liftandcarryinfo->BodyState =  BigLeftRotation;
												BodyMove=true;
												break;
											}
										}
										else if(liftandcarryinfo->WhichStair == Stair_2)
										{
											if( strategy_info->label_model[ ImageWidth * i + 5 ] == TopColor )
											{
												liftandcarryinfo->BodyState =  BigLeftRotation;
												BodyMove=true;
												break;
											}
										}
									}
								}
							}
							if(!BodyMove)
								liftandcarryinfo->BodyState =  SmallFront;
						}
					}
					ros_com->sendHeadMotor(HeadMotorID::HorizontalID,HeadpositionX,400);
					tool->Delay(50);
					ros_com->sendHeadMotor(HeadMotorID::VerticalID,HeadpositionY,400);
					tool->Delay(50);
					ros::spinOnce();
					Initial_IMUdata();
					ShowMainData();
					if(liftandcarryinfo->IMUenable)
					{
						if(liftandcarryinfo->BodyState ==  BigLeftRotation)
						{
							ros_com->sendBodyAuto(BigLeftRotationX,BigLeftRotationY,0,BigLeftRotationTha,WalkingMode::ContinuousStep,SensorMode(BigLeftRotationimu));
							tool->Delay(1500);
							Initial_IMUdata();
						}
						else if(liftandcarryinfo->BodyState ==  BigRightRotation)
						{
							ros_com->sendBodyAuto(BigRightRotationX,BigRightRotationY,0,BigRightRotationTha,WalkingMode::ContinuousStep,SensorMode(BigRightRotationimu));
							tool->Delay(1500);
							Initial_IMUdata();
						}
						else if(liftandcarryinfo->BodyState ==  SmallLeftRotation)
						{
							ros_com->sendBodyAuto(SmallLeftRotationX,SmallLeftRotationY,0,SmallLeftRotationTha,WalkingMode::ContinuousStep,SensorMode(SmallLeftRotationimu));
							tool->Delay(1500);
							Initial_IMUdata();
						}
						else if(liftandcarryinfo->BodyState ==  SmallRightRotation)
						{
							ros_com->sendBodyAuto(SmallRightRotationX,SmallRightRotationY,0,SmallRightRotationTha,WalkingMode::ContinuousStep,SensorMode(SmallRightRotationimu));
							tool->Delay(1500);
							Initial_IMUdata();
						}
						else if(liftandcarryinfo->BodyState == RightShift)
							ros_com->sendBodyAuto(RightShiftX,RightShiftY,0,RightShiftTha+Theta_offset,WalkingMode::ContinuousStep,SensorMode(RightShiftimu));
						else if(liftandcarryinfo->BodyState == LeftShift)
							ros_com->sendBodyAuto(LeftShiftX,LeftShiftY,0,LeftShiftTha+Theta_offset,WalkingMode::ContinuousStep,SensorMode(LeftShiftimu));
						else
							ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
					}
					else
					{
						if(liftandcarryinfo->BodyState ==  BigLeftRotation)
							ros_com->sendBodyAuto(BigLeftRotationX,BigLeftRotationY,0,BigLeftRotationTha,WalkingMode::ContinuousStep,SensorMode(BigLeftRotationimu));
						else if(liftandcarryinfo->BodyState ==  BigRightRotation)
							ros_com->sendBodyAuto(BigRightRotationX,BigRightRotationY,0,BigRightRotationTha,WalkingMode::ContinuousStep,SensorMode(BigRightRotationimu));
						else if(liftandcarryinfo->BodyState ==  SmallLeftRotation)
							ros_com->sendBodyAuto(SmallLeftRotationX,SmallLeftRotationY,0,SmallLeftRotationTha,WalkingMode::ContinuousStep,SensorMode(SmallLeftRotationimu));
						else if(liftandcarryinfo->BodyState ==  SmallRightRotation)
							ros_com->sendBodyAuto(SmallRightRotationX,SmallRightRotationY,0,SmallRightRotationTha,WalkingMode::ContinuousStep,SensorMode(SmallRightRotationimu));
						else if(liftandcarryinfo->BodyState == RightShift)
							ros_com->sendBodyAuto(RightShiftX,RightShiftY,0,RightShiftTha+Theta_offset,WalkingMode::ContinuousStep,SensorMode(RightShiftimu));
						else if(liftandcarryinfo->BodyState == LeftShift)
							ros_com->sendBodyAuto(LeftShiftX,LeftShiftY,0,LeftShiftTha+Theta_offset,WalkingMode::ContinuousStep,SensorMode(LeftShiftimu));
						else
							ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_3)//在第3層	
				{
					liftandcarryinfo->RobotUp = false;
					liftandcarryinfo->BodyState = SmallFront;
					ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
					tool->Delay(2000);
				}
			}
			else
			{
				ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
				tool->Delay(3000);
				ros_com->sendBodySector(29);
				tool->Delay(1000);
				ros_com->sendBodySector(45);
				tool->Delay(5000);
				ros_com->sendBodyAuto(LC_DownX,LC_DownY,0,LC_DownTha,WalkingMode::LC_Down,SensorMode(LC_Downimu));    //執行下坡動作
				ros_com->sendBodySector(29);
				tool->Delay(3000);
				ros_com->sendBodySector(41);
				tool->Delay(500);
				Initial_IMUdata();
				liftandcarryinfo->WhichStair++;	//目前層數++
				liftandcarryinfo->BodyState = SmallFront;
				ros::spinOnce();
				if (liftandcarryinfo->WhichStair == Stair_4 || liftandcarryinfo->WhichStair == Stair_5)
					ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
				else if (liftandcarryinfo->WhichStair == Stair_6)//在第6層
				{
					liftandcarryinfo->BodyState = Zero;//不動作
					liftandcarryinfo->StrategyState = Finish;//策略結束
				}
			}
		break;
		case SmallFront:
			Theta_Offset();
			liftandcarryinfo->Delaytime = SmallFrontDelay;
			if(liftandcarryinfo->WhichStair == Stair_0)//在第0層
				ros_com->sendContinuousValue(st0_SmallFrontX,st0_SmallFrontY,0,st0_SmallFrontTha+Theta_offset,SensorMode(st0_SmallFrontimu));
			else//執行特殊步態
				ros_com->sendContinuousValue(SmallFrontX,SmallFrontY,0,SmallFrontTha+Theta_offset,SensorMode(SmallFrontimu));
		break;
		case BigFront:
			Theta_Offset();
			liftandcarryinfo->Delaytime = BigFrontDelay;
			if(liftandcarryinfo->WhichStair == Stair_0)//在第0層
				ros_com->sendContinuousValue(st0_BigFrontX,st0_BigFrontY,0,st0_BigFrontTha+Theta_offset,SensorMode(st0_BigFrontimu));//執行一般步態
			else
				ros_com->sendContinuousValue(BigFrontX,BigFrontY,0,BigFrontTha+Theta_offset,SensorMode(BigFrontimu));//執行特殊步態
		break;
		case Spr:
			Theta_Offset();
			liftandcarryinfo->Delaytime = SprDelay;
			if(liftandcarryinfo->WhichStair == Stair_0)//在第0層	
				ros_com->sendContinuousValue(st0_SprX,st0_SprY,0,st0_SprTha+Theta_offset,SensorMode(st0_Sprimu));//執行特殊步態
			else
				ros_com->sendContinuousValue(SprX,SprY,0,SprTha+Theta_offset,SensorMode(Sprimu));
		break;
		case LeftShift:
			Theta_Offset();
			liftandcarryinfo->Delaytime = LeftShiftDelay;
			ros_com->sendContinuousValue(LeftShiftX,LeftShiftY,0,LeftShiftTha+Theta_offset,SensorMode(LeftShiftimu));
		break;
		case RightShift:
			Theta_Offset();
			liftandcarryinfo->Delaytime = RightShiftDelay;
			ros_com->sendContinuousValue(RightShiftX,RightShiftY,0,RightShiftTha+Theta_offset,SensorMode(RightShiftimu));
		break;
		case SmallLeftShift:
			Theta_Offset();
			liftandcarryinfo->Delaytime = SmallLeftShiftDelay;
			ros_com->sendContinuousValue(SmallLeftShiftX,SmallLeftShiftY,0,SmallLeftShiftTha+Theta_offset,SensorMode(SmallLeftShiftimu));
		break;
		case SmallRightShift:
			Theta_Offset();
			liftandcarryinfo->Delaytime = SmallRightShiftDelay;
			ros_com->sendContinuousValue(SmallRightShiftX,SmallRightShiftY,0,SmallRightShiftTha+Theta_offset,SensorMode(SmallRightShiftimu));
		break;
		case SmallLeftRotation:
			liftandcarryinfo->Delaytime = SmallLeftRotationDelay;
			ros_com->sendContinuousValue(SmallLeftRotationX,SmallLeftRotationY,0,SmallLeftRotationTha,SensorMode(SmallLeftRotationimu));
			liftandcarryinfo->leftrotatcount++;
			if (liftandcarryinfo->leftrotatcount > 4)
			{
				liftandcarryinfo->BodyState = SmallRightShift;
				liftandcarryinfo->leftrotatcount = 0;
			}
			Initial_IMUdata();
		break;
		case SmallRightRotation:
			liftandcarryinfo->Delaytime = SmallRightShiftDelay;
			ros_com->sendContinuousValue(SmallRightRotationX,SmallRightRotationY,0,SmallRightRotationTha,SensorMode(SmallRightRotationimu));
			liftandcarryinfo->rightrotatcount++;
			if (liftandcarryinfo->rightrotatcount > 4)
			{
				liftandcarryinfo->BodyState = SmallLeftShift;
				liftandcarryinfo->rightrotatcount = 0;
			}
			Initial_IMUdata();
		break;
		case BigLeftRotation:
			liftandcarryinfo->Delaytime = BigLeftRotationDelay;
			ros_com->sendContinuousValue(BigLeftRotationX,BigLeftRotationY,0,BigLeftRotationTha,SensorMode(BigLeftRotationimu));
			Initial_IMUdata();
		break;
		case BigRightRotation:
			liftandcarryinfo->Delaytime = BigRightRotationDelay;
			ros_com->sendContinuousValue(BigRightRotationX,BigRightRotationY,0,BigRightRotationTha,SensorMode(BigRightRotationimu));
			Initial_IMUdata();
		break;
		case Zero:
			liftandcarryinfo->Delaytime = 0;
		break;
	}
}

void KidsizeStrategy::CW_distance()
{
	//------------執行頭部動作-----------//
	ros_com->sendHeadMotor(HeadMotorID::HorizontalID,HeadpositionX,400);
	tool->Delay(50);
	ros_com->sendHeadMotor(HeadMotorID::VerticalID,HeadpositionY,400);
	tool->Delay(50);
	//----------------------------------//
	//------------初始化----------------//
	CW_Leftfoot_flag = false;
	CW_Rightfoot_flag = false;
	CW_Leftfoot_distance = 9999 + liftandcarryinfo->LeftFoot.YMax;
	CW_Rightfoot_distance = 9999 + liftandcarryinfo->RightFoot.YMax;
	//---------------------------------//
	//-------------算出左腳與梯子的距離--------------//
	for(int h = liftandcarryinfo->LeftFoot.YMax; h > 0; h--)//h=左虛擬腳Y座標最大值
	{
        if((strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::YellowLabel))//取樣像素點顏色=目標色模顏色
		{
			CW_Leftfoot_distance = liftandcarryinfo->LeftFoot.YMax - h ;//左腳與梯子的距離=左虛擬腳Y座標最大值-h
			for (h ; h > 0 ; h--)
			{
                if((strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] != (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::YellowLabel))
				{
					CW_Slope_Leftdistance = liftandcarryinfo->LeftFoot.YMax - h ;
					break;
				}
			}
			break;
		}
	}
	//----------------------------------------------//
	//-------------算出右腳與梯子的距離--------------(int)LabelMark::OrangeLabel//
	for(int h = liftandcarryinfo->RightFoot.YMax; h > 0; h--)//h=右虛擬腳Y座標最大值
	{
        if((strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->RightFoot.XMax ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::YellowLabel))//取樣像素點顏色=目標色模顏色
		{
			CW_Rightfoot_distance = liftandcarryinfo->RightFoot.YMax - h ;//右腳與梯子的距離=右虛擬腳Y座標最大值-h
			for (h ; h > 0 ; h--)
			{
                if((strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->RightFoot.XMax ] != (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::YellowLabel))
				{
					CW_Slope_Rightdistance = liftandcarryinfo->RightFoot.YMax - h ;
					break;
				}
			}
			break;
		}
	}
	//----------------------------------------------//
	CW_Slope = (float)(CW_Slope_Leftdistance - CW_Slope_Rightdistance)/(liftandcarryinfo->RightFoot.XMax - liftandcarryinfo->LeftFoot.XMin)+0.028;
	//-----------------判斷左腳旗標-----------------//
	for(int h = liftandcarryinfo->LeftFoot.YMax; h >= liftandcarryinfo->LeftFoot.YMin; h--)//h=左虛擬腳Y座標最大值;h>左虛擬腳Y座標最小值;h--
	{
        if((strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::YellowLabel))//取樣像素點顏色=目標色模顏色
		{
			CW_Leftfoot_flag = true;//左腳旗標=true
			break;
		}
	}
	//----------------------------------------------//
	//-----------------判斷右腳旗標-----------------//
	for(int h = liftandcarryinfo->RightFoot.YMax; h >= liftandcarryinfo->RightFoot.YMin; h--)//h=右虛擬腳Y座標最大值;h>右虛擬腳Y座標最小值;h--
	{
        if((strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->RightFoot.XMax] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin ] == (int)LabelMark::YellowLabel))//取樣像素點顏色=目標色模顏色
		{
			CW_Rightfoot_flag = true;//右腳旗標=true
			break;
		}
	}
	//----------------------------------------------//
}

void KidsizeStrategy::CW_StrategyBody()
{
	if(CW_step_flag)//爬樓梯步態旗標
	{
		switch (liftandcarryinfo->BodyState)
		{
			case Spr:
				Theta_Offset();
				liftandcarryinfo->Delaytime = SprDelay;
				ros_com->sendContinuousValue(st0_SprX,st0_SprY,0,st0_SprTha+Theta_offset,SensorMode(st0_Sprimu));
			break;
			case BigLeftRotation:
				liftandcarryinfo->Delaytime = BigLeftRotationDelay;
				ros_com->sendContinuousValue(BigLeftRotationX,BigLeftRotationY,0,BigLeftRotationTha,SensorMode(BigLeftRotationimu));
				if(liftandcarryinfo->Laststate == BigRightRotation || liftandcarryinfo->Laststate == SmallRightRotation)
					CW_Count++;
				else
					CW_Count = 0;
				liftandcarryinfo->Laststate = BigLeftRotation;
				Initial_IMUdata();
			break;
			case BigRightRotation:
				liftandcarryinfo->Delaytime = BigRightRotationDelay;
				ros_com->sendContinuousValue(BigRightRotationX,BigRightRotationY,0,BigRightRotationTha,SensorMode(BigRightRotationimu));
				if(liftandcarryinfo->Laststate == BigLeftRotation || liftandcarryinfo->Laststate == SmallLeftRotation)
					CW_Count++;
				else
					CW_Count = 0;
				liftandcarryinfo->Laststate = BigRightRotation;
				Initial_IMUdata();
			break;
			case BigFront:
				Theta_Offset();
				liftandcarryinfo->Delaytime = BigFrontDelay;
				ros_com->sendContinuousValue(st0_BigFrontX,st0_BigFrontY,0,st0_BigFrontTha+Theta_offset,SensorMode(st0_BigFrontimu));
				CW_Count = 0;
			break;
			case SmallFront:
				Theta_Offset();
				liftandcarryinfo->Delaytime = SmallFrontDelay;
				ros_com->sendContinuousValue(st0_SmallFrontX,st0_SmallFrontY,0,st0_SmallFrontTha+Theta_offset,SensorMode(st0_SmallFrontimu));
				CW_Count = 0;
			break;
			case SmallLeftRotation:
				liftandcarryinfo->Delaytime = BigLeftRotationDelay;
				ros_com->sendContinuousValue(SmallLeftRotationX,SmallLeftRotationY,0,SmallLeftRotationTha,SensorMode(SmallLeftRotationimu));//設定成10
				if(liftandcarryinfo->Laststate == BigRightRotation || liftandcarryinfo->Laststate == SmallRightRotation)
					CW_Count++;
				else
					CW_Count = 0;
				liftandcarryinfo->Laststate = SmallLeftRotation;
				Initial_IMUdata();
			break;
			case SmallRightRotation:
				liftandcarryinfo->Delaytime = SmallRightRotationDelay;
				ros_com->sendContinuousValue(SmallRightRotationX,SmallRightRotationY,0,SmallRightRotationTha,SensorMode(SmallRightRotationimu));
				if(liftandcarryinfo->Laststate == BigLeftRotation || liftandcarryinfo->Laststate == SmallLeftRotation)
					CW_Count++;
				else
					CW_Count = 0;
				liftandcarryinfo->Laststate = SmallRightRotation;
				Initial_IMUdata();
			break;
			case Zero:
			break;
		}
	}
}

void KidsizeStrategy::CW_StrategyClassify()
{
	int i,stairdistance;
	switch (liftandcarryinfo->CW_Strategy) 
	{
		case Go_to_stair:
			CW_distance();
			if(CW_Count >=4)
			{
				liftandcarryinfo->BodyState = BigFront;
			}
				
			else if(CW_Leftfoot_flag && CW_Rightfoot_flag)	//11
			{
				
				if(CW_Leftfoot_distance >= 40 && CW_Rightfoot_distance >= 40)
					liftandcarryinfo->BodyState = BigFront;
				else if(CW_Slope < -0.1)
				{
					if(abs(CW_Slope) <= 0.2)
						liftandcarryinfo->BodyState = SmallLeftRotation;
					else
						liftandcarryinfo->BodyState = BigLeftRotation;	
				}
				else if(CW_Slope > 0.1)
				{
					if(abs(CW_Slope) <= 0.2)
						liftandcarryinfo->BodyState = SmallRightRotation;
					else
						liftandcarryinfo->BodyState = BigRightRotation;	
				}
				else if(CW_Leftfoot_distance > CW_Up_distance && CW_Rightfoot_distance > CW_Up_distance)		//00
					liftandcarryinfo->BodyState = SmallFront;
				else if(CW_Leftfoot_distance <= CW_Up_distance && CW_Rightfoot_distance <= CW_Up_distance)       //11
				{
					liftandcarryinfo->CW_Strategy = Climb_to_first_stair;
					liftandcarryinfo->BodyState = Zero;
				}
			}
			else if(CW_Leftfoot_flag && !CW_Rightfoot_flag)			//10
				liftandcarryinfo->BodyState = BigLeftRotation;
			else if(!CW_Leftfoot_flag && CW_Rightfoot_flag)			//01
				liftandcarryinfo->BodyState = BigRightRotation;
			else if(!CW_Leftfoot_flag && !CW_Rightfoot_flag)		//00
				liftandcarryinfo->BodyState = Spr;
		break;
		case Climb_to_first_stair:
			CW_step_flag = false;
			if(!CW_handcheck)
			{
	//			ros_com->sendContinuousValue(st0_SmallFrontX,st0_SmallFrontY,0,st0_SmallFrontTha+Theta_offset,SensorMode(st0_SmallFrontimu));
				//ros_com->sendBodyAuto(SmallFrontX,SmallFrontY,0,SmallFrontTha,WalkingMode::ContinuousStep,SensorMode(SmallFrontimu));
				//tool->Delay(1500);
				ros_com->sendBodyAuto(SmallLeftRotationX,SmallLeftRotationY,0,SmallLeftRotationTha,WalkingMode::ContinuousStep,SensorMode(SmallLeftRotationimu));
				tool->Delay(1500);
				ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2048, 400);
				tool->Delay(50);
				ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1500, 400);
				tool->Delay(50);
				
				ros_com->sendBodySector(29);
				tool->Delay(500);
				ros_com->sendBodySector(30);//爬梯站姿
				tool->Delay(1000);
				ros_com->sendBodySector(21);//手
				ROS_INFO("Sector = 21 , delay = %d", CW_first_hand_delay);
				tool->Delay(CW_first_hand_delay);
				CW_handcheck = true;
			}
			else
			{
				ros_com->sendBodySector(22);//腳
				ROS_INFO("Sector = 22 , delay = %d", CW_first_foot_delay);
				tool->Delay(CW_first_foot_delay);
				tool->Delay(5000);
				CW_Stair++;
				liftandcarryinfo->CW_Strategy = Climb_to_second_stair;
				CW_handcheck = false;
			}
		break;
		case Climb_to_second_stair:
			if(!CW_handcheck)
			{
				 for (i = 1500; i<=2500; i+=2)	//從平視往上看
				{
					ros_com->sendHeadMotor(HeadMotorID::VerticalID, i, 400);
					tool->Delay(50);
					ros::spinOnce();
					tool->Delay(50);
					if ((((strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] == (int)LabelMark::RedLabel))||(strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] == (int)LabelMark::YellowLabel)) || ((strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->RightFoot.XMin ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->RightFoot.XMin ] ==  (int)LabelMark::YellowLabel)))
					{
						ROS_INFO("%d",i);
						stairdistance = i - 1500;	//算梯距
						CW_stairdistance[CW_Stair+2] = stairdistance;
						break;
					}
				}
				ros_com->sendBodySector(23);
				ROS_INFO("Sector = 23 , delay = %d", CW_hand_delay);
				tool->Delay(CW_hand_delay);
				CW_handcheck = true;
			}
			else
			{
				ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1500, 400);
				tool->Delay(50);
				ros_com->sendBodySector(24);
				ROS_INFO("Sector = 24 , delay = %d", CW_foot_delay);
				tool->Delay(4000);
				CW_Stair++;
				liftandcarryinfo->CW_Strategy = On_stair_top;
				CW_handcheck = false;
			}
		break;
		case Climb_to_third_stair:
			if(!CW_handcheck)
			{
				for (i = 1500; i<=2500; i+=2)	//從平視往上看
				{
					ros_com->sendHeadMotor(HeadMotorID::VerticalID, i, 400);
					tool->Delay(50);
					ros::spinOnce();
					tool->Delay(50);
					if (((strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] == (int)LabelMark::YellowLabel)) || ((strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->RightFoot.XMin ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] == (int)LabelMark::YellowLabel)))
					{
						ROS_INFO("%d",i);
						stairdistance = i - 1500;	//算梯距
						CW_stairdistance[CW_Stair+2] = stairdistance;
						break;
					}
				}
				ros_com->sendBodySector(25);
				ROS_INFO("Sector = 25 , delay = %d", CW_hand_delay);
				tool->Delay(2000);
				CW_handcheck = true;
			}
			else
			{
				ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1500, 400);
				tool->Delay(50);
				ros_com->sendBodySector(26);
				ROS_INFO("Sector = 26 , delay = %d", CW_foot_delay);
				tool->Delay(CW_foot_delay);
				CW_Stair++;
				liftandcarryinfo->CW_Strategy = Climb_next;
				CW_handcheck = false;
			}
		break;
		case Climb_next:
			if (CW_Stair == CW_top_stair)
			{
				CW_finish = true;
				liftandcarryinfo->CW_Strategy = On_stair_top;
			}
			else if (!CW_handcheck)
			{
				for (i = 1500; i<=2500; i+=2)	//從平視往上看
				{
					ros_com->sendHeadMotor(HeadMotorID::VerticalID, i, 400);
					tool->Delay(50);
					ros::spinOnce();
					tool->Delay(50);
					if (((strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] ==(int)LabelMark::YellowLabel)) || ((strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->RightFoot.XMin ] == (int)LabelMark::RedLabel)||(strategy_info->label_model[ ImageWidth * 100 + liftandcarryinfo->LeftFoot.XMax ] == (int)LabelMark::YellowLabel)))
					{
						ROS_INFO("%d",i);
						stairdistance = i - 1500;	//算梯距
						CW_stairdistance[CW_Stair+2] = stairdistance;
						break;
					}
				}
				if (i >= 2500)
				{
					CW_finish = true;
					liftandcarryinfo->CW_Strategy = On_stair_top;
				}
				else
				{
					if(CW_stairdistance[CW_Stair]+CW_stairdistance[CW_Stair+1]<=320)			//10cm
						ros_com->sendBodySector(23);
					else if(CW_stairdistance[CW_Stair]+CW_stairdistance[CW_Stair+1]<=500)		//13cm
						ros_com->sendBodySector(25);
					else if(CW_stairdistance[CW_Stair]+CW_stairdistance[CW_Stair+1]<=620)		//15cm
						ros_com->sendBodySector(27);
					else if(CW_stairdistance[CW_Stair]+CW_stairdistance[CW_Stair+1]<=760)		//17cm
						ros_com->sendBodySector(31);
					else							//20cm
						ros_com->sendBodySector(33);
					ROS_INFO("Sector = 23 , delay = %d", CW_hand_delay);
					tool->Delay(CW_hand_delay);
					CW_handcheck = true;
				}
			}
			else
			{
				ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1500, 400);
				tool->Delay(50);
				if(CW_stairdistance[CW_Stair]<=160)
					ros_com->sendBodySector(24);
				else if(CW_stairdistance[CW_Stair]<=250)
					ros_com->sendBodySector(26);
				else if(CW_stairdistance[CW_Stair]<=310)
					ros_com->sendBodySector(28);
				else if(CW_stairdistance[CW_Stair]<=380)
					ros_com->sendBodySector(32);
				else
					ros_com->sendBodySector(34);
				ROS_INFO("Sector = 24 , delay = %d", CW_foot_delay);
				tool->Delay(CW_foot_delay);
				CW_Stair++;
				CW_handcheck = false;
			}
		break;
		case On_stair_top:
			ROS_INFO("Already on top of stair!!!");
		break;
	}
}

void KidsizeStrategy::ShowMainData()		//比賽時建議關閉
{
	ROS_INFO("---------------------------");
	ROS_INFO("IMUenable = %s",(liftandcarryinfo->IMUenable == true ? "true" : "false"));
	ROS_INFO("initial_IMUdata = %lf", initial_IMUdata);
	ROS_INFO("current_IMUdata = %lf", current_IMUdata);
	ROS_INFO("Theta_offset = %d", Theta_offset);
	switch (liftandcarryinfo->WhichStrategy)
	{
		case strategy_liftandcarry:
			ROS_INFO("SureUpDistance = %d",SureUpDistance);
			ROS_INFO("WhichStrategy = LC");
			ROS_INFO("WhichStair = %d",liftandcarryinfo->WhichStair);      
			ROS_INFO("SlopeFlag = %s %s %s",(liftandcarryinfo->LeftSlopeFlag == true ? "true" : "false"),(liftandcarryinfo->CenterSlopeFlag == true ? "true" : "false"),(liftandcarryinfo->RightSlopeFlag == true ? "true" : "false"));
			ROS_INFO("FootDistance = %d %d %d %d %d",liftandcarryinfo->LeftFootDistance,liftandcarryinfo->LeftCenterFootDistance,liftandcarryinfo->CenterFootDistance,liftandcarryinfo->RightCenterFootDistance,liftandcarryinfo->RightFootDistance);
			switch (liftandcarryinfo->StrategyState)
			{
				case CalSlope:
					ROS_INFO("StrategyState = CalSlope");
				break;
				case Finish:
					ROS_INFO("StrategyState = Finish");
				break;
				default:
				break;
			}
			switch (liftandcarryinfo->BodyState) 
			{
				case Spr:
					ROS_INFO("BodyState = Spr\n");
				break;
				case Up:
					ROS_INFO("BodyState = Up\n");
				break;
				case Zero:
					ROS_INFO("BodyState = Zero\n");
				break;
				case SmallFront:
					ROS_INFO("BodyState = SmallFront\n");
				break;
				case SmallLeftRotation:
					ROS_INFO("BodyState = SmallLeftRotation\n");
				break;
				case SmallRightRotation:
					ROS_INFO("BodyState = SmallRightRotation\n");
				break;
				case BigLeftRotation:
					ROS_INFO("BodyState = BigLeftRotation\n");
				break;
				case BigRightRotation:
					ROS_INFO("BodyState = BigRightRotation\n");
				break;
				case RightShift:
					ROS_INFO("BodyState = RightShift\n");
				break;
				case LeftShift:
					ROS_INFO("BodyState = LeftShift\n");
				break;
				case SmallLeftShift:
					ROS_INFO("BodyState = SmallLeftShift\n");
				break;
				case SmallRightShift:
					ROS_INFO("BodyState = SmallRightShift\n");
				break;
				case BigFront:
					ROS_INFO("BodyState = BigFront\n");
				break;
				case Find_Wood:
					ROS_INFO("BodyState = Find_Wood\n");
				break;
				default:
				break;
			}
		break;
		case strategy_climbingwall:	
			ROS_INFO("WhichStrategy = CW");
			ROS_INFO("CW_Stair = %d",CW_Stair);
			ROS_INFO("CW_finish = %s",(CW_finish == true ? "true" : "false"));
			ROS_INFO("CW_foot_flag = %s %s",(CW_Leftfoot_flag == true ? "true" : "false"),(CW_Rightfoot_flag == true ? "true" : "false"));
			ROS_INFO("CW_distance = %d %d", CW_Leftfoot_distance, CW_Rightfoot_distance);
			ROS_INFO("CW_Slope = %f\n",CW_Slope);
			ROS_INFO("Count = %d",CW_Count);
			switch (liftandcarryinfo->CW_Strategy) 
			{
				case Go_to_stair:
					ROS_INFO("CW_Strategy_STATE = Go_to_stair");
				break;
				case Climb_to_first_stair:
					ROS_INFO("CW_Strategy_STATE = Climb_to_first_stair");
				break;
				case Climb_to_second_stair:
					ROS_INFO("CW_Strategy_STATE = Climb_to_second_stair");
				break;
				case Climb_to_third_stair:
					ROS_INFO("CW_Strategy_STATE = Climb_to_third_stair");
				break;
				case Climb_next:
					ROS_INFO("CW_Strategy_STATE = Climb_next");
				break;
				case On_stair_top:
					ROS_INFO("CW_Strategy_STATE = On_stair_top");
				break;
			}
			switch (liftandcarryinfo->BodyState) 
			{
				case Spr:
					ROS_INFO("BodyState = Spr\n");
				break;
				case BigLeftRotation:
					ROS_INFO("BodyState = BigLeftRotation\n");
				break;
				case BigRightRotation:
					ROS_INFO("BodyState = BigRightRotation\n");
				break;
				case BigFront:
					ROS_INFO("BodyState = BigFront\n");
				break;
				case SmallFront:
					ROS_INFO("BodyState = SmallFront\n");
				break;
				case SmallLeftRotation:
					ROS_INFO("BodyState = SmallLeftRotation\n");
				break;
				case SmallRightRotation:
					ROS_INFO("BodyState = SmallRightRotation\n");
				break;
				case Zero:
					ROS_INFO("BodyState = Zero\n");
				break;
			}
		break;
	}
}

void KidsizeStrategy::StrategyClassify()
{
	int i,h,leftwood,rightwood,foothalfdistance,l=0,leftdistance,rightdistance;
	int prevent010[liftandcarryinfo->RightFoot.XMax-liftandcarryinfo->LeftFoot.XMin];
	switch(liftandcarryinfo->StrategyState)
	{
		case CalSlope:
			if (liftandcarryinfo->WhichStair >=  Stair_6)
				liftandcarryinfo->BodyState = Zero;
			else if (liftandcarryinfo->LeftSlopeFlag && liftandcarryinfo->CenterForSlope && liftandcarryinfo->RightSlopeFlag)//111
			{
				//---------------------------------up---------------------------------//
				if (liftandcarryinfo->RobotUp)			
				{
					if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance)) //111
						liftandcarryinfo->BodyState = Up;
					else if ((liftandcarryinfo->LeftFootDistance > 60) && (liftandcarryinfo->CenterFootDistance > 60) && (liftandcarryinfo->RightFootDistance > 60))
						liftandcarryinfo->BodyState = Spr;
					else if ((liftandcarryinfo->LeftFootDistance > 40) && (liftandcarryinfo->CenterFootDistance > 40) && (liftandcarryinfo->RightFootDistance > 40))//000
						liftandcarryinfo->BodyState = BigFront;//大直走
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance ) && (liftandcarryinfo->CenterFootDistance > SureUpDistance ) && (liftandcarryinfo->RightFootDistance > SureUpDistance ))//000
						liftandcarryinfo->BodyState = SmallFront;//小直走
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance))		//011
					{
						if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + 3) && (liftandcarryinfo->LeftCenterFootDistance <= SureUpDistance))
							liftandcarryinfo->BodyState = Up;
						else
						{
							liftandcarryinfo->BodyState = SmallRightShift;
							ShiftDanger();
							if(shiftdanger)
								liftandcarryinfo->BodyState = SmallRightRotation;
						}
					}
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance) && (liftandcarryinfo->CenterFootDistance > SureUpDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance))		//101
					{
						if(liftandcarryinfo->LeftCenterFootDistance <= SureUpDistance + 7 && liftandcarryinfo->RightCenterFootDistance <= SureUpDistance + 7)
							liftandcarryinfo->BodyState = Up;
						else if (liftandcarryinfo->LeftCenterFootDistance > 17  && liftandcarryinfo->RightCenterFootDistance <= 17)//01
							liftandcarryinfo->BodyState = SmallRightRotation;
						else if (liftandcarryinfo->LeftCenterFootDistance <= 17 && liftandcarryinfo->RightCenterFootDistance > 17)//10
							liftandcarryinfo->BodyState = SmallLeftRotation;
						else
							liftandcarryinfo->BodyState = SmallFront;
					}
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance) && (liftandcarryinfo->CenterFootDistance > SureUpDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance))		//001
					{
						if ((liftandcarryinfo->LeftFootDistance - liftandcarryinfo->CenterFootDistance > 16)||((liftandcarryinfo->CenterFootDistance > SureUpDistance + 12) ))
							liftandcarryinfo->BodyState = BigRightRotation;
						else if(liftandcarryinfo->LeftFootDistance - liftandcarryinfo->CenterFootDistance > 6)
							liftandcarryinfo->BodyState = SmallRightRotation;
						else
						{
							if (liftandcarryinfo->CenterFootDistance - liftandcarryinfo->RightFootDistance > 15 && liftandcarryinfo->LeftFootDistance - liftandcarryinfo->RightFootDistance > 30)
								liftandcarryinfo->BodyState = BigRightRotation;
							else if (liftandcarryinfo->CenterFootDistance - liftandcarryinfo->RightFootDistance > 6 && liftandcarryinfo->LeftFootDistance - liftandcarryinfo->RightFootDistance > 12)
								liftandcarryinfo->BodyState = SmallRightRotation;
							else
								liftandcarryinfo->BodyState = SmallFront;
						}
					}
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance) && (liftandcarryinfo->RightFootDistance > SureUpDistance))		//110
					{
						if ((liftandcarryinfo->RightFootDistance <= SureUpDistance + 10) && (liftandcarryinfo->RightCenterFootDistance <= SureUpDistance + 6))
							liftandcarryinfo->BodyState = Up;
						else
						{
							liftandcarryinfo->BodyState = SmallLeftShift;
							ShiftDanger();
							if(shiftdanger)
								liftandcarryinfo->BodyState = SmallLeftRotation;
						}
					}
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance) && (liftandcarryinfo->RightFootDistance > SureUpDistance))		//010
					{
						if((liftandcarryinfo->LeftFootDistance <= SureUpDistance + 3) && (liftandcarryinfo->RightFootDistance <= SureUpDistance + 3))
							liftandcarryinfo->BodyState = Up;
						else if((liftandcarryinfo->LeftCenterFootDistance <= SureUpDistance + 2) && (liftandcarryinfo->RightCenterFootDistance <= SureUpDistance + 2))
							liftandcarryinfo->BodyState = Up;
						else
						{
							//-----------------找出010最低點-----------------//
							for(l=liftandcarryinfo->LeftFoot.XMin;l<liftandcarryinfo->RightFoot.XMax;l++)
							{
								for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
								{
									if(liftandcarryinfo->WhichStair == Stair_0)
									{
										if( strategy_info->label_model[ ImageWidth * i + l ] == TrdColor )
										{
											prevent010[l-liftandcarryinfo->LeftFoot.XMin]=liftandcarryinfo->LeftFoot.YMax-i;
											break;
										}
									}
									else if(liftandcarryinfo->WhichStair == Stair_1)
									{
										if( strategy_info->label_model[ ImageWidth * i + l ] == SecColor )
										{
											prevent010[l-liftandcarryinfo->LeftFoot.XMin]=liftandcarryinfo->LeftFoot.YMax-i;
											break;
										}
									}
									else if(liftandcarryinfo->WhichStair == Stair_2)
									{
										if( strategy_info->label_model[ ImageWidth * i + l ] == TopColor )
										{
											prevent010[l-liftandcarryinfo->LeftFoot.XMin]=liftandcarryinfo->LeftFoot.YMax-i;
											break;
										}
									}
								}
							}
							for(i=1;i<liftandcarryinfo->RightFoot.XMax-liftandcarryinfo->LeftFoot.XMin-1;i++)
							{
								if(prevent010[i]<prevent010[i-1])
									l=i+liftandcarryinfo->LeftFoot.XMin;	//l為最低點x座標
							}
							//--------------------------------------------//
							if(l<(liftandcarryinfo->LeftFoot.XMax+liftandcarryinfo->LeftFoot.XMin)/2)
							{
								liftandcarryinfo->BodyState = RightShift;
								ShiftDanger();
								if(shiftdanger)
									liftandcarryinfo->BodyState = BigLeftRotation;
							}
							else if(l>(liftandcarryinfo->RightFoot.XMax+liftandcarryinfo->RightFoot.XMin)/2)
							{
								liftandcarryinfo->BodyState = LeftShift;
								ShiftDanger();
								if(shiftdanger)
									liftandcarryinfo->BodyState = BigRightRotation;
							}
							else
							{
								foothalfdistance=(liftandcarryinfo->LeftFoot.XMax-liftandcarryinfo->LeftFoot.XMin)/2;
								leftslope=(((double)(liftandcarryinfo->LeftFootDistance-liftandcarryinfo->LeftCenterFootDistance)/(double)foothalfdistance)-((double)(liftandcarryinfo->LeftCenterFootDistance-prevent010[l])/(double)(l-((liftandcarryinfo->LeftFoot.XMax+liftandcarryinfo->LeftFoot.XMin)/2))));
								rightslope=(((double)(liftandcarryinfo->RightFootDistance-liftandcarryinfo->RightCenterFootDistance)/(double)foothalfdistance)-((double)(liftandcarryinfo->RightCenterFootDistance-prevent010[l])/(double)(((liftandcarryinfo->RightFoot.XMax+liftandcarryinfo->RightFoot.XMin)/2)-l)));
								if(abs(leftslope)>abs(rightslope))
								{
									liftandcarryinfo->BodyState = RightShift;
									ShiftDanger();
									if(shiftdanger)
										liftandcarryinfo->BodyState = BigLeftRotation;
								}
								else if(abs(leftslope)<abs(rightslope))
								{
									liftandcarryinfo->BodyState = LeftShift;
									ShiftDanger();
									if(shiftdanger)
										liftandcarryinfo->BodyState = BigRightRotation;
								}
								else
								{
									if(liftandcarryinfo->LeftFootDistance>liftandcarryinfo->RightFootDistance)
									{
										liftandcarryinfo->BodyState = RightShift;
										ShiftDanger();
										if(shiftdanger)
											liftandcarryinfo->BodyState = BigLeftRotation;
									}
									else
									{
										liftandcarryinfo->BodyState = LeftShift;
										ShiftDanger();
										if(shiftdanger)
											liftandcarryinfo->BodyState = BigRightRotation;
									}		
								}
							}
						}
					}
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance) && (liftandcarryinfo->CenterFootDistance > SureUpDistance) && (liftandcarryinfo->RightFootDistance > SureUpDistance))		//100
					{
						if ((liftandcarryinfo->RightFootDistance - liftandcarryinfo->CenterFootDistance > 16)||((liftandcarryinfo->CenterFootDistance > SureUpDistance + 15)))
							liftandcarryinfo->BodyState = BigLeftRotation;
						else if(liftandcarryinfo->RightFootDistance - liftandcarryinfo->CenterFootDistance > 6)
							liftandcarryinfo->BodyState = SmallLeftRotation;
						else
						{
							if (liftandcarryinfo->CenterFootDistance - liftandcarryinfo->LeftFootDistance > 15 && liftandcarryinfo->RightFootDistance - liftandcarryinfo->LeftFootDistance > 30)
								liftandcarryinfo->BodyState = BigLeftRotation;
							else if (liftandcarryinfo->CenterFootDistance - liftandcarryinfo->LeftFootDistance > 6 && liftandcarryinfo->RightFootDistance - liftandcarryinfo->LeftFootDistance > 12)
								liftandcarryinfo->BodyState = SmallLeftRotation;
							else
								liftandcarryinfo->BodyState = SmallFront;
						}
					}
					else
						liftandcarryinfo->BodyState = SmallFront;
				}
				//--------------------------------------------------------------------//
				//--------------------------------down--------------------------------//
				else
				{
					if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance + ErrorDownDistance))
						liftandcarryinfo->BodyState = Up;
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance + ErrorDownDistance))		//011
					{
						if (liftandcarryinfo->LeftFootDistance <= SureUpDistance + ErrorDownDistance + 5 )
							liftandcarryinfo->BodyState = Up;
						else
							liftandcarryinfo->BodyState = SmallRightRotation;
					}
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->CenterFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance + ErrorDownDistance))		//101
					{
						if (liftandcarryinfo->CenterFootDistance <= (SureUpDistance + ErrorDownDistance + 5) && liftandcarryinfo->LeftCenterFootDistance <= (SureUpDistance + ErrorDownDistance) && liftandcarryinfo->RightCenterFootDistance <=(SureUpDistance + ErrorDownDistance))
							liftandcarryinfo->BodyState = Up;
						else//
						{
							if (liftandcarryinfo->LeftCenterFootDistance <= SureUpDistance && liftandcarryinfo->RightCenterFootDistance <= SureUpDistance)//11
								liftandcarryinfo->BodyState = Up;
							else if (liftandcarryinfo->LeftCenterFootDistance > SureUpDistance && liftandcarryinfo->RightCenterFootDistance <= SureUpDistance)//01
							{
								liftandcarryinfo->BodyState = SmallRightShift;
								ShiftDanger();
								if(shiftdanger)
									liftandcarryinfo->BodyState = SmallRightRotation;
							}
							else if (liftandcarryinfo->LeftCenterFootDistance <= SureUpDistance && liftandcarryinfo->RightCenterFootDistance > SureUpDistance)//10
							{
								liftandcarryinfo->BodyState = SmallLeftShift;
								ShiftDanger();
								if(shiftdanger)
									liftandcarryinfo->BodyState = SmallLeftRotation;
							}
							else
								liftandcarryinfo->BodyState = SmallFront;
						}//
					}
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->CenterFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance <= SureUpDistance + ErrorDownDistance))		//001
					{
						if((liftandcarryinfo->LeftFootDistance >= SureUpDistance + ErrorDownDistance + 5) && (liftandcarryinfo->CenterFootDistance >= SureUpDistance + ErrorDownDistance + 5))
						{
							if((abs(liftandcarryinfo->LeftFootDistance - liftandcarryinfo->LeftCenterFootDistance) <=  6) && (abs(liftandcarryinfo->LeftFootDistance - liftandcarryinfo->CenterFootDistance) <= 6))
								liftandcarryinfo->BodyState = SmallLeftShift;
						}
						else
						{
							if (abs(liftandcarryinfo->LeftFootDistance - liftandcarryinfo->CenterFootDistance) > 16)
								liftandcarryinfo->BodyState = BigRightRotation;
							else if(abs(liftandcarryinfo->LeftFootDistance - liftandcarryinfo->CenterFootDistance) > 6)
								liftandcarryinfo->BodyState = SmallRightRotation;
							else
							{
								if ((liftandcarryinfo->RightFootDistance > SureUpDistance + ErrorDownDistance - 4))
								{
									liftandcarryinfo->BodyState = SmallLeftShift;
									ShiftDanger();
									if(shiftdanger)
										liftandcarryinfo->BodyState = SmallLeftRotation;
								}
								else
									liftandcarryinfo->BodyState = SmallFront;
							}
						}
					}
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + ErrorDownDistance ) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance > SureUpDistance + ErrorDownDistance))		//110
					{
						if (liftandcarryinfo->RightFootDistance <= SureUpDistance + ErrorDownDistance + 5)
							liftandcarryinfo->BodyState = Up;
						else
							liftandcarryinfo->BodyState = SmallLeftRotation;
					}
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->CenterFootDistance <= SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance > SureUpDistance + ErrorDownDistance))		//010
					{
						if (liftandcarryinfo->LeftFootDistance <= (SureUpDistance + 5) && liftandcarryinfo->RightFootDistance <= (SureUpDistance + 5))
							liftandcarryinfo->BodyState = Up;
						else
						{
							if (liftandcarryinfo->LeftCenterFootDistance <= SureUpDistance && liftandcarryinfo->RightCenterFootDistance <= SureUpDistance)//11
							{
								if (liftandcarryinfo->LeftFootDistance > (SureUpDistance + 8) && liftandcarryinfo->RightFootDistance <= (SureUpDistance + 8))
								{
									liftandcarryinfo->BodyState = SmallRightShift;
									ShiftDanger();
									if(shiftdanger)
										liftandcarryinfo->BodyState = SmallRightRotation;
								}
								else if (liftandcarryinfo->LeftFootDistance <= (SureUpDistance + 8) && liftandcarryinfo->RightFootDistance > (SureUpDistance + 8))
								{
									liftandcarryinfo->BodyState = SmallLeftShift;
									ShiftDanger();
									if(shiftdanger)
										liftandcarryinfo->BodyState = SmallLeftRotation;
								}
								else if (liftandcarryinfo->LeftFootDistance <= (SureUpDistance + 9) && liftandcarryinfo->RightFootDistance <= (SureUpDistance + 9))
									liftandcarryinfo->BodyState = Up;
							}
							else if (liftandcarryinfo->LeftCenterFootDistance > SureUpDistance && liftandcarryinfo->RightCenterFootDistance <= SureUpDistance)//01
							{
								liftandcarryinfo->BodyState = SmallRightShift;
								ShiftDanger();
								if(shiftdanger)
									liftandcarryinfo->BodyState = SmallRightRotation;
							}
							else if (liftandcarryinfo->LeftCenterFootDistance <= SureUpDistance && liftandcarryinfo->RightCenterFootDistance > SureUpDistance)//10
							{
								liftandcarryinfo->BodyState = SmallLeftShift;
								ShiftDanger();
								if(shiftdanger)
									liftandcarryinfo->BodyState = SmallLeftRotation;
							}
							else
								liftandcarryinfo->BodyState = SmallFront;
						}
					}
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->CenterFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance > SureUpDistance + ErrorDownDistance))		//100
					{
						if((liftandcarryinfo->RightFootDistance >= SureUpDistance + ErrorDownDistance + 5) && (liftandcarryinfo->CenterFootDistance >= SureUpDistance + ErrorDownDistance + 5))		//001ready
						{
							if((abs(liftandcarryinfo->RightFootDistance - liftandcarryinfo->RightCenterFootDistance) <=  6) && (abs(liftandcarryinfo->RightFootDistance - liftandcarryinfo->CenterFootDistance) <= 6))
								liftandcarryinfo->BodyState = SmallRightShift;
						}
						else
						{
							if (abs(liftandcarryinfo->RightFootDistance - liftandcarryinfo->CenterFootDistance) > 16 )
								liftandcarryinfo->BodyState = BigLeftRotation;
							else if(abs(liftandcarryinfo->RightFootDistance - liftandcarryinfo->CenterFootDistance) > 6)
								liftandcarryinfo->BodyState = SmallLeftRotation;
							else
							{
								if ((liftandcarryinfo->LeftFootDistance > SureUpDistance + ErrorDownDistance - 4))
								{
									liftandcarryinfo->BodyState = SmallRightShift;
									ShiftDanger();
									if(shiftdanger)
										liftandcarryinfo->BodyState = SmallRightRotation;
								}
								else
									liftandcarryinfo->BodyState = SmallFront;
							}
						}
					}
					else if ((liftandcarryinfo->LeftFootDistance > 60) && (liftandcarryinfo->CenterFootDistance > 60) && (liftandcarryinfo->RightFootDistance > 60))
						liftandcarryinfo->BodyState = Spr;
					else if ((liftandcarryinfo->LeftFootDistance > 40) && (liftandcarryinfo->CenterFootDistance > 40) && (liftandcarryinfo->RightFootDistance > 40))//000
						liftandcarryinfo->BodyState = BigFront;
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->CenterFootDistance > SureUpDistance + ErrorDownDistance) && (liftandcarryinfo->RightFootDistance > SureUpDistance + ErrorDownDistance))//000
						liftandcarryinfo->BodyState = SmallFront;
				}
				//--------------------------------------------------------------------//
			}
			else if (!liftandcarryinfo->LeftSlopeFlag && !liftandcarryinfo->CenterSlopeFlag && !liftandcarryinfo->RightSlopeFlag)//000
				liftandcarryinfo->BodyState = Spr;
			else if (!liftandcarryinfo->LeftSlopeFlag && liftandcarryinfo->CenterSlopeFlag && !liftandcarryinfo->RightSlopeFlag)//010
			{
				if (liftandcarryinfo->CenterFootDistance >= SureUpDistance + 20)
					liftandcarryinfo->BodyState = SmallFront;
				else
				{
					for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
					{
						if(liftandcarryinfo->WhichStair == Stair_0 || liftandcarryinfo->WhichStair == Stair_5)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == TrdColor )
							{
								break;
							}
						}
						else if(liftandcarryinfo->WhichStair == Stair_1 || liftandcarryinfo->WhichStair == Stair_4)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == SecColor )
							{
								break;
							}
						}
						else if(liftandcarryinfo->WhichStair == Stair_2 || liftandcarryinfo->WhichStair == Stair_3)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->LeftFoot.XMin ] == TopColor )
							{
								break;
							}
						}
					}
					leftdistance=liftandcarryinfo->LeftFoot.YMax-i;
					for(i=liftandcarryinfo->LeftFoot.YMax;i>0;i--)
					{
						if(liftandcarryinfo->WhichStair == Stair_0 || liftandcarryinfo->WhichStair == Stair_5)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax  ] == TrdColor )
							{
								break;
							}
						}
						else if(liftandcarryinfo->WhichStair == Stair_1 || liftandcarryinfo->WhichStair == Stair_4)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax ] == SecColor )
							{
								break;
							}
						}
						else if(liftandcarryinfo->WhichStair == Stair_2 || liftandcarryinfo->WhichStair == Stair_3)
						{
							if( strategy_info->label_model[ ImageWidth * i + liftandcarryinfo->RightFoot.XMax ] == TopColor )
							{
								break;
							}
						}
					}
					rightdistance=liftandcarryinfo->LeftFoot.YMax-i;
					if(leftdistance>rightdistance)
					{
						liftandcarryinfo->BodyState = RightShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigLeftRotation;
					}
					else if(leftdistance<rightdistance)
					{
						liftandcarryinfo->BodyState = LeftShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigRightRotation;
					}
					else
					{
						liftandcarryinfo->BodyState = LeftShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigRightRotation;
					}
				}
			}
			else if(liftandcarryinfo->LeftSlopeFlag && liftandcarryinfo->CenterSlopeFlag && !liftandcarryinfo->RightSlopeFlag)//110
			{
				if (liftandcarryinfo->RobotUp)
				{
					if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 25 && liftandcarryinfo->LeftFootDistance > SureUpDistance + 25)
						liftandcarryinfo->BodyState = BigFront;
					else if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 15 && liftandcarryinfo->LeftFootDistance > SureUpDistance + 15)
						liftandcarryinfo->BodyState = SmallFront;
					else if ((liftandcarryinfo->CenterFootDistance - liftandcarryinfo->LeftFootDistance) > 15)
						liftandcarryinfo->BodyState = BigLeftRotation;
					else if ((liftandcarryinfo->CenterFootDistance - liftandcarryinfo->LeftFootDistance) > 6)
						liftandcarryinfo->BodyState = SmallLeftRotation;
					else
					{
						liftandcarryinfo->BodyState = LeftShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigLeftRotation;
					}
				}
				else
				{
					if (liftandcarryinfo->CenterFootDistance > 60 && liftandcarryinfo->LeftFootDistance > 60)
						liftandcarryinfo->BodyState = Spr;
					else if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 20 && liftandcarryinfo->LeftFootDistance > SureUpDistance + 20)
						liftandcarryinfo->BodyState = BigFront;
					else if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 10 && liftandcarryinfo->LeftFootDistance > SureUpDistance + 10)
						liftandcarryinfo->BodyState = SmallFront;
					else if ((liftandcarryinfo->CenterFootDistance - liftandcarryinfo->LeftFootDistance) > 10)
						liftandcarryinfo->BodyState = BigLeftRotation;
					else
					{
						liftandcarryinfo->BodyState = LeftShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigLeftRotation;
					}
				}
			}
			else if(!liftandcarryinfo->LeftSlopeFlag && liftandcarryinfo->CenterSlopeFlag && liftandcarryinfo->RightSlopeFlag)//011
			{
				if (liftandcarryinfo->RobotUp)
				{
					if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 25 && liftandcarryinfo->RightFootDistance > SureUpDistance + 25)
						liftandcarryinfo->BodyState = BigFront;
					else if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 15 && liftandcarryinfo->RightFootDistance > SureUpDistance + 15)
						liftandcarryinfo->BodyState = SmallFront;
					else if ((liftandcarryinfo->CenterFootDistance - liftandcarryinfo->RightFootDistance) > 15)
						liftandcarryinfo->BodyState = BigRightRotation;
					else if ((liftandcarryinfo->CenterFootDistance - liftandcarryinfo->RightFootDistance) > 6)
						liftandcarryinfo->BodyState = SmallRightRotation;
					else
					{
						liftandcarryinfo->BodyState = RightShift;
						ShiftDanger();
						//AvoidDrop();
						if(shiftdanger)
						{
							liftandcarryinfo->BodyState = BigRightRotation;
						}
						//else if(avoiddrop)
						//{
							//liftandcarryinfo->BodyState = LeftShift;
						//}
					}		
				}
				else
				{
					if (liftandcarryinfo->CenterFootDistance > 60 && liftandcarryinfo->RightFootDistance > 60)
						liftandcarryinfo->BodyState = Spr;
					else if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 20 && liftandcarryinfo->RightFootDistance > SureUpDistance + 20)
						liftandcarryinfo->BodyState = BigFront;
					else if (liftandcarryinfo->CenterFootDistance > SureUpDistance + 10 && liftandcarryinfo->RightFootDistance > SureUpDistance + 10)
						liftandcarryinfo->BodyState = SmallFront;
					else if ((liftandcarryinfo->CenterFootDistance - liftandcarryinfo->RightFootDistance) > 10)
						liftandcarryinfo->BodyState = BigRightRotation;
					else
					{
						liftandcarryinfo->BodyState = RightShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigRightRotation;
					}
				}
			}
			else if (liftandcarryinfo->LeftSlopeFlag && !liftandcarryinfo->CenterSlopeFlag &&!liftandcarryinfo->RightSlopeFlag)//100
			{
				if (liftandcarryinfo->RobotUp)
				{
					if (liftandcarryinfo->LeftFootDistance > SureUpDistance + 25 && liftandcarryinfo->LeftCenterFootDistance > SureUpDistance + 25)
						liftandcarryinfo->BodyState = BigFront;
					else if (liftandcarryinfo->LeftFootDistance > SureUpDistance + 15 && liftandcarryinfo->LeftCenterFootDistance > SureUpDistance + 15)
						liftandcarryinfo->BodyState = SmallFront;
					else
					{
						liftandcarryinfo->BodyState = LeftShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigLeftRotation;
					}		
				}
				else
				{
					if (liftandcarryinfo->LeftFootDistance <= SureUpDistance + ErrorDownDistance )
						liftandcarryinfo->BodyState = BigLeftRotation;
					else if (liftandcarryinfo->LeftFootDistance <= SureUpDistance + 12 + ErrorDownDistance )
						liftandcarryinfo->BodyState = SmallFront;
					else
						liftandcarryinfo->BodyState = SmallLeftRotation;
				}
			}
			else if (!liftandcarryinfo->LeftSlopeFlag  && !liftandcarryinfo->CenterSlopeFlag && liftandcarryinfo->RightSlopeFlag)//001
			{
				if (liftandcarryinfo->RobotUp)
				{
					if (liftandcarryinfo->RightFootDistance > SureUpDistance + 25 && liftandcarryinfo->RightCenterFootDistance > SureUpDistance + 25)
						liftandcarryinfo->BodyState = BigFront;
					else if (liftandcarryinfo->RightFootDistance > SureUpDistance + 15 && liftandcarryinfo->RightCenterFootDistance > SureUpDistance + 15)
						liftandcarryinfo->BodyState = SmallFront;
					else
					{
						liftandcarryinfo->BodyState = RightShift;
						ShiftDanger();
						if(shiftdanger)
							liftandcarryinfo->BodyState = BigRightRotation;
					}
				}
				else
				{
					if(liftandcarryinfo->RightFootDistance  <= SureUpDistance + ErrorDownDistance)
						liftandcarryinfo->BodyState = BigRightRotation;
					else if (liftandcarryinfo->RightFootDistance <= SureUpDistance + 12 + ErrorDownDistance)
						liftandcarryinfo->BodyState = SmallFront;
					else
						liftandcarryinfo->BodyState = SmallRightRotation;
				}
			}
			else if (liftandcarryinfo->LeftSlopeFlag  && !liftandcarryinfo->CenterSlopeFlag && liftandcarryinfo->RightSlopeFlag) //101
			{
				if (liftandcarryinfo->RobotUp)
				{
					if (liftandcarryinfo->LeftFootDistance <= (SureUpDistance-8) && liftandcarryinfo->RightFootDistance <= (SureUpDistance-8))
						liftandcarryinfo->BodyState = Up;
					else
						liftandcarryinfo->BodyState = SmallFront;
				}
				else
				{
					if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + 10) && (liftandcarryinfo->RightFootDistance <= SureUpDistance +10) && (liftandcarryinfo->RightFootDistance>=liftandcarryinfo->LeftFootDistance))
						liftandcarryinfo->BodyState = BigRightRotation;
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + 10) && (liftandcarryinfo->RightFootDistance <= SureUpDistance +10) && (liftandcarryinfo->RightFootDistance<=liftandcarryinfo->LeftFootDistance))
						liftandcarryinfo->BodyState = BigLeftRotation;
					else if ((liftandcarryinfo->LeftFootDistance <= SureUpDistance + 5) && (liftandcarryinfo->RightFootDistance > SureUpDistance + 20))
						liftandcarryinfo->BodyState = BigRightRotation;
					else if ((liftandcarryinfo->LeftFootDistance > SureUpDistance + 20) && (liftandcarryinfo->RightFootDistance <= SureUpDistance + 5))
						liftandcarryinfo->BodyState = BigLeftRotation;
					else
						liftandcarryinfo->BodyState = SmallFront;
				}
			}
		break;
		case Finish:
		break;
	}
}

void KidsizeStrategy::initparameterpath()
{
	while(parameter_path == "N")
	{
		parameter_path = tool->getPackagePath("strategy");
	}
	printf("parameter_path is %s\n", parameter_path.c_str());
}

void KidsizeStrategy::Loadparameter()
{
	fstream fin;
	char path[200];
	strcpy(path, parameter_path.c_str());
	strcat(path, "/SpartanRace_Parameter.ini");
	string sTmp;
	char line[100];
	fin.open(path, ios::in);
	try
	{
		st0_SmallFrontX = tool->readvalue(fin, "st0_SmallFrontX", 1);
		st0_SmallFrontY = tool->readvalue(fin, "st0_SmallFrontY", 1);
		st0_SmallFrontTha = tool->readvalue(fin, "st0_SmallFrontTha", 1);
		st0_SmallFrontimu = tool->readvalue(fin, "st0_SmallFrontimu", 1);
		st0_BigFrontX = tool->readvalue(fin, "st0_BigFrontX", 1);
		st0_BigFrontY = tool->readvalue(fin, "st0_BigFrontY", 1);
		st0_BigFrontTha = tool->readvalue(fin, "st0_BigFrontTha", 1);
		st0_BigFrontimu = tool->readvalue(fin, "st0_BigFrontimu", 1);
		st0_SprX = tool->readvalue(fin, "st0_SprX", 1);
		st0_SprY = tool->readvalue(fin, "st0_SprY", 1);
		st0_SprTha = tool->readvalue(fin, "st0_SprTha", 1);
		st0_Sprimu = tool->readvalue(fin, "st0_Sprimu", 1);
		SmallFrontX = tool->readvalue(fin, "SmallFrontX", 1);
		SmallFrontY = tool->readvalue(fin, "SmallFrontY", 1);
		SmallFrontTha = tool->readvalue(fin, "SmallFrontTha", 1);
		SmallFrontimu = tool->readvalue(fin, "SmallFrontimu", 1);
		SmallFrontDelay = tool->readvalue(fin, "SmallFrontDelay", 1);
		BigFrontX = tool->readvalue(fin, "BigFrontX", 1);
		BigFrontY = tool->readvalue(fin, "BigFrontY", 1);
		BigFrontTha = tool->readvalue(fin, "BigFrontTha", 1);
		BigFrontimu = tool->readvalue(fin, "BigFrontimu", 1);
		BigFrontDelay = tool->readvalue(fin, "BigFrontDelay", 1);
		SprX = tool->readvalue(fin, "SprX", 1);
		SprY = tool->readvalue(fin, "SprY", 1);
		SprTha = tool->readvalue(fin, "SprTha", 1);
		Sprimu = tool->readvalue(fin, "Sprimu", 1);
		SprDelay = tool->readvalue(fin, "SprDelay", 1);
		LeftShiftX = tool->readvalue(fin, "LeftShiftX", 1);
		LeftShiftY = tool->readvalue(fin, "LeftShiftY", 1);
		LeftShiftTha = tool->readvalue(fin, "LeftShiftTha", 1);
		LeftShiftimu = tool->readvalue(fin, "LeftShiftimu", 1);
		LeftShiftDelay = tool->readvalue(fin, "LeftShiftDelay", 1);
		RightShiftX = tool->readvalue(fin, "RightShiftX", 1);
		RightShiftY = tool->readvalue(fin, "RightShiftY", 1);
		RightShiftTha = tool->readvalue(fin, "RightShiftTha", 1);
		RightShiftimu = tool->readvalue(fin, "RightShiftimu", 1);
		RightShiftDelay = tool->readvalue(fin, "RightShiftDelay", 1);
		SmallLeftShiftX = tool->readvalue(fin, "SmallLeftShiftX", 1);
		SmallLeftShiftY = tool->readvalue(fin, "SmallLeftShiftY", 1);
		SmallLeftShiftTha = tool->readvalue(fin, "SmallLeftShiftTha", 1);
		SmallLeftShiftimu = tool->readvalue(fin, "SmallLeftShiftimu", 1);
		SmallLeftShiftDelay = tool->readvalue(fin, "SmallLeftShiftDelay", 1);
		SmallRightShiftX = tool->readvalue(fin, "SmallRightShiftX", 1);
		SmallRightShiftY = tool->readvalue(fin, "SmallRightShiftY", 1);
		SmallRightShiftTha = tool->readvalue(fin, "SmallRightShiftTha", 1);
		SmallRightShiftimu = tool->readvalue(fin, "SmallRightShiftimu", 1);
		SmallRightShiftDelay = tool->readvalue(fin, "SmallRightShiftDelay", 1);
		SmallLeftRotationX = tool->readvalue(fin, "SmallLeftRotationX", 1);
		SmallLeftRotationY = tool->readvalue(fin, "SmallLeftRotationY", 1);
		SmallLeftRotationTha = tool->readvalue(fin, "SmallLeftRotationTha", 1);
		SmallLeftRotationimu = tool->readvalue(fin, "SmallLeftRotationimu", 1);
		SmallLeftRotationDelay = tool->readvalue(fin, "SmallLeftRotationDelay", 1);
		SmallRightRotationX = tool->readvalue(fin, "SmallRightRotationX", 1);
		SmallRightRotationY = tool->readvalue(fin, "SmallRightRotationY", 1);
		SmallRightRotationTha = tool->readvalue(fin, "SmallRightRotationTha", 1);
		SmallRightRotationimu = tool->readvalue(fin, "SmallRightRotationimu", 1);
		SmallRightRotationDelay = tool->readvalue(fin, "SmallRightRotationDelay", 1);
		BigLeftRotationX = tool->readvalue(fin, "BigLeftRotationX", 1);
		BigLeftRotationY = tool->readvalue(fin, "BigLeftRotationY", 1);
		BigLeftRotationTha = tool->readvalue(fin, "BigLeftRotationTha", 1);
		BigLeftRotationimu = tool->readvalue(fin, "BigLeftRotationimu", 1);
		BigLeftRotationDelay = tool->readvalue(fin, "BigLeftRotationDelay", 1);
		BigRightRotationX = tool->readvalue(fin, "BigRightRotationX", 1);
		BigRightRotationY = tool->readvalue(fin, "BigRightRotationY", 1);
		BigRightRotationTha = tool->readvalue(fin, "BigRightRotationTha", 1);
		BigRightRotationimu = tool->readvalue(fin, "BigRightRotationimu", 1);
		BigRightRotationDelay = tool->readvalue(fin, "BigRightRotationDelay", 1);
		Hole_LeftX = tool->readvalue(fin, "Hole_LeftX", 1);
		Hole_LeftY = tool->readvalue(fin, "Hole_LeftY", 1);
		Hole_LeftTha = tool->readvalue(fin, "Hole_LeftTha", 1);
		Hole_Leftimu = tool->readvalue(fin, "Hole_Leftimu", 1);
		Hole_RightX = tool->readvalue(fin, "Hole_RightX", 1);
		Hole_RightY = tool->readvalue(fin, "Hole_RightY", 1);
		Hole_RightTha = tool->readvalue(fin, "Hole_RightTha", 1);
		Hole_Rightimu = tool->readvalue(fin, "Hole_Rightimu", 1);
		Hole_Delay = tool->readvalue(fin, "Hole_Delay", 1);
		LC_StepX = tool->readvalue(fin, "LC_StepX", 1);
		LC_StepY = tool->readvalue(fin, "LC_StepY", 1);
		LC_StepTha = tool->readvalue(fin, "LC_StepTha", 1);
		LC_Stepimu = tool->readvalue(fin, "LC_Stepimu", 1);
		LC_StepDelay = tool->readvalue(fin, "LC_StepDelay", 1);
		LC_DownX = tool->readvalue(fin, "LC_DownX", 1);
		LC_DownY = tool->readvalue(fin, "LC_DownY", 1);
		LC_DownTha = tool->readvalue(fin, "LC_DownTha", 1);
		LC_Downimu = tool->readvalue(fin, "LC_Downimu", 1);
		LC_DownDelay = tool->readvalue(fin, "LC_DownDelay", 1);
		HeadpositionX = tool->readvalue(fin, "HeadpositionX", 1);
		HeadpositionY = tool->readvalue(fin, "HeadpositionY", 1);
		SureUpDistance = tool->readvalue(fin, "SureUpDistance", 1);
		ErrorDownDistance = tool->readvalue(fin, "ErrorDownDistance", 1);
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

void KidsizeStrategy::Strategy_Select()
{
	if(liftandcarryinfo->Start)
	{
		if(strategy_info->DIOValue.Switch.D0 == false && strategy_info->DIOValue.Switch.D1 == false && strategy_info->DIOValue.Switch.D2 == false)
		{
			liftandcarryinfo->WhichStrategy = strategy_liftandcarry;
			liftandcarryinfo->WhichStair = Stair_0;
			liftandcarryinfo->RobotUp = true;
		}
		else if(strategy_info->DIOValue.Switch.D0 == true && strategy_info->DIOValue.Switch.D1 == false && strategy_info->DIOValue.Switch.D2 == false)
		{
			liftandcarryinfo->WhichStrategy = strategy_liftandcarry;
			liftandcarryinfo->WhichStair = Stair_1;
			liftandcarryinfo->RobotUp = true;
		}
		else if(strategy_info->DIOValue.Switch.D0 == false && strategy_info->DIOValue.Switch.D1 == true && strategy_info->DIOValue.Switch.D2 == false)
		{
			liftandcarryinfo->WhichStrategy = strategy_liftandcarry;
			liftandcarryinfo->WhichStair = Stair_2;
			liftandcarryinfo->RobotUp = true;
		}
		else if(strategy_info->DIOValue.Switch.D0 == false && strategy_info->DIOValue.Switch.D1 == false && strategy_info->DIOValue.Switch.D2 == true)
		{
			liftandcarryinfo->WhichStrategy = strategy_liftandcarry;
			liftandcarryinfo->WhichStair = Stair_3;
			liftandcarryinfo->RobotUp = false;
		}
		else if(strategy_info->DIOValue.Switch.D0 == true && strategy_info->DIOValue.Switch.D1 == false && strategy_info->DIOValue.Switch.D2 == true)
		{
			liftandcarryinfo->WhichStrategy = strategy_liftandcarry;
			liftandcarryinfo->WhichStair = Stair_4;
			liftandcarryinfo->RobotUp = false;
		}
		else if(strategy_info->DIOValue.Switch.D0 == false && strategy_info->DIOValue.Switch.D1 == true && strategy_info->DIOValue.Switch.D2 == true)
		{
			liftandcarryinfo->WhichStrategy = strategy_liftandcarry;
			liftandcarryinfo->WhichStair = Stair_5;
			liftandcarryinfo->RobotUp = false;
		}
		else
			liftandcarryinfo->WhichStrategy = strategy_climbingwall;
			liftandcarryinfo->WhichStair = Stair_0;

	}
}

void KidsizeStrategy::Theta_Offset()	//記得在StrategyInitial中開啟IMUenable
{
	if(liftandcarryinfo->IMUenable)
	{
		current_IMUdata=strategy_info->getIMUValue().Yaw;
		IMUoffset=current_IMUdata-initial_IMUdata;
		if(IMUoffset < 0)
   		{
	        if(abs(IMUoffset) > 41)
	            Theta_offset = 6;
  	      	else if(abs(IMUoffset) > 28 && abs(IMUoffset) < 40)
   	        	Theta_offset = 4;         
	        else if(abs(IMUoffset) > 4 && abs(IMUoffset) < 27)
   	        	Theta_offset = 2;
    	    else
            	Theta_offset = 0;
    	}
	    else
   		{
        	if(abs(IMUoffset) > 41)
            	Theta_offset = -8;
        	else if(abs(IMUoffset) > 28 && abs(IMUoffset) < 40)
            	Theta_offset = -4;        
        	else if(abs(IMUoffset) > 4 && abs(IMUoffset) < 27)
            	Theta_offset = -2;
        	else
            	Theta_offset = 0;
    	}
	}
}

void KidsizeStrategy::Initial_IMUdata()
{
	if(liftandcarryinfo->IMUenable)
		initial_IMUdata=strategy_info->getIMUValue().Yaw;
}

void KidsizeStrategy::ShiftDanger()
{
	int h,i;
	int shiftdistance[51]={0};
	int smallshiftdistance[21]={0};
	shiftdanger=false;
	if(liftandcarryinfo->BodyState == RightShift)
	{
		for(i=liftandcarryinfo->RightFoot.XMax;i<=liftandcarryinfo->RightFoot.XMax + 50 ;i++)
		{
			for(h=liftandcarryinfo->LeftFoot.YMax; h > 0; h--)
			{
				if(liftandcarryinfo->WhichStair == Stair_0 || liftandcarryinfo->WhichStair == Stair_4)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == TrdColor )
					{
						shiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_1 || liftandcarryinfo->WhichStair == Stair_3)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == SecColor )
					{
						shiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_2)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == TopColor )
					{
						shiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_5)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == FileColor )
					{
						shiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
			}
		}
		for(i=0;i<52;i++)
		{
			if(shiftdistance[i]<SureUpDistance )
			{
				shiftdanger=true;
				break;
			}
		}
	}
	else if(liftandcarryinfo->BodyState == LeftShift)
	{
		for(i=liftandcarryinfo->LeftFoot.XMin - 50;i<=liftandcarryinfo->LeftFoot.XMin;i++)
		{
			for(h=liftandcarryinfo->LeftFoot.YMax; h > 0; h--)
			{
				if(liftandcarryinfo->WhichStair == Stair_0 || liftandcarryinfo->WhichStair == Stair_4)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == TrdColor )
					{
						shiftdistance[i-liftandcarryinfo->LeftFoot.XMin + 50]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_1 || liftandcarryinfo->WhichStair == Stair_3)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == SecColor )
					{
						shiftdistance[i-liftandcarryinfo->LeftFoot.XMin + 50]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_2)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == TopColor )
					{
						shiftdistance[i-liftandcarryinfo->LeftFoot.XMin + 50]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_5)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == FileColor )
					{
						shiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
			}
		}
		for(i=0;i<52;i++)
		{
			if(shiftdistance[i]<SureUpDistance )
			{
				shiftdanger=true;
				break;
			}
		}
	}
	else if(liftandcarryinfo->BodyState == SmallLeftShift)
	{
		for(i=liftandcarryinfo->LeftFoot.XMin - 20;i<=liftandcarryinfo->LeftFoot.XMin;i++)
		{
			for(h=liftandcarryinfo->LeftFoot.YMax; h > 0; h--)
			{
				if(liftandcarryinfo->WhichStair == Stair_0 || liftandcarryinfo->WhichStair == Stair_4)
				{
					if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin - 20 ] == TrdColor )
					{
						smallshiftdistance[i-liftandcarryinfo->LeftFoot.XMin + 20]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_1 || liftandcarryinfo->WhichStair == Stair_3)
				{
					if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin - 20 ] == SecColor )
					{
						smallshiftdistance[i-liftandcarryinfo->LeftFoot.XMin + 20]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_2)
				{
					if( strategy_info->label_model[ ImageWidth * h + liftandcarryinfo->LeftFoot.XMin - 20 ] == TopColor )
					{
						smallshiftdistance[i-liftandcarryinfo->LeftFoot.XMin + 20]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_5)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == FileColor )
					{
						shiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
			}
		}
		for(i=0;i<21;i++)
		{
			if(smallshiftdistance[i]<SureUpDistance )
			{
				shiftdanger=true;
				break;
			}
		}
	}
	else if(liftandcarryinfo->BodyState == SmallRightShift)
	{
		for(i=liftandcarryinfo->RightFoot.XMax;i<=liftandcarryinfo->RightFoot.XMax + 20 ;i++)
		{
			for(h=liftandcarryinfo->LeftFoot.YMax; h > 0; h--)
			{
				if(liftandcarryinfo->WhichStair == Stair_0 || liftandcarryinfo->WhichStair == Stair_4)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == TrdColor )
					{
						smallshiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_1 || liftandcarryinfo->WhichStair == Stair_3)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == SecColor )
					{
						smallshiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_2)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == TopColor )
					{
						smallshiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_5)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == FileColor )
					{
						shiftdistance[i-liftandcarryinfo->RightFoot.XMax]=liftandcarryinfo->LeftFoot.YMax-h;
						break;
					}
				}
			}
		}
		for(i=0;i<21;i++)
		{
			if(smallshiftdistance[i]<SureUpDistance )
			{
				shiftdanger=true;
				break;
			}
		}
	}
}
/*void KidsizeStrategy::AvoidDrop()
{
	avoiddrop=false;
	int dropdistance[101]={0};
	if(liftandcarryinfo->BodyState == RightShift)
	{
		for(i=liftandcarryinfo->RightFoot.XMax;i<=liftandcarryinfo->RightFoot.XMax + 100 ;i++)
		{
			for(h=liftandcarryinfo->LeftFoot.YMax; h > 0; h--)
			{
				if(liftandcarryinfo->WhichStair == Stair_1)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == FileColor )
					{
						dropdistance[ liftandcarryinfo->LeftFoot.YMax-h ] = i-liftandcarryinfo->RightFoot.XMax;
						break;
					}
				}
				else if(liftandcarryinfo->WhichStair == Stair_2)
				{
					if( strategy_info->label_model[ ImageWidth * h + i ] == TrdColor )
					{
						dropdistance[ liftandcarryinfo->LeftFoot.YMax-h ] = i-liftandcarryinfo->RightFoot.XMax;
						break;
					}
				}
			}
		}
		for(i=0;i<101;i++)
		{
			if(dropdistance[i]< 50 )
			{
				avoiddrop=true;
				break;
			}
		}
	}
}*/