#include "strategy/strategy_main.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kidsize");
	ros::NodeHandle nh;
	KidsizeStrategy kidsizeStrategy(nh);
	ros::Rate loop_rate(30);

	while (nh.ok()) 
	{
		kidsizeStrategy.strategymain();		//主策略
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void KidsizeStrategy::strategymain()
{
	if(!strategy_info->getStrategyStart())	//策略指撥開關沒開啟
	{
		ros_com->sendBodySector(29);
        tool->Delay(2000);
		hand_state_change = true;
	}
	else//策略指撥開關開啟
	{
		int p_x = 0;
		int p_y = 0;
		if(first_flag)
		{
			ros_com->sendBodySector(29);
        	tool->Delay(2000);
			MoveHead(HeadMotorID::HorizontalID,2048, 200);
			MoveHead(HeadMotorID::VerticalID,1800, 200);

			first_flag =false;	
		}	
		// MoveHead(HeadMotorID::VerticalID,1900, 200);
		ROS_INFO("red object cnt = %d ",strategy_info->color_mask_subject_cnts[5]);		
		for (int i = 0; i < strategy_info->color_mask_subject_cnts[5]; i++)
        {
			if (strategy_info->color_mask_subject[5][i].size > 300)
        	{	


				strategy_info->get_image_flag = true;
    			ros::spinOnce();
    			while(abs(strategy_info->color_mask_subject[5][i].YMin - 120) > 0)//頭部上下轉動直到對準籃框的水平中心線
    			{
    			    if(strategy_info->color_mask_subject[5][i].YMin == 0)
    			    {
    			        MoveHead(HeadMotorID::VerticalID,1800, 200);
    			    } 
    			    else if((strategy_info->color_mask_subject[5][i].YMin - 120) > 2)
    			    {
    			        MoveHead(HeadMotorID::VerticalID,VerticalHeadPosition - 1, 200);
    			    }
    			    else if((strategy_info->color_mask_subject[5][i].YMin - 120) < -2)
    			    {
    			        MoveHead(HeadMotorID::VerticalID,VerticalHeadPosition + 1, 200);
    			    }
					else
					{
						strategy_info->get_image_flag = true;
    					ros::spinOnce();
						break;
					}
    			    strategy_info->get_image_flag = true;
    				ros::spinOnce();
    			}
				p_x = strategy_info->color_mask_subject[5][i].X;
				p_y = strategy_info->color_mask_subject[5][i].YMin;
				ROS_INFO("red object X = %d ",strategy_info->color_mask_subject[5][i].X);
				ROS_INFO("red object ymax = %d ",strategy_info->color_mask_subject[5][i].YMin);

			}

		}

		MoveHead(HeadMotorID::VerticalID,VerticalHeadPosition, 200);

		int head_motor_d = VerticalHeadPosition;
		double head_a = (double)(head_motor_d-2048)*360/4096;
		double error = 22.5;
		double HeadVerticalAngle = ((double)(head_motor_d - 1024) * Scale2Deg) + head_a + error;
		ROS_INFO("HeadAngle = %lf", head_a);
    	double Distancenew = (4 + 3.85 * sin(HeadVerticalAngle * Deg2Rad)) * tan(HeadVerticalAngle * Deg2Rad) + 3.85 * cos(HeadVerticalAngle * Deg2Rad) + 0;
		
    	ROS_INFO("DistanceError = %lf", Distancenew);


		systemtransform(p_x, p_y, head_a, Distancenew);

		strategy_info->get_image_flag = true;
	}

}

void KidsizeStrategy::systemtransform(int x, int y, double headangle,double distance)
{
	double yaw_angle[3] = {0,-90+abs(headangle),-90};
	double s_r = sin(yaw_angle[0]*Deg2Rad);
	double c_r = cos(yaw_angle[0]*Deg2Rad);

	double s_p = sin(yaw_angle[1]*Deg2Rad);
	double c_p = cos(yaw_angle[1]*Deg2Rad);

	double s_y = sin(yaw_angle[2]*Deg2Rad);
	double c_y = cos(yaw_angle[2]*Deg2Rad);

	Float T[9] = {   c_y*c_p     ,c_y*s_p*s_r-s_y*c_r     ,c_y*s_p*c_r+s_y*s_r  , 
        		     s_y*c_p     ,s_y*s_p*s_r+c_y*c_r     ,s_y*s_p*c_r-c_y*s_r  , 
        		      -s_p       ,      c_p*s_r           ,      c_p*c_r        };
	
	fMatrix out_matrix(T, 3, 3);
    
	Float m[3] ={ 0,
			      0, 
			      0};

	fMatrix move(m, 3, 1);

	Float i[9] =  {493.5,    0   , 162.82, 
				     0  , 460.97 , 119.69, 
				     0  ,    0   ,   1     };

	fMatrix in_matrix(i, 3, 3);

	fMatrix out_matrix_n(3, 3);
	out_matrix_n = Inverse(out_matrix);

	fMatrix in_matrix_n(3, 3);
	in_matrix_n = Inverse(in_matrix);

	Float p[3] = {(double)x, 
				  (double)(240-y), 
				  1 };

	fMatrix pixel(p, 3, 1);
	fMatrix end_point(3, 1);
	
	// end_point = out_matrix_n*in_matrix_n*pixel*distance-out_matrix_n*in_matrix_n*move;
	//end_point.Show();

	Float m_l[3] ={ 0,
			        8.525 , 
			        -3.125 };//cm 3.123 7.4 8.525

	fMatrix move_l(m_l, 3, 1);
	fMatrix end_point_l(3, 1);
	
	end_point_l = out_matrix_n*in_matrix_n*pixel*distance-out_matrix_n*move_l;
	end_point_l.Show();
	end_point_l = motion_effect_offset(30, end_point_l);
	fVector end_point_vector(end_point_l.GetCol(0)); 
	endpoint_temp.x = (int)(end_point_vector.receiveelem(0)*1000);
	endpoint_temp.y = (int)((end_point_vector.receiveelem(1)+3.7)*1000);
	endpoint_temp.z = (int)(end_point_vector.receiveelem(2)*1000);

	// endpoint_temp.x = (int)(99*100);
	// endpoint_temp.y = (int)(150*100);
	// endpoint_temp.z = (int)(-99*100);

	endpoint_temp.state = 1;

	ROS_INFO("x = %d, y = %d, z = %d\n",endpoint_temp.x,endpoint_temp.y,endpoint_temp.z);
	if(strategy_info->DIOValue.Switch.D0 == 1)
	{
		if(hand_state_change == true)
		{
			trajectory_plan(endpoint_temp);
			hand_state_change = false;	
		}
	}

}

fMatrix KidsizeStrategy::motion_effect_offset(double waist_angle, fMatrix end_point_l)
{
	//211.5mm upper body long
	endpoint_temp.x = endpoint_temp.x - 21.15*sin((waist_angle*360/4096)*Deg2Rad);
	endpoint_temp.z = endpoint_temp.z + (21.15 - 21.15*cos((waist_angle*360/4096)*Deg2Rad));
	
	Float waist_m[3] ={ -21.15*sin((waist_angle*360/4096)*Deg2Rad),
			        	0 , 
			        	21.15 - 21.15*cos((waist_angle*360/4096)*Deg2Rad) };//cm 3.123 7.4 8.525

	fMatrix waist_move(waist_m, 3, 1);

	end_point_l = end_point_l + waist_move;

	end_point_l.Show();

	double yaw_angle[3] = {0,-waist_angle,0};
	double s_r = sin(yaw_angle[0]*Deg2Rad);
	double c_r = cos(yaw_angle[0]*Deg2Rad);

	double s_p = sin(yaw_angle[1]*Deg2Rad);
	double c_p = cos(yaw_angle[1]*Deg2Rad);

	double s_y = sin(yaw_angle[2]*Deg2Rad);
	double c_y = cos(yaw_angle[2]*Deg2Rad);

	Float T[9] = {   c_y*c_p     ,c_y*s_p*s_r-s_y*c_r     ,c_y*s_p*c_r+s_y*s_r  , 
        		     s_y*c_p     ,s_y*s_p*s_r+c_y*c_r     ,s_y*s_p*c_r-c_y*s_r  , 
        		      -s_p       ,      c_p*s_r           ,      c_p*c_r        };

	fMatrix transfer_matrix(T, 3, 3);			  

	end_point_l = transfer_matrix*end_point_l;

	return end_point_l;


}

void KidsizeStrategy::MoveHead(HeadMotorID ID, int Position, int Speed)//動頭(馬達編號，刻度，速度)
{
    ros_com->sendHeadMotor(ID,Position,Speed);
    tool->Delay(50);
    if(ID == HeadMotorID::HorizontalID) 
    {
        HorizontalHeadPosition = Position;
    }
    else
    {
        VerticalHeadPosition = Position;
    }
}

void KidsizeStrategy::trajectory_plan(tku_msgs::PointData endpoint)
{
	// while(strategy_info->DIOValue.Switch.D0 == 0 || strategy_info->DIOValue.Switch.D1 == 0 || strategy_info->DIOValue.Switch.D1 == 0 || strategy_info->DIOValue.Switch.D1 == 0)
	// {
	// 	ROS_INFO("in trajectory plan");
	// 	if(strategy_info->DIOValue.Switch.D0 == 1 && strategy_info->DIOValue.Switch.D1 == 0 && strategy_info->DIOValue.Switch.D1 == 0 && strategy_info->DIOValue.Switch.D1 == 0)
	// 	{
			ROS_INFO("end_point %d %d %d",endpoint.x,endpoint.y,endpoint.z);
			tku_msgs::PointData handup_point1;
			handup_point1 = endpoint;
			ROS_INFO("origin point1 %d %d %d",handup_point1.x,handup_point1.y,handup_point1.z);
			handup_point1.x = handup_point1.x - 5*1000;
			handup_point1.z = handup_point1.z - 1*1000;
			Endpoint_Publish.publish(handup_point1);
			ROS_INFO("point1 %d %d %d",handup_point1.x,handup_point1.y,handup_point1.z);
			tool->Delay(500);
		// }
		// else if(strategy_info->DIOValue.Switch.D0 == 1 && strategy_info->DIOValue.Switch.D1 == 1 && strategy_info->DIOValue.Switch.D1 == 0 && strategy_info->DIOValue.Switch.D1 == 0)
		// {		
			tku_msgs::PointData handup_point2;
			handup_point2 = endpoint;
			handup_point2.x = handup_point2.x - 5*1000;
			handup_point2.z = handup_point2.z + 1*1000;
			Endpoint_Publish.publish(handup_point2);
			ROS_INFO("point2 %d %d %d",handup_point2.x,handup_point2.y,handup_point2.z);
			tool->Delay(500);
		// }
		// else if(strategy_info->DIOValue.Switch.D0 == 1 && strategy_info->DIOValue.Switch.D1 == 1 && strategy_info->DIOValue.Switch.D1 == 1 && strategy_info->DIOValue.Switch.D1 == 0)
		// {
			tku_msgs::PointData handup_point3;
			handup_point3 = endpoint;
			handup_point3.x = handup_point3.x - 5*1000;
			handup_point3.z = handup_point3.z + 2*1000;
			Endpoint_Publish.publish(handup_point3);
			ROS_INFO("point3 %d %d %d",handup_point3.x,handup_point3.y,handup_point3.z);
			tool->Delay(500);
		// }
		// else if(strategy_info->DIOValue.Switch.D0 == 1 && strategy_info->DIOValue.Switch.D1 == 1 && strategy_info->DIOValue.Switch.D1 == 1 && strategy_info->DIOValue.Switch.D1 == 1)
		// {
			ROS_INFO("end point");
			endpoint.z = endpoint.z + 0.5*1000;
			Endpoint_Publish.publish(endpoint);
			tool->Delay(500);
		// }
		ros::spinOnce();
	// }
}

