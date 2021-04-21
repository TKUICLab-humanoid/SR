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
		
	}
	else//策略指撥開關開啟
	{
		int p_x = 0;
		int p_y = 0;
		if(first_flag)
		{
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
		double error = 5;
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
	
	end_point = out_matrix_n*in_matrix_n*pixel*distance-out_matrix_n*in_matrix_n*move;
	//end_point.Show();

	Float m_l[3] ={ 0,
			        8.525 , 
			        -3.125 };//cm 3.123 7.4 8.525

	fMatrix move_l(m_l, 3, 1);
	fMatrix end_point_l(3, 1);
	
	end_point_l = out_matrix_n*in_matrix_n*pixel*distance-out_matrix_n*move_l;
	end_point_l.Show();
	fVector end_point_vector(end_point_l.GetCol(0)); 
	endpoint_temp.x = (int)(end_point_vector.receiveelem(0)*100);
	endpoint_temp.y = (int)(end_point_vector.receiveelem(1)*100);
	endpoint_temp.z = -(int)(end_point_vector.receiveelem(2)*100);

	ROS_INFO("x = %d, y = %d, z = %d\n",endpoint_temp.x,endpoint_temp.y,endpoint_temp.z);

	Endpoint_Publish.publish(endpoint_temp);
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
