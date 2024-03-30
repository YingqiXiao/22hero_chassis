#include "main.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"

gimbal_move_t gimbal_move;

void gimbal_feedback_update(gimbal_move_t *gimbal_move_update)
{
    if (gimbal_move_update == NULL)
    {
        return;
    }
        // update motor speed, accel is differential of speed PID
        gimbal_move_update->motor_yaw.speed = M6020_MOTOR_RPM_TO_VECTOR * gimbal_move_update->motor_yaw.gimbal_motor_measure->speed_rpm;

		//yaw电机总角度的读取
	gimbal_move_update->motor_yaw.total_angle = gimbal_move_update->motor_yaw.gimbal_motor_measure->total_angle/8192.0f*360.0f;
	gimbal_move_update->motor_yaw.actual_angle = gimbal_move_update->motor_yaw.total_angle + 140.0f;
	while(gimbal_move_update->motor_yaw.actual_angle > 180.0f)
	{
		gimbal_move_update->motor_yaw.actual_angle = gimbal_move_update->motor_yaw.actual_angle -360.0f;
	}
	
	while(gimbal_move_update->motor_yaw.actual_angle < -180.0f)
	{
		gimbal_move_update->motor_yaw.actual_angle = gimbal_move_update->motor_yaw.actual_angle +360.0f;
	}	
}


void gimbal_init(gimbal_move_t *gimbal_move_init)
{
	
		const static float motor_encorder_location_pid_yaw[3] = 
			{0.08f, 0.0f, 3.0f};
					
		const static float motor_encorder_speed_pid_yaw[3] = 
			{7000.0f, 0.0f, 0.0f};
			
		const static float motor_imu_location_pid_yaw[3] = 
			{0.06f, 0.0f, 0.0f};
					
		const static float motor_imu_speed_pid_yaw[3] = 
			{30000.0f, 800.0f, 600.0f};			
	    //Gets the remote control pointer
        gimbal_move_init->gimbal_RC = get_xyz_point();
		gimbal_move_init->imu = get_imu_point();
		gimbal_move_init->auto_target = get_auto_measure_point();
		//拨弹轮数据获取
		gimbal_move_init->motor_yaw.gimbal_motor_measure = get_yaw_motor_measure_point();
			
		//YAW角度环pid
		PID_init(&gimbal_move_init->motor_location_encorder_pid, 
							motor_encorder_location_pid_yaw, 
								700.0f, 
									0.0f,
										0.01f);
		//YAW速度环pid	
		PID_init(&gimbal_move_init->motor_speed_encorder_pid, 			
							motor_encorder_speed_pid_yaw, 
								25000.0f, 
									1000.0f,
										0.0f);

		//YAW角度环pid
		PID_init(&gimbal_move_init->motor_location_imu_pid, 
							motor_imu_location_pid_yaw, 
								1.0f, 
									0.0f,
										0.1f);
		//YAW速度环pid	
		PID_init(&gimbal_move_init->motor_speed_imu_pid, 			
							motor_imu_speed_pid_yaw, 
								25000.0f, 
									8000.0f,
										0.0f);
		
		gimbal_move_init->gimbal_yaw.target_angle_encorder = 38.62;
		
		gimbal_move_init->motor_yaw.add_angle_imu = 0.1f;
		
		gimbal_move_init->motor_yaw.add_angle_encorder = 0.0001f;
		
		gimbal_feedback_update(gimbal_move_init);	

}

float separate = 0.095f;

uint8_t flag = 1;

uint8_t flag_back = 0;

void gimbal_control(gimbal_move_t *gimbal_control)
{

if(flag == 1)	
{
	gimbal_control->gimbal_yaw.target_angle_imu = gimbal_control->imu->imu_yaw;
	flag--;
}	
	
	else if(channel[0].sR == 3)
	{	

		if(gimbal_control->gimbal_RC->z >= 20 && gimbal_control->gimbal_RC->z <= 660)
		{

				gimbal_control->gimbal_yaw.target_angle_imu = gimbal_control->gimbal_yaw.target_angle_imu - gimbal_control->motor_yaw.add_angle_imu * gimbal_control->gimbal_RC->z;
						
		}		

		if(gimbal_control->gimbal_RC->z <= -20 && gimbal_control->gimbal_RC->z >= -660)
		{

				gimbal_control->gimbal_yaw.target_angle_imu = gimbal_control->gimbal_yaw.target_angle_imu + gimbal_control->motor_yaw.add_angle_imu * ( - gimbal_control->gimbal_RC->z);
						
		}
//		
//		if(flag_back == 0 && back[0].back == 1)
//		{
//			gimbal_control->gimbal_yaw.target_angle_imu += 90.0f;
//			flag_back = 1;
//		}
//		
//		if(back[0].back == 0)
//		{
//			flag_back = 0;
//		}		
		
		if((gimbal_control->gimbal_yaw.target_angle_imu - gimbal_control->imu->imu_yaw)/100.0f > 180.0f)
			gimbal_control->gimbal_yaw.target_angle_imu -= 36000.0f;
				
		if(gimbal_control->gimbal_yaw.target_angle_imu / 100.0f - gimbal_control->imu->imu_yaw / 100.0f < -180.0f)
			gimbal_control->gimbal_yaw.target_angle_imu += 36000.0f;			
		
		gimbal_control->gimbal_yaw.target_speed_imu = PID_calc(&gimbal_control->motor_location_imu_pid,gimbal_control->imu->imu_yaw / 100.0f,gimbal_control->gimbal_yaw.target_angle_imu / 100.0f);
		gimbal_control->motor_yaw.give_current = PID_calc_interal_seprate(&gimbal_control->motor_speed_imu_pid,gimbal_control->motor_yaw.gimbal_motor_measure->speed_rpm / 60.0f,gimbal_control->gimbal_yaw.target_speed_imu,separate);
}		

	
else if(channel[0].sR == 1)
{	
	
		gimbal_control->gimbal_yaw.target_speed_imu = PID_calc(&gimbal_control->motor_location_imu_pid,gimbal_control->imu->imu_yaw / 100.0f,gimbal_control->auto_target->auto_target_angle / 100.0f);
		gimbal_control->motor_yaw.give_current = PID_calc_interal_seprate(&gimbal_control->motor_speed_imu_pid,gimbal_control->motor_yaw.gimbal_motor_measure->speed_rpm / 60.0f,gimbal_control->gimbal_yaw.target_speed_imu,separate);		
		gimbal_control->gimbal_yaw.target_angle_imu = gimbal_control->auto_target->auto_target_angle;
}		

}
