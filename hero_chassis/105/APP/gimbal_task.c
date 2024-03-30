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

}


void gimbal_init(gimbal_move_t *gimbal_move_init)
{
		const static float motor_location_pid_yaw[3] = 
			{7.0f, 0.0f, 1.0f};
					
		const static float motor_speed_pid_yaw[3] = 
			{260.0f, 0.001f, 1.0f};
	    //Gets the remote control pointer
        gimbal_move_init->gimbal_RC = get_xyz_point();
		gimbal_move_init->imu = get_imu_point();
		//拨弹轮数据获取
		gimbal_move_init->motor_yaw.gimbal_motor_measure = get_yaw_motor_measure_point();
			
		//YAW角度环pid
		PID_init(&gimbal_move_init->motor_location_pid, 
							motor_location_pid_yaw, 
								70.0f, 
									0.2f,
										0.0f);
		//YAW速度环pid	
		PID_init(&gimbal_move_init->motor_speed_pid, 			
							motor_speed_pid_yaw, 
								30000.0f, 
									1000.0f,
										0.0f);
				
		gimbal_move_init->gimbal_yaw.angle_delt = 45;
		gimbal_move_init->motor_yaw.first_total_angle = 155.3f;

		gimbal_feedback_update(gimbal_move_init);	

}

int16_t flag_1 = 0;
int16_t flag_2 = 0;

void gimbal_control(gimbal_move_t *gimbal_control)
{

	
	if(gimbal_control->gimbal_RC->z >= 330)
	{
		
		flag_1++;
		
		if(flag_1 == 30)		
		{
			gimbal_control->gimbal_yaw.angle_delt = gimbal_control->gimbal_yaw.angle_delt - 1;
			flag_1 = 0;
		}
		 			
	}		

	if(gimbal_control->gimbal_RC->z <= -330)
	{
		flag_2++;
		
		if(flag_2 == 30)		
		{
			gimbal_control->gimbal_yaw.angle_delt = gimbal_control->gimbal_yaw.angle_delt + 1;
			flag_2 = 0;
		}		
				
	}
	
	if(gimbal_control->gimbal_yaw.angle_delt - gimbal_control->imu->imu_yaw > 180)
		gimbal_control->gimbal_yaw.angle_delt -= 360;
			
	if(gimbal_control->gimbal_yaw.angle_delt - gimbal_control->imu->imu_yaw < -180)
		gimbal_control->gimbal_yaw.angle_delt  += 360;			
	
	gimbal_control->gimbal_yaw.angle_out = PID_calc(&gimbal_control->motor_location_pid,gimbal_control->imu->imu_yaw,gimbal_control->gimbal_yaw.angle_delt);
    gimbal_control->motor_yaw.give_current = PID_calc(&gimbal_control->motor_speed_pid,gimbal_control->motor_yaw.gimbal_motor_measure->speed_rpm,gimbal_control->gimbal_yaw.angle_out);

}
