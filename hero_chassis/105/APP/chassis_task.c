#include "main.h"
#include "bsp_can.h"
#include "pid.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include <math.h>

chassis_move_t chassis_move;

//获取云台与底盘之间的夹角
void get_chassis_gimbal_angle(void)
{
	float temp,temp2;
	if(gimbal_move.motor_yaw.total_angle<155.3f)
		temp=gimbal_move.motor_yaw.total_angle+360.0f;
	else temp=gimbal_move.motor_yaw.total_angle;
	temp2=temp-155.3f;	
	chassis_move.chassis_gimbal_angle=temp2/360.0f*2*PI;
}

void chassis_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
    wheel_speed[1] = (-vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set);
    wheel_speed[3] = (vx_set - vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set);
    wheel_speed[2] = (vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set);
    wheel_speed[0] = (-vx_set + vy_set -MOTOR_DISTANCE_TO_CENTER * wz_set);
}

void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    float max_vector = 0.0f, vector_rate = 0.0f;
    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

	chassis_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, 
									      chassis_move_control_loop->wz_set, wheel_speed);
	
    // calculate the max speed in four wheels, limit the max speed
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        if (max_vector < chassis_move_control_loop->motor_chassis[i].speed_set)
        {
            max_vector = chassis_move_control_loop->motor_chassis[i].speed_set;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    // calculate pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
	
    // Assign a current value
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
	
}

void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        // update motor speed, accel is differential of speed PID
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
	}
		
	// calculate vertical speed, horizontal speed ,rotation speed, left hand rule
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed 
									- chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed
									+ chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed 
									- chassis_move_update->motor_chassis[3].speed) 
											* MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}


void chassis_init(chassis_move_t *chassis_move_init)
{
	    const static float motor_speed_pid[3] = 
			{15000.0f, 10.0f,0.0f};
			
		const static float chassis_follow_pid[3] = 
		    {1,0,0};
			
	    uint8_t i;
			
	    //Gets the remote control pointer
        chassis_move_init->chassis_RC = get_xyz_point();
			
	    for (i = 0; i < 4; i++)
        {
			chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);			
				PID_init(&chassis_move_init->motor_speed_pid[i], 
							motor_speed_pid, 
								16000, 
									2000,
										0);
        }
				
		PID_init(&chassis_move_init->chassis_follow_pid[0], 
							chassis_follow_pid, 
								2, 
									0,
										0.2);		
		
				
		chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
        chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

        chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
        chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
		
		chassis_feedback_update(chassis_move_init);
				
}

void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    float vx_set_channel, vy_set_channel;
    // deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->x, vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->y, vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
   
    *vx_set = vx_set_channel;
    *vy_set = vy_set_channel;
}

static void chassis_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    //*wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->z;
}

float constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    // get three control set-point
    chassis_control(&vx_set, &vy_set, &angle_set, chassis_move_control);
    //chassis_move_control->wz_set = angle_set;
    chassis_move_control->vx_set = constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);

}

void chassis_follow(chassis_move_t *chassis_follow)
{
	PID_calc(chassis_follow->chassis_follow_pid,gimbal_move.motor_yaw.total_angle,155.3f);
	chassis_follow->wz_set = - chassis_follow->chassis_follow_pid->out;
}
