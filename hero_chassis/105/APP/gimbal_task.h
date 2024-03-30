#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stdint.h"
#include "bsp_can.h"
#include "pid.h"

#define M6020_MOTOR_RPM_TO_VECTOR 1//M6020参数目前还未知

#define CHASSIS_CONTROL_TIME 0.002f

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    float speed;
    float speed_set;
    int16_t give_current;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	int32_t		round_cnt;
	float		total_angle;
	float     first_total_angle;
} gimbal_motor_t;


typedef struct
{			
	float setangle;
	
	float angle_out;
	
	int16_t angle_delt;

}gimbal_t;

typedef struct
{
  const xyz_measure_t *gimbal_RC;               //the point to remote control
  const imu_measure_t *imu;
  gimbal_motor_t motor_yaw;
  gimbal_t gimbal_yaw;

  pid_type_def motor_speed_pid;             //motor speed PID.
  pid_type_def motor_location_pid;              //follow angle PID.
	
} gimbal_move_t;

extern gimbal_move_t gimbal_move;

void gimbal_feedback_update(gimbal_move_t *gimbal_move_update);
void gimbal_init(gimbal_move_t *gimbal_move_init);
void gimbal_control(gimbal_move_t *gimbal_control);

#endif
