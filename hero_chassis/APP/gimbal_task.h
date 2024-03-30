#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stdint.h"
#include "bsp_can.h"
#include "pid.h"

#define M6020_MOTOR_RPM_TO_VECTOR 1//M6020参数目前还未知

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
	float       actual_angle;
	float       add_angle_imu;
	float       add_angle_encorder;
} gimbal_motor_t;


typedef struct
{				
	float target_speed_imu;
	
	float target_speed_encorder;	
	
	float target_angle_imu;
	
	float target_angle_encorder;

}gimbal_t;

typedef struct
{
  const xyz_measure_t *gimbal_RC;               //the point to remote control
  const imu_measure_t *imu;
  const auto_measure_t *auto_target;
  gimbal_motor_t motor_yaw;
  gimbal_t gimbal_yaw;

  pid_type_def motor_speed_encorder_pid;             //motor speed PID.
  pid_type_def motor_location_encorder_pid;              //follow angle PID.
	
  pid_type_def motor_speed_imu_pid;             //motor speed PID.
  pid_type_def motor_location_imu_pid;              //follow angle PID.  
	
} gimbal_move_t;

extern gimbal_move_t gimbal_move;

void gimbal_feedback_update(gimbal_move_t *gimbal_move_update);
void gimbal_init(gimbal_move_t *gimbal_move_init);
void gimbal_control(gimbal_move_t *gimbal_control);

#endif
