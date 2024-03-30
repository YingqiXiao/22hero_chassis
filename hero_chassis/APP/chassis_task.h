#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "stdint.h"
#include "pid.h"
#include "bsp_can.h"

#define MOTOR_DISTANCE_TO_CENTER 0.2f

#define PI 3.141593f

// chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
// chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

// rocker value deadline
#define CHASSIS_RC_DEADLINE 10

// rocker value (max 660) change to vertial speed (m/s)
#define CHASSIS_VX_RC_SEN 0.006f
// rocker value (max 660) change to horizontal speed (m/s)
#define CHASSIS_VY_RC_SEN 0.005f

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
	
// single chassis motor max speed
#define MAX_WHEEL_SPEED 4.0f

// in not following yaw angle mode, rocker value change to rotation speed
#define CHASSIS_WZ_RC_SEN 0.01f	

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    float speed;
    float speed_set;
    int16_t give_current;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	int32_t		round_cnt;
	int32_t		total_angle;	
} chassis_motor_t;

typedef struct
{
  const channel_measure_t *channel_s;
  const xyz_measure_t *chassis_RC;               //the point to remote control
  chassis_motor_t motor_chassis[4];          //chassis motor data.
  pid_type_def motor_speed_pid[4];             //motor speed PID.
  pid_type_def chassis_follow_pid[1];
  pid_type_def buffer_pid;	//motor speed PID.	

  float vx;                          //chassis vertical speed, positive means forward,unit m/s. 
  float vy;                          //chassis horizontal speed, positive means letf,unit m/s.
  float wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.
  float vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.
  float vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.
  float vx_spin_set;                      //chassis set vertical speed,positive means forward,unit m/s.
  float vy_spin_set;                      //chassis set horizontal speed,positive means left,unit m/s.
  float wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.    

  float vx_max_speed;  //max forward speed, unit m/s.
  float vx_min_speed;  //max backward speed, unit m/s.
  float vy_max_speed;  //max letf speed, unit m/s.
  float vy_min_speed;  //max right speed, unit m/s.
  
  float chassis_gimbal_angle;
	
} chassis_move_t;

extern chassis_move_t chassis_move;

void chassis_init(chassis_move_t *chassis_move_init);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_set_contorl(chassis_move_t *chassis_move_control);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
void get_chassis_gimbal_angle(void);
void chassis_follow(void);
void chassis_spin(void);

#endif
