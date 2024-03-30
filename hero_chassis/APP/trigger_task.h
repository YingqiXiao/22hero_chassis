#ifndef TRIGGER_H
#define TRIGGER_H

#include "stdint.h"
#include "bsp_can.h"
#include "pid.h"

#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define REDUCTION_RATE 3591/187
#define GEAR_REDUCTION_RATE 2

//8192 * 19 * 2 / 7

typedef struct
{
    const motor_measure_t *trigger_motor_measure;
    float speed;
    float speed_set;
    int16_t give_current;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	int32_t		round_cnt;
	int32_t		total_angle;	
} trigger_motor_t;

typedef struct
{
	int16_t flag_trigger;
		
	float setangle;
	
	float setround;
	
	float angle_out;
	
}trigger_t;

typedef struct
{
  const trigger_measure_t *trigger_RC;               //the point to remote control
  trigger_motor_t motor_trigger;          //chassis motor data.
  trigger_t trigger_wheel;
  pid_type_def motor_speed_pid;             //motor speed PID.
  pid_type_def motor_location_pid;              //follow angle PID.
	
} trigger_move_t;

extern trigger_move_t trigger_move;

void trigger_feedback_update(trigger_move_t *trigger_move_update);
void trigger_init(trigger_move_t *trigger_move_init);
void trigger_control(trigger_move_t *trigger_control);

#endif
