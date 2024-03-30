#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#define ID_Mec_L1_Rec 0x201
#define ID_Mec_L2_Rec 0x202
#define ID_Mec_R1_Rec 0x203
#define ID_Mec_R2_Rec 0x204

#define ID_YAW_Tran 0x2FF

#define ID_TRIGGER_Tran 0x1FF

#define ID_IMU_YAW_Rec 0x006

#define ID_YAW_Rec 0x20B

#define MOTOR_NUM 6 //电机数量

#define ID_XYZ_Rec 0x001

#define ID_TRIGGER_MOTOR_Rec 0x205

#define ID_TRIGGER_Rec 0x002

#define ID_CHANNEL_Rec 0x006

#define ID_IMU_PITCH_Rec 0x006

#define ID_AUTO_YAW_Rec 0x007

#define ID_FRIC_Rec 0x007

#define ID_BACK_Rec 0x007

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
	int32_t	round_cnt;
	int32_t	total_angle;
	
} motor_measure_t;

typedef struct
{
	int16_t imu_yaw;
	int16_t imu_pitch;
	
}imu_measure_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	
}xyz_measure_t;

typedef struct
{
	uint8_t sL;
	uint8_t sR;
	
}channel_measure_t;

typedef struct
{
	int16_t trigger;
	
}trigger_measure_t;

typedef struct
{
	int16_t auto_target_angle;
	
}auto_measure_t;

typedef struct
{
	int16_t fric_flag;
	
}fric_measure_t;

typedef struct
{
	int16_t back;
	
}back_measure_t;

void Can_Filter_Init(void);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_yaw(int16_t yaw);
void CAN_cmd_trigger(int16_t trigger);

extern channel_measure_t channel[1];
extern fric_measure_t fric[1];
extern back_measure_t back[1];

const xyz_measure_t *get_xyz_point(void);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
const motor_measure_t *get_yaw_motor_measure_point(void);
const motor_measure_t *get_trigger_motor_measure_point(void);
const trigger_measure_t *get_trigger_point(void);
const imu_measure_t *get_imu_point(void);
const channel_measure_t *get_channel_measure_point(void);
const auto_measure_t *get_auto_measure_point(void);
const fric_measure_t *get_fric_measure_point(void);

#endif
