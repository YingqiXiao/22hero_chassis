#include "bsp_can.h"
#include "main.h"
#include "delay.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//底盘电机发送句柄与数组
CAN_TxHeaderTypeDef  chassis_tx_message;
uint8_t              chassis_can_send_data[8];

//YAW电机发送句柄与数组
CAN_TxHeaderTypeDef  yaw_tx_message;
uint8_t              yaw_can_send_data[8];

//拨弹轮电机发送句柄与数组
CAN_TxHeaderTypeDef  trigger_tx_message;
uint8_t              trigger_can_send_data[8];

//接收数组
uint8_t rx_data[8];

motor_measure_t motor[MOTOR_NUM];//电机数据结构体

imu_measure_t imu[1];

xyz_measure_t xyz[1];

trigger_measure_t trigger[1];

channel_measure_t channel[1];

auto_measure_t auto_target[1];

fric_measure_t fric[1];

back_measure_t back[1];

void Can_Filter_Init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void XYZ_measure_fun(xyz_measure_t *ptr,uint8_t* data)
{
	(ptr)->x = (uint16_t)((data)[0] << 8 | (data)[1]);
	(ptr)->y = (uint16_t)((data)[2] << 8 | (data)[3]);
	(ptr)->z = (uint16_t)((data)[4] << 8 | (data)[5]);	
}

//电机原始数据解析
void Motor_measure_fun(motor_measure_t *ptr,uint8_t* data)                                 
{                                                                   
    (ptr)->last_ecd = (ptr)->ecd;                                   
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  
    (ptr)->temperate = (data)[6];
    if((ptr)->ecd - (ptr)->last_ecd > 4096)				
		(ptr)->round_cnt --;								
	else if((ptr)->ecd - (ptr)->last_ecd < -4096)		
		(ptr)->round_cnt ++;
	(ptr)->total_angle = (ptr)->round_cnt * 8192 + (ptr)->ecd;			
}

void Auto_measure_fun(auto_measure_t *ptr,uint8_t* data)                                 
{                                                                   
	(ptr)->auto_target_angle = (int16_t)((data)[1] | (data)[2] << 8);			
}

void Fric_measure_fun(fric_measure_t *ptr,uint8_t* data)                                 
{                                                                   
	(ptr)->fric_flag = data[0];		
}

void Imu_yaw_measure_fun(imu_measure_t *ptr,uint8_t* data)
{
	(ptr)-> imu_yaw = (int16_t)((data)[4] | (data)[5] << 8); 

}
void Channel_measure_fun(channel_measure_t *ptr,uint8_t* data)
{
	(ptr)-> sR = data[0]; 
	(ptr)-> sL = data[1]; 
}

void Imu_pitch_measure_fun(imu_measure_t *ptr,uint8_t* data)
{
	(ptr)-> imu_pitch = (int16_t)((data)[2] | (data)[3] << 8);
}

void Back_measure_fun(back_measure_t *ptr,uint8_t* data)
{
	(ptr)-> back = data[3];
}

void Trigger_measure_fun(trigger_measure_t *ptr,uint8_t* data)
{
	(ptr)->trigger = (uint16_t)((data)[0] << 8 | (data)[1]);
}

//CAN中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	//CAN接收句柄
	CAN_RxHeaderTypeDef rx_header;
		
	//CAN接收函数
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if(hcan->Instance == CAN1)
	{
	//接收底盘电机数据并解析
		if(rx_header.StdId == ID_Mec_L1_Rec)
			Motor_measure_fun(&motor[0], rx_data);
			
		if(rx_header.StdId == ID_Mec_L2_Rec)
			Motor_measure_fun(&motor[1], rx_data);
			
		if(rx_header.StdId == ID_Mec_R1_Rec)
			Motor_measure_fun(&motor[2], rx_data);
		
		if(rx_header.StdId == ID_Mec_R2_Rec)
			Motor_measure_fun(&motor[3], rx_data);
		
		if(rx_header.StdId == ID_TRIGGER_MOTOR_Rec)
			Motor_measure_fun(&motor[4], rx_data);
		
		
		
		
	}
	
	if(hcan->Instance == CAN2)
	{
		if(rx_header.StdId == ID_IMU_YAW_Rec)
			Imu_yaw_measure_fun(&imu[0], rx_data);
				
		if(rx_header.StdId == ID_XYZ_Rec)
			XYZ_measure_fun(&xyz[0], rx_data);
		
		if(rx_header.StdId == ID_TRIGGER_Rec)
			Trigger_measure_fun(&trigger[0],rx_data);
		
		if(rx_header.StdId == ID_CHANNEL_Rec)
			Channel_measure_fun(&channel[0],rx_data);

		if(rx_header.StdId == ID_IMU_PITCH_Rec)
			Imu_pitch_measure_fun(&imu[0],rx_data);

		if(rx_header.StdId == ID_AUTO_YAW_Rec)
			Auto_measure_fun(&auto_target[0],rx_data);

		if(rx_header.StdId == ID_FRIC_Rec)
			Fric_measure_fun(&fric[0],rx_data);	
		
		if(rx_header.StdId == ID_YAW_Rec)
			Motor_measure_fun(&motor[5], rx_data);		
		
	}
				
}

//拨弹轮电机电流发送
void CAN_cmd_trigger(int16_t trigger)
{
    

	uint32_t send_mail_box;
    trigger_tx_message.StdId = ID_TRIGGER_Tran;
    trigger_tx_message.IDE = CAN_ID_STD;
    trigger_tx_message.RTR = CAN_RTR_DATA;
    trigger_tx_message.DLC = 0x08;
    trigger_can_send_data[0] = trigger >> 8;
    trigger_can_send_data[1] = trigger;
    
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
    HAL_CAN_AddTxMessage(&hcan1, &trigger_tx_message, trigger_can_send_data, &send_mail_box);


}

//底盘电机电流发送
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

//YAW轴电机电流发送
void CAN_cmd_yaw(int16_t yaw)
{
	
	uint32_t send_mail_box;
    yaw_tx_message.StdId = ID_YAW_Tran;
    yaw_tx_message.IDE = CAN_ID_STD;
    yaw_tx_message.RTR = CAN_RTR_DATA;
    yaw_tx_message.DLC = 0x08;
    
	yaw_can_send_data[4] = yaw >> 8;
    yaw_can_send_data[5] = yaw;

    HAL_CAN_AddTxMessage(&hcan2, &yaw_tx_message, yaw_can_send_data, &send_mail_box);
}

const imu_measure_t *get_imu_point(void)
{
    return &imu[0];
}

const xyz_measure_t *get_xyz_point(void)
{
    return &xyz[0];
}

const trigger_measure_t *get_trigger_point(void)
{
    return &trigger[0];
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor[(i & 0x03)];
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor[4];
}

const motor_measure_t *get_yaw_motor_measure_point(void)
{
    return &motor[5];
}

const channel_measure_t *get_channel_measure_point(void)
{
    return &channel[0];
}

const auto_measure_t *get_auto_measure_point(void)
{
    return &auto_target[0];
}

const fric_measure_t *get_fric_measure_point(void)
{
    return &fric[0];
}
