#include "bsp_can.h"
#include "pid.h"
#include "main.h"
#include "trigger_task.h"

trigger_move_t trigger_move;

void trigger_feedback_update(trigger_move_t *trigger_move_update)
{
    if (trigger_move_update == NULL)
    {
        return;
    }

        // update motor speed, accel is differential of speed PID
    trigger_move_update->motor_trigger[0].speed = M3508_MOTOR_RPM_TO_VECTOR * trigger_move_update->motor_trigger[0].trigger_motor_measure->speed_rpm;

	//拨弹轮电机总角度的读取
	trigger_move_update->motor_trigger[0].total_angle = trigger_move_update->motor_trigger[0].trigger_motor_measure->total_angle/8192.0f*360.0f;
	
}

void trigger_init(trigger_move_t *trigger_move_init)
{
		const static float motor_location_pid[3] = 
			{10.0f, 0.0f, 10.0f};
		const static float motor_speed_pid[3] = 
			{30.0f, 2.5f, 30.0f};
						
	    //Gets the remote control pointer
        trigger_move_init->trigger_RC = get_trigger_point();
					
		//拨弹轮数据获取
		trigger_move_init->motor_trigger[0].trigger_motor_measure = get_trigger_motor_measure_point();
			
		//角度环pid
		PID_init(&trigger_move_init->motor_location_pid, 
							motor_location_pid, 
								3000.0f, 
									1500.0f,
										100.0f);
		//速度环pid	
		PID_init(&trigger_move_init->motor_speed_pid, 			
							motor_speed_pid, 
								16000.0f, 
									6000.0f,
										100.0f);
				
		trigger_feedback_update(trigger_move_init);
		
		//拨弹轮初始化
		trigger_move_init->trigger_wheel[0].angle_out = 0;
		trigger_move_init->trigger_wheel[0].setround = 0;
		trigger_move_init->trigger_wheel[0].angle_out = 0;
		trigger_move_init->trigger_wheel[0].flag_trigger = 0;
		
}

//拨弹轮控制
void trigger_control(trigger_move_t *trigger_control)
{
	
	if (trigger_control->trigger_RC->trigger >= 330 && trigger_control->trigger_wheel[0].flag_trigger == 0)
	{
		trigger_control->trigger_wheel[0].setround = trigger_control->trigger_wheel[0].setround + 1;
		
		trigger_control->trigger_wheel[0].flag_trigger = 1;
	}	
	
	trigger_control->trigger_wheel[0].setangle = trigger_control->trigger_wheel[0].setround * 1978;
	
	trigger_control->trigger_wheel[0].angle_out = PID_calc(&trigger_control->motor_location_pid,trigger_control->motor_trigger[0].total_angle,trigger_control->trigger_wheel[0].setangle);
	
	trigger_move.motor_trigger[0].give_current = PID_calc(&trigger_control->motor_speed_pid,trigger_control->motor_trigger[0].trigger_motor_measure->speed_rpm,trigger_control->trigger_wheel[0].angle_out);

}
