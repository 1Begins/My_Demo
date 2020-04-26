#include "Jscope_Task.h"
#include "Gimbal_Task.h"
#include "CanBus_Task.h"
#include "Vision_interact.h"

double_closed_loop Yaw_data;
double_closed_loop pluck_data;
double_closed_loop pitch_data;
double_closed_loop pushrod_data;
double_closed_loop vision_yaw_data;

single_closed_loop classis_data;

void Jscope_pushrod()
{
	pushrod_data.angle      	 = Gimbal_Motor_Encoder[PLUCKUP_CAN1_205].total_angle/36;
	pushrod_data.angle_init		 = Gimbal_Motor_Encoder[PLUCKUP_CAN1_205].offset_angle;
	pushrod_data.angular_speed = Gimbal_Motor_Encoder[PLUCKUP_CAN1_205].speed_rpm/36;
	pushrod_data.Pout_Sset		 = Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205].pos_out;
	pushrod_data.Pset     		 = PluckUP_angel_set;
	pushrod_data.pid_out			 = Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205].pos_out;
}

void Jscope_pluck()
{
	pluck_data.angle				 = Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle/19.2;
	pluck_data.angle_init		 = Gimbal_Motor_Encoder[PLUCK_CAN2_206].angle_bias;
	pluck_data.angular_speed = Gimbal_Motor_Encoder[PLUCK_CAN2_206].speed_rpm/19.2;
	pluck_data.Pout_Sset 		 = Gimbal_Motor_pid_Pos[PLUCK_CAN2_206].pos_out;
	pluck_data.Pset 				 = PluckData.angle_set;
	pluck_data.pid_out 			 = Gimbal_Motor_pid_Spd[PLUCK_CAN2_206].pos_out;
}

void Jscope_yaw()
{
	Yaw_data.angle 				 = Gimbal_Motor_Encoder[YAW_CAN2_205].total_angle;
	Yaw_data.angle_init		 = Gimbal_Motor_Encoder[YAW_CAN2_205].angle_bias;
	Yaw_data.angular_speed = Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm;
	Yaw_data.Pout_Sset		 = Gimbal_Motor_pid_Pos[YAW_CAN2_205].pos_out;
	Yaw_data.Pset					 = GimbalData[YAW].angle_set;
	Yaw_data.pid_out			 = Gimbal_Motor_pid_Spd[YAW_CAN2_205].pos_out;
}

void Jscope_pitch()
{
	pitch_data.angle 				 = Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle;
	pitch_data.angle_init		 = Gimbal_Motor_Encoder[PLUCK_CAN2_206].angle_bias;
	pitch_data.angular_speed = Gimbal_Motor_Encoder[PLUCK_CAN2_206].speed_rpm;
	pitch_data.Pout_Sset		 = Gimbal_Motor_pid_Pos[PLUCK_CAN2_206].pos_out;
	pitch_data.Pset 				 = GimbalData[PITCH].angle_set;
	pitch_data.pid_out 			 = Gimbal_Motor_pid_Spd[PLUCK_CAN2_206].pos_out;
}

void Jscope_vision_yaw()
{
	
	vision_yaw_data.angle					= gimbal_control.gimbal_yaw_motor.Vision_Angle_Error;
	vision_yaw_data.angular_speed	= Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm;
	vision_yaw_data.Pout_Sset 		= Gimbal_Pid_Vision_Pos[0].pos_out;
	vision_yaw_data.pid_out				= Gimbal_Pid_Vision_Spd[0].pos_out;
	
}
