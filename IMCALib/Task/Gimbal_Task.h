#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "pid.h"
#include "mytype.h"

#define PITCH_ROTATE_INC_FACT  0.006
#define YAW_ROTATE_INC_FACT  0.02

#define CHASSIS_YAW_MID_FRONT        1310   //底盘前面正中央yaw轴编码器对应的数值
#define CHASSIS_YAW_MID_BACK         5406   //底盘后面正中央yaw轴编码器对应的数值
#define FAULT_TOLERANT_OFFSET_ANGLE  23     //上电时yaw轴云台回中时的容错角度，正负多少度

//角度上下限声明
#define ANGLE_LIMIT(angle,angle_max,angle_min)        \
{                                                     \
	if(angle > angle_max){                              \
		angle = angle_max;                                \
	}else if(angle<angle_min){                          \
	  angle = angle_min;}                               \
}

/*云台电机序号,供结构体数组使用*/
enum{

	PLUCKUP_CAN1_205   = 0,
	PITCH_CAN1_206     = 1,
	FRICTION1_CAN1_207 = 2,	
	FRICTION2_CAN1_208 = 3,
	
	YAW_CAN2_205       = 4,
	PLUCK_CAN2_206     = 5,

};
enum{
    YAW   = 0,
    PITCH = 1,
    
};

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //云台无力
  GIMBAL_Auto_aiming,    //云台自瞄测试
  GIMBAL_Manaul_control, //云台推杆遥控测试
  GIMBAL_Wave_move,   //云台调参模式 
} gimbal_behaviour_e;

//云台电机数据结构体
typedef struct 
{
	
	float angle_set;           //给定角度
	float actual_angle;        //实际角度
	
  uint16_t target_val;       //视觉目标值
	
	float base_angle;         //基准角度
	
	float relative_angle;     //编码器角度
	float relative_angle_set; //编码器角度设置
	float relative_offset_angle; //编码器初始角度
	float absolute_angle;     //陀螺仪角度
	float absolute_angle_set; //陀螺仪角度设置
	float absolute_offset_angle;//陀螺仪初始角度
	
	float gimbal_can_send;     //云台CAN发送值
	
	float cail_angle;       //修正角度
	
	int16_t Vision_Px_Error;    //视觉像素偏差
	float   Vision_Angle_Error;
	float   Vision_Data;

}gimbal_motor_t;

typedef struct
{
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

/*PLUCK轴数据结构体*/
typedef struct 
{
	bool pluck_flag;						//拨弹标志位
	int	 pluck_time_count;						//500ms拨弹计数
	int32_t angle_set;
	int32_t speed_set;
	int32_t angle_staging;			//角度暂存
	
	/*供卡弹处理使用*/
	bool lock_flag;     				//一次拨弹完成标志位
	int	 lock_time_count;						//150ms卡弹扫描延时计数
	
	int32_t set_angle;              //设定角度
	int32_t offset_relative_angle;  //上电时yaw轴相对于底盘前面正中心的相对机械角度值
	int32_t relative_angle;         //相对于底盘前面正中心的相对机械角度值
	float relative_angle_rad;       //云台与底盘的相对角度，单位：rad
	float can_send;                 //CAN发送的速度环PID输出值
	float set_angle_imu;
	
}gimbal_t;

extern pid_t Gimbal_Motor_pid_Spd[6] ;  //云台电机PID结构体
extern pid_t Gimbal_Motor_pid_Pos[6] ;
extern pid_t Gimbal_Pid_Vision_Pos[2];
extern pid_t Gimbal_Pid_Vision_Spd[2];

extern uint8_t Gimbal_Init_Flag;  //云台初始化标识

extern gimbal_t PluckData;
extern gimbal_t GimbalData[2];  //云台结构体
extern gimbal_control_t gimbal_control;

//位置环角度设定值
extern int32_t PluckUP_angel_set;

void lock_bullet_detection(void);

void Gimbal_PID_Init(void);
void GimbalDataUpdate(void);
void ShootControlSetValue(void);
void GimbalPidCalc(void);
void GimbalPidOutput(void);

void GimbalInit(void);

void GimbalControl(void);
void GimbalRC(void);
void GimbalAuto(void);

#endif
