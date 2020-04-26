#ifndef _JSCOPE_TASK_H
#define _JSCOPE_TASK_H

#include "mytype.h"

typedef struct{
	
	float angle_init;		//初始角度
	float angle;    		//实时电机角度
	float angular_speed;	//角速度
	float Pset;				//位置环设定值
	float Pout_Sset;		//位置环输出值
	float pid_out;			//串级PID输出
	
}double_closed_loop;

typedef struct{
	
	float angular_speed;	//角速度
	float Pset;				//位置环设定值
	float pid_out;			//PID输出
	
}single_closed_loop;


void Jscope_pluck(void);
void Jscope_pitch(void);           
void Jscope_yaw(void);
void Jscope_pushrod(void);
void Jscope_vision_yaw(void);


#endif

