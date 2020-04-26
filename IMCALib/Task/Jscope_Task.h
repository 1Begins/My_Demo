#ifndef _JSCOPE_TASK_H
#define _JSCOPE_TASK_H

#include "mytype.h"

typedef struct{
	
	float angle_init;		//��ʼ�Ƕ�
	float angle;    		//ʵʱ����Ƕ�
	float angular_speed;	//���ٶ�
	float Pset;				//λ�û��趨ֵ
	float Pout_Sset;		//λ�û����ֵ
	float pid_out;			//����PID���
	
}double_closed_loop;

typedef struct{
	
	float angular_speed;	//���ٶ�
	float Pset;				//λ�û��趨ֵ
	float pid_out;			//PID���
	
}single_closed_loop;


void Jscope_pluck(void);
void Jscope_pitch(void);           
void Jscope_yaw(void);
void Jscope_pushrod(void);
void Jscope_vision_yaw(void);


#endif

