#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "Gimbal_Task.h"
#include "Chassis_Task.h"


extern uint8_t Campaign_Ctrl;
extern uint8_t usmart_count ;						//usmartÉ¨Ãè100ms¼ÆÊý

void AllTask(void);

void pid_reset_yaw_speed(uint32_t kp, uint32_t ki, uint32_t kd);
void pid_reset_yaw_position(uint32_t kp, uint32_t ki, uint32_t kd);
void pid_reset_pitch_speed(uint32_t kp, uint32_t ki, uint32_t kd);
void pid_reset_pitch_position(uint32_t kp, uint32_t ki, uint32_t kd);
void pid_reset_pushrod_speed(uint32_t kp, uint32_t ki, uint32_t kd);
void pid_reset_pushrod_position(uint32_t kp, uint32_t ki, uint32_t kd);

void PID_Init(void);


#endif

