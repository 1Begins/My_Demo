#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "pid.h"
#include "mytype.h"

#define PITCH_ROTATE_INC_FACT  0.006
#define YAW_ROTATE_INC_FACT  0.02

#define CHASSIS_YAW_MID_FRONT        1310   //����ǰ��������yaw���������Ӧ����ֵ
#define CHASSIS_YAW_MID_BACK         5406   //���̺���������yaw���������Ӧ����ֵ
#define FAULT_TOLERANT_OFFSET_ANGLE  23     //�ϵ�ʱyaw����̨����ʱ���ݴ�Ƕȣ��������ٶ�

//�Ƕ�����������
#define ANGLE_LIMIT(angle,angle_max,angle_min)        \
{                                                     \
	if(angle > angle_max){                              \
		angle = angle_max;                                \
	}else if(angle<angle_min){                          \
	  angle = angle_min;}                               \
}

/*��̨������,���ṹ������ʹ��*/
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
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_Auto_aiming,    //��̨�������
  GIMBAL_Manaul_control, //��̨�Ƹ�ң�ز���
  GIMBAL_Wave_move,   //��̨����ģʽ 
} gimbal_behaviour_e;

//��̨������ݽṹ��
typedef struct 
{
	
	float angle_set;           //�����Ƕ�
	float actual_angle;        //ʵ�ʽǶ�
	
  uint16_t target_val;       //�Ӿ�Ŀ��ֵ
	
	float base_angle;         //��׼�Ƕ�
	
	float relative_angle;     //�������Ƕ�
	float relative_angle_set; //�������Ƕ�����
	float relative_offset_angle; //��������ʼ�Ƕ�
	float absolute_angle;     //�����ǽǶ�
	float absolute_angle_set; //�����ǽǶ�����
	float absolute_offset_angle;//�����ǳ�ʼ�Ƕ�
	
	float gimbal_can_send;     //��̨CAN����ֵ
	
	float cail_angle;       //�����Ƕ�
	
	int16_t Vision_Px_Error;    //�Ӿ�����ƫ��
	float   Vision_Angle_Error;
	float   Vision_Data;

}gimbal_motor_t;

typedef struct
{
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

/*PLUCK�����ݽṹ��*/
typedef struct 
{
	bool pluck_flag;						//������־λ
	int	 pluck_time_count;						//500ms��������
	int32_t angle_set;
	int32_t speed_set;
	int32_t angle_staging;			//�Ƕ��ݴ�
	
	/*����������ʹ��*/
	bool lock_flag;     				//һ�β�����ɱ�־λ
	int	 lock_time_count;						//150ms����ɨ����ʱ����
	
	int32_t set_angle;              //�趨�Ƕ�
	int32_t offset_relative_angle;  //�ϵ�ʱyaw������ڵ���ǰ�������ĵ���Ի�е�Ƕ�ֵ
	int32_t relative_angle;         //����ڵ���ǰ�������ĵ���Ի�е�Ƕ�ֵ
	float relative_angle_rad;       //��̨����̵���ԽǶȣ���λ��rad
	float can_send;                 //CAN���͵��ٶȻ�PID���ֵ
	float set_angle_imu;
	
}gimbal_t;

extern pid_t Gimbal_Motor_pid_Spd[6] ;  //��̨���PID�ṹ��
extern pid_t Gimbal_Motor_pid_Pos[6] ;
extern pid_t Gimbal_Pid_Vision_Pos[2];
extern pid_t Gimbal_Pid_Vision_Spd[2];

extern uint8_t Gimbal_Init_Flag;  //��̨��ʼ����ʶ

extern gimbal_t PluckData;
extern gimbal_t GimbalData[2];  //��̨�ṹ��
extern gimbal_control_t gimbal_control;

//λ�û��Ƕ��趨ֵ
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
