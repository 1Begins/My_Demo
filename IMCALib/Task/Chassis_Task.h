#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "pid.h"


//3508�ڴ������ָ��������յ�����µ����ת��Ϊ430rpm���ң���������ٶȾ�����Ҫ���������ֵ
#define CHASSIS_MAXSPEED_RPM          400    //�������̵�����ת�٣���λ��rpm
#define CHASSIS_VX_MAXSPEED_RPM       350    //��������ƽ��ʱ�����̵��������ٶȣ���λ��rpm
#define REDUCTION_RATIO_3508          19.2   //3508������ٱ�
#define ROTATE_MOVE_MAXSPEED          2.5f    //С����ģʽ�µ�����ƶ��ٶȣ���λ��m/s
#define CHASSIS_ROTATE_RAD_SPEED      7.2f    //������ת�Ľ��ٶȣ�����˳ʱ�룬������ʱ��
#define CHASSIS_LENGTH_LX             0.237f  //���̵��������ĵ��������ĵľ����x���������ĸ����������Ĺ��ɵľ��εĿ��һ�룬��λ��m
#define CHASSIS_LENGTH_LY             0.160f  //���̵��������ĵ��������ĵľ����y���������ĸ����������Ĺ��ɵľ��εĳ���һ�룬��λ��m
#define CHASSIS_WHEEL_RADIUS          0.076f  //�������ӵİ뾶����λ��m



/*���̵�����*/
enum{
    
    FRON_RIGH_201 = 0, //ǰ��
    FRON_LEFT_202 = 1, //ǰ��
    REAR_LEFT_203 = 2, //����
    REAR_RIGH_204 = 3, //����
    
};


/*���̽ṹ��*/
typedef struct{
    
    int32_t  fr_motor_rpm_201; //ǰ�ҵ��
    int32_t  fl_motor_rpm_202; //ǰ����
    int32_t  rl_motor_rpm_203; //������
    int32_t  rr_motor_rpm_204; //���ҵ��
    

    float vx; //����ƽ��
    float vy; //ǰ��
    float vw; //��ת
    
    //��������ϵ�е��ٶ�
    float car_vx; //��λ��m/s
    float car_vy;
    float car_vw; //rad/s
    
    //���Ӷ�Ӧ��������ϵ���ٶ�
    float wheel_rad_201; //���ӵ�ת�٣���λ��rad/s
    float wheel_rad_202;
    float wheel_rad_203;
    float wheel_rad_204;
    
}chassis_t;


extern uint8_t Chassis_Ctrl;
extern pid_t  Moto_Chassis_Pid_Pos[4];  //λ�û�PID�ṹ��
extern pid_t  Moto_Chassis_Pid_Spd[4];  //�ٶȻ�PID�ṹ��
extern chassis_t  Chassis;


void ChassisControl(void);
void ChassisControlSetValue(void);
void ChassisPidCalc(void);
void ChassisDataCanSend(void);

void ChassisNormalCalc(void);
void ChassisFollowCalc(void);
void ChassisSpeedCalc(void);
float ChassisRotationPID(float measure, float target);

void Chassis_PID_Init(void);


#endif
