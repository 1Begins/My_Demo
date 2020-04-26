#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "pid.h"


//3508在带有麦轮负载且悬空的情况下的最大转速为430rpm左右，设置最大速度尽量不要超过这个数值
#define CHASSIS_MAXSPEED_RPM          400    //步兵底盘电机最大转速，单位：rpm
#define CHASSIS_VX_MAXSPEED_RPM       350    //底盘左右平移时，底盘电机的最大速度，单位：rpm
#define REDUCTION_RATIO_3508          19.2   //3508电机减速比
#define ROTATE_MOVE_MAXSPEED          2.5f    //小陀螺模式下的最大移动速度，单位：m/s
#define CHASSIS_ROTATE_RAD_SPEED      7.2f    //底盘自转的角速度，正：顺时针，负：逆时针
#define CHASSIS_LENGTH_LX             0.237f  //底盘的轮子中心到底盘中心的距离的x分量，即四个轮子轴中心构成的矩形的宽的一半，单位：m
#define CHASSIS_LENGTH_LY             0.160f  //底盘的轮子中心到底盘中心的距离的y分量，即四个轮子轴中心构成的矩形的长的一半，单位：m
#define CHASSIS_WHEEL_RADIUS          0.076f  //底盘轮子的半径，单位：m



/*底盘电机序号*/
enum{
    
    FRON_RIGH_201 = 0, //前右
    FRON_LEFT_202 = 1, //前左
    REAR_LEFT_203 = 2, //后左
    REAR_RIGH_204 = 3, //后右
    
};


/*底盘结构体*/
typedef struct{
    
    int32_t  fr_motor_rpm_201; //前右电机
    int32_t  fl_motor_rpm_202; //前左电机
    int32_t  rl_motor_rpm_203; //后左电机
    int32_t  rr_motor_rpm_204; //后右电机
    

    float vx; //左右平移
    float vy; //前后
    float vw; //自转
    
    //车体坐标系中的速度
    float car_vx; //单位：m/s
    float car_vy;
    float car_vw; //rad/s
    
    //轮子对应车体坐标系的速度
    float wheel_rad_201; //轮子的转速，单位：rad/s
    float wheel_rad_202;
    float wheel_rad_203;
    float wheel_rad_204;
    
}chassis_t;


extern uint8_t Chassis_Ctrl;
extern pid_t  Moto_Chassis_Pid_Pos[4];  //位置环PID结构体
extern pid_t  Moto_Chassis_Pid_Spd[4];  //速度环PID结构体
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
