/*************************************************************************************************************************
 * @File   Chassis_Task.c        
 * @Brief  机器人底盘控制，从底盘右上角电机逆时针开始数，底盘对应电机的ID为1 ~ 4，底盘四个电机的控制是挂在CAN1上，
 *         PID对应的参数是在带有轮子负载的情况下调试得到的
 *
*************************************************************************************************************************/

#include "Chassis_Task.h"
#include "CanBus_Task.h"
#include "RC_Task.h"
#include "pid.h"
#include "ramp.h"
#include "Gimbal_Task.h"
#include "arm_math.h"
#include "user_lib.h"

uint8_t Chassis_Ctrl; //底盘控制标志位
chassis_t  Chassis;  /*用于底盘*/

pid_t  Moto_Chassis_Pid_Pos[4];  //底盘电机位置环PID结构体
pid_t  Moto_Chassis_Pid_Spd[4];  //底盘电机速度环PID结构体
pid_t  Chassis_Rotate;  				 //底盘旋转PID结构体


void Chassis_PID_Init()
{
    memset(&Chassis, 0, sizeof(chassis_t));
//    memset(&remote_control, 0, sizeof(RC_Type));
    
    ChassisRampInit(); //底盘斜坡初始化
    
    /*底盘四个电机速度环初始化PID参数*/
    for(int i=0; i<4; i++)
    {
        PID_struct_init(&Moto_Chassis_Pid_Spd[i], POSITION_PID, 15000, 500, 12.0f, 0.15f, 2.0f);
    }
		
    //底盘跟随旋转PID参数初始化
    PID_struct_init(&Chassis_Rotate, POSITION_PID, 200, 10, 0.7f, 0.0f, 6.5f); //0.3, 0.0, 5.0
}

void ChassisControl(void)
{
    
    ChassisControlSetValue(); 
    ChassisPidCalc(); 
    ChassisDataCanSend();
    
}



void ChassisControlSetValue(void)
{
	
    if(!Gimbal_Init_Flag||(Gimbal_Motor_Encoder[YAW_CAN2_205].msg_cnt < 50)) /*云台初始化，使云台回到底盘前面的中心*/
    {
        GimbalInit();
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
    else if(Gimbal_Init_Flag) /*云台初始化完成*/
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); /*初始化完成，可进行控制，指示灯灭*/
        if(RC_UPPER_RIGHT_SW_UP) /*遥控器右上拨杆推上，单独控制*/
        {
            
            ChassisNormalCalc();  //分离控制
            
        }
        else if(RC_UPPER_RIGHT_SW_MID) /*遥控器右上拨杆推到中间，跟随*/
        {
            ChassisFollowCalc(); //跟随
            
//            ChassisSpeedCalc(); //小陀螺
            
        }
        else if(RC_UPPER_RIGHT_SW_DOWN) /*遥控器右上拨杆推到下面，小陀螺*/
        {
            ChassisSpeedCalc(); //小陀螺
            
        }
//        /*云台设值*/
//        GimbalControlSetValue();
//        //发弹设值
//        ShootControlSetValue();
        
    }
		
}

void ChassisNormalCalc()
{
	  int16_t /*chassis_vx_channel,*/ chassis_vy_channel, chassis_vw_channel;

    /*遥控器死区限制*/
    //chassis_vx_channel = RcDeadlineLimit(remote_control.ch1, 10)*CHASSIS_MAXSPEED_RPM/660;
    chassis_vy_channel = RcDeadlineLimit(remote_control.ch2, 10)*CHASSIS_MAXSPEED_RPM/660;
    chassis_vw_channel = RcDeadlineLimit(remote_control.ch1, 10)*CHASSIS_MAXSPEED_RPM/660;


/****************************************斜坡测试*********************************************************/
        //一阶低通滤波代替斜坡作为底盘速度输入
        //FirstOrderFilterCali(&chassis_cmd_slow_set_vx, chassis_vx_channel);
        FirstOrderFilterCali(&chassis_cmd_slow_set_vy, chassis_vy_channel);

        //Chassis.vx = chassis_cmd_slow_set_vx.out; //左右平移
        Chassis.vy = chassis_cmd_slow_set_vy.out; //前后运动
        Chassis.vw = chassis_vw_channel; //自转

/*******************************************END***********************************************************/
        
        //J_Scope_rcinput_x = Chassis.vx;
        //J_Scope_rcinput_y = Chassis.vy;
        
        Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
        Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
}

/*底盘跟随云台模式*/
void ChassisFollowCalc(void)
{
    
    Chassis.vw = ChassisRotationPID(-GimbalData[YAW].relative_angle, 0);
    
    Chassis.vx = remote_control.ch1*CHASSIS_MAXSPEED_RPM/660; //左右平移
    Chassis.vy = remote_control.ch2*CHASSIS_MAXSPEED_RPM/660; //前后运动
//    Chassis.vw = remote_control.ch3*CHASSIS_MAXSPEED_RPM/660; //自转
    
    Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
    Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
    Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
    Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
    
}

void ChassisSpeedCalc(void)
{
    float sin_rotate, cos_rotate;
    
    /*大地坐标系（即云台坐标系）下速度的设定*/
    Chassis.vx = remote_control.ch1*ROTATE_MOVE_MAXSPEED/660; //左右移动
    Chassis.vy = remote_control.ch2*ROTATE_MOVE_MAXSPEED/660; //前后移动
    Chassis.vw = CHASSIS_ROTATE_RAD_SPEED; //底盘自转速度

    /*通过旋转矩阵将大地坐标系（即云台坐标系）下的速度转换为车体坐标系（即底盘坐标系）下的速度*/
    sin_rotate = arm_sin_f32(GimbalData[YAW].relative_angle_rad);
    cos_rotate = arm_cos_f32(GimbalData[YAW].relative_angle_rad);
    Chassis.car_vx = -Chassis.vx*cos_rotate - Chassis.vy*sin_rotate; //X轴方向的速度
    Chassis.car_vy = -Chassis.vx*sin_rotate + Chassis.vy*cos_rotate; //Y轴方向的速度
    Chassis.car_vw = Chassis.vw; //自转速度

    /*底盘轮子对应的角速度，此速度是在车体坐标系下的*/
    Chassis.wheel_rad_201 = (-Chassis.car_vx - Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //右上
    Chassis.wheel_rad_202 = (-Chassis.car_vx + Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //左上
    Chassis.wheel_rad_203 = (+Chassis.car_vx + Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //左下
    Chassis.wheel_rad_204 = (+Chassis.car_vx - Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //右下

    /*将底盘轮子速度转换为转每分钟，作为PID的输入*/
    Chassis.fr_motor_rpm_201 = Chassis.wheel_rad_201/RPM_To_Rads;
    Chassis.fl_motor_rpm_202 = Chassis.wheel_rad_202/RPM_To_Rads;
    Chassis.rl_motor_rpm_203 = Chassis.wheel_rad_203/RPM_To_Rads;
    Chassis.rr_motor_rpm_204 = Chassis.wheel_rad_204/RPM_To_Rads;
    
}

/*底盘旋转PID计算*/
float ChassisRotationPID(float measure, float target)
{
    pid_calc(&Chassis_Rotate, measure, target);
    return Chassis_Rotate.pos_out;
}

void ChassisPidCalc(void)
{
    
    pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_201], Chassis_Motor_Encoder[FRON_RIGH_201].speed_rpm, Chassis.fr_motor_rpm_201*REDUCTION_RATIO_3508);
    pid_calc(&Moto_Chassis_Pid_Spd[FRON_LEFT_202], Chassis_Motor_Encoder[FRON_LEFT_202].speed_rpm, Chassis.fl_motor_rpm_202*REDUCTION_RATIO_3508);
    pid_calc(&Moto_Chassis_Pid_Spd[REAR_LEFT_203], Chassis_Motor_Encoder[REAR_LEFT_203].speed_rpm, Chassis.rl_motor_rpm_203*REDUCTION_RATIO_3508);
    pid_calc(&Moto_Chassis_Pid_Spd[REAR_RIGH_204], Chassis_Motor_Encoder[REAR_RIGH_204].speed_rpm, Chassis.rr_motor_rpm_204*REDUCTION_RATIO_3508);
    
}


void ChassisDataCanSend(void)
{
   
//    if((Chassis.fr_motor_rpm_201 != 0)&&(Chassis.fl_motor_rpm_202 != 0)&&(Chassis.rl_motor_rpm_203 != 0)&&(Chassis.rr_motor_rpm_204 != 0))
//    {
        set_moto_current1(&hcan2, Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out, 
																	Moto_Chassis_Pid_Spd[FRON_LEFT_202].pos_out, \
																	Moto_Chassis_Pid_Spd[REAR_LEFT_203].pos_out,
																	Moto_Chassis_Pid_Spd[REAR_RIGH_204].pos_out		);
        
//    }
//    else
//    {
//        SetChassisMotorCurrent(&hcan1, 0, 0, 0, 0);
//    }
//    
    
}


