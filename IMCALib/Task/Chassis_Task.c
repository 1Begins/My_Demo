/*************************************************************************************************************************
 * @File   Chassis_Task.c        
 * @Brief  �����˵��̿��ƣ��ӵ������Ͻǵ����ʱ�뿪ʼ�������̶�Ӧ�����IDΪ1 ~ 4�������ĸ�����Ŀ����ǹ���CAN1�ϣ�
 *         PID��Ӧ�Ĳ������ڴ������Ӹ��ص�����µ��Եõ���
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

uint8_t Chassis_Ctrl; //���̿��Ʊ�־λ
chassis_t  Chassis;  /*���ڵ���*/

pid_t  Moto_Chassis_Pid_Pos[4];  //���̵��λ�û�PID�ṹ��
pid_t  Moto_Chassis_Pid_Spd[4];  //���̵���ٶȻ�PID�ṹ��
pid_t  Chassis_Rotate;  				 //������תPID�ṹ��


void Chassis_PID_Init()
{
    memset(&Chassis, 0, sizeof(chassis_t));
//    memset(&remote_control, 0, sizeof(RC_Type));
    
    ChassisRampInit(); //����б�³�ʼ��
    
    /*�����ĸ�����ٶȻ���ʼ��PID����*/
    for(int i=0; i<4; i++)
    {
        PID_struct_init(&Moto_Chassis_Pid_Spd[i], POSITION_PID, 15000, 500, 12.0f, 0.15f, 2.0f);
    }
		
    //���̸�����תPID������ʼ��
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
	
    if(!Gimbal_Init_Flag||(Gimbal_Motor_Encoder[YAW_CAN2_205].msg_cnt < 50)) /*��̨��ʼ����ʹ��̨�ص�����ǰ�������*/
    {
        GimbalInit();
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
    else if(Gimbal_Init_Flag) /*��̨��ʼ�����*/
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); /*��ʼ����ɣ��ɽ��п��ƣ�ָʾ����*/
        if(RC_UPPER_RIGHT_SW_UP) /*ң�������ϲ������ϣ���������*/
        {
            
            ChassisNormalCalc();  //�������
            
        }
        else if(RC_UPPER_RIGHT_SW_MID) /*ң�������ϲ����Ƶ��м䣬����*/
        {
            ChassisFollowCalc(); //����
            
//            ChassisSpeedCalc(); //С����
            
        }
        else if(RC_UPPER_RIGHT_SW_DOWN) /*ң�������ϲ����Ƶ����棬С����*/
        {
            ChassisSpeedCalc(); //С����
            
        }
//        /*��̨��ֵ*/
//        GimbalControlSetValue();
//        //������ֵ
//        ShootControlSetValue();
        
    }
		
}

void ChassisNormalCalc()
{
	  int16_t /*chassis_vx_channel,*/ chassis_vy_channel, chassis_vw_channel;

    /*ң������������*/
    //chassis_vx_channel = RcDeadlineLimit(remote_control.ch1, 10)*CHASSIS_MAXSPEED_RPM/660;
    chassis_vy_channel = RcDeadlineLimit(remote_control.ch2, 10)*CHASSIS_MAXSPEED_RPM/660;
    chassis_vw_channel = RcDeadlineLimit(remote_control.ch1, 10)*CHASSIS_MAXSPEED_RPM/660;


/****************************************б�²���*********************************************************/
        //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
        //FirstOrderFilterCali(&chassis_cmd_slow_set_vx, chassis_vx_channel);
        FirstOrderFilterCali(&chassis_cmd_slow_set_vy, chassis_vy_channel);

        //Chassis.vx = chassis_cmd_slow_set_vx.out; //����ƽ��
        Chassis.vy = chassis_cmd_slow_set_vy.out; //ǰ���˶�
        Chassis.vw = chassis_vw_channel; //��ת

/*******************************************END***********************************************************/
        
        //J_Scope_rcinput_x = Chassis.vx;
        //J_Scope_rcinput_y = Chassis.vy;
        
        Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
        Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
}

/*���̸�����̨ģʽ*/
void ChassisFollowCalc(void)
{
    
    Chassis.vw = ChassisRotationPID(-GimbalData[YAW].relative_angle, 0);
    
    Chassis.vx = remote_control.ch1*CHASSIS_MAXSPEED_RPM/660; //����ƽ��
    Chassis.vy = remote_control.ch2*CHASSIS_MAXSPEED_RPM/660; //ǰ���˶�
//    Chassis.vw = remote_control.ch3*CHASSIS_MAXSPEED_RPM/660; //��ת
    
    Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
    Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
    Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
    Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
    
}

void ChassisSpeedCalc(void)
{
    float sin_rotate, cos_rotate;
    
    /*�������ϵ������̨����ϵ�����ٶȵ��趨*/
    Chassis.vx = remote_control.ch1*ROTATE_MOVE_MAXSPEED/660; //�����ƶ�
    Chassis.vy = remote_control.ch2*ROTATE_MOVE_MAXSPEED/660; //ǰ���ƶ�
    Chassis.vw = CHASSIS_ROTATE_RAD_SPEED; //������ת�ٶ�

    /*ͨ����ת���󽫴������ϵ������̨����ϵ���µ��ٶ�ת��Ϊ��������ϵ������������ϵ���µ��ٶ�*/
    sin_rotate = arm_sin_f32(GimbalData[YAW].relative_angle_rad);
    cos_rotate = arm_cos_f32(GimbalData[YAW].relative_angle_rad);
    Chassis.car_vx = -Chassis.vx*cos_rotate - Chassis.vy*sin_rotate; //X�᷽����ٶ�
    Chassis.car_vy = -Chassis.vx*sin_rotate + Chassis.vy*cos_rotate; //Y�᷽����ٶ�
    Chassis.car_vw = Chassis.vw; //��ת�ٶ�

    /*�������Ӷ�Ӧ�Ľ��ٶȣ����ٶ����ڳ�������ϵ�µ�*/
    Chassis.wheel_rad_201 = (-Chassis.car_vx - Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //����
    Chassis.wheel_rad_202 = (-Chassis.car_vx + Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //����
    Chassis.wheel_rad_203 = (+Chassis.car_vx + Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //����
    Chassis.wheel_rad_204 = (+Chassis.car_vx - Chassis.car_vy + (CHASSIS_LENGTH_LX + CHASSIS_LENGTH_LY)*Chassis.car_vw)/CHASSIS_WHEEL_RADIUS; //����

    /*�����������ٶ�ת��Ϊתÿ���ӣ���ΪPID������*/
    Chassis.fr_motor_rpm_201 = Chassis.wheel_rad_201/RPM_To_Rads;
    Chassis.fl_motor_rpm_202 = Chassis.wheel_rad_202/RPM_To_Rads;
    Chassis.rl_motor_rpm_203 = Chassis.wheel_rad_203/RPM_To_Rads;
    Chassis.rr_motor_rpm_204 = Chassis.wheel_rad_204/RPM_To_Rads;
    
}

/*������תPID����*/
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


