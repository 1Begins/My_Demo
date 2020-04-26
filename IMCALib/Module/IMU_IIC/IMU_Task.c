#include "IMU_Task.h"
#include "IOI2C.h"
#include "stdio.h"


IMU_t IMU_Yaw; /*陀螺仪yaw轴结构体*/


void GetImuAngle(IMU_t *IMU)
{
    //static int8_t flag,flag_out; 
    
    IICreadBytes(0x50, AX, 24,&chrTemp[0]); /*模拟IIC读取陀螺仪数据*/
    
    /*更新数据*/
    IMU->last_angle = IMU->angle;
    IMU->speed_gyro = -CharToShort(&chrTemp[6])/32768.f*2000.f; /*读取角速度，加负号是为了使它的正负和电机的转速的正负相同*/
    IMU->angle = (float)CharToShort(&chrTemp[22])/32768*180; /*获得IMU的yaw角度值，对应陀螺仪的Z轴*/
    IMU->angle_diff = IMU->angle - IMU->last_angle; /*直接作差*/
    if(IMU->angle_diff < -240) /*顺时针，过180度处理*/
    {
        IMU->round_cnt++;
//        IMU->angle_diff_process = 360 + IMU->angle - IMU->last_angle;   //(angle + 180)+(180 - last_angle)
    }
    else if(IMU->angle_diff > 240)  /*逆时针，过180度处理*/
    {
        IMU->round_cnt--;
//        IMU->angle_diff_process = 360 + IMU->last_angle - IMU->angle;   //(180 - angle)+(180 + last_angle)
    }
//    else
//    {
//        IMU->angle_diff_process = IMU->angle_diff;
//    }
    
    IMU->total_angle = IMU->angle + IMU->round_cnt*360.0f;
//    IMU->total_angle += IMU->angle_diff_process; /*得到IMU转过的角度值*/
    
}

