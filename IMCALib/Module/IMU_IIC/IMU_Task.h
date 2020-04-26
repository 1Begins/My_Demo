#ifndef __IMU_TASK_
#define __IMU_TASK_

#include "mytype.h"


#define GYRO_SPEED_DIFF  3900

typedef struct{
    float angle; //������yaw���ʵʱ�Ƕ�
    float speed_gyro; //�����Ƿ������ٶ�
    float last_speed_gyro; //�ϴ������Ƿ������ٶ�
    float speed_gyro_diff; //�������ε��ٶȱ仯��
    float last_angle; //��һ�νǶ�
    float angle_diff; //ǰ������ֱ������õ��Ĳ�ֵ
    float angle_diff_process; //���������ĽǶȲ�ֵ
    float total_angle; //ת�����ܽǶ�
    float offset_relative_angle; //�ϵ�����������ڵ���ǰ�������ĵ���ԽǶ�
    int32_t round_cnt; //Ȧ��
    
}IMU_t;


extern IMU_t IMU_Yaw;

void GetImuAngle(IMU_t *IMU);



#endif
