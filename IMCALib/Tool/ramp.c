#include "ramp.h"

/*一阶低通滤波结构体*/
first_order_filter_type_t chassis_cmd_slow_set_vx;
first_order_filter_type_t chassis_cmd_slow_set_vy;


/*************************************************************************************************************
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
*************************************************************************************************************/
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/*************************************************************************************************************
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
*************************************************************************************************************/
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}


//底盘斜坡初始化
void ChassisRampInit(void)
{
    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //用一阶滤波代替斜坡函数生成
    FirstOrderFilterInit(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    FirstOrderFilterInit(&chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    
}


