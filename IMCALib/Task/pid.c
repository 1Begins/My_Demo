/*************************************************************************************************************
 * @file			pid.c
 * @brief   		对于PID, 反馈/测量习惯性叫get/measure/real/fdb,
 *					期望值输入一般叫set/target/ref
************************************************************************************************************/
  
  
#include "pid.h"
#include <math.h>
#include "Chassis_Task.h"
#include "Gimbal_Task.h"


#define ABS(x)		((x>0)? (x): (-x))  /*取绝对值*/



/**********************************************************************************************************************
 * @brief   数值限幅
 * @param   float *a        输入数据的指针变量
 *          float ABS_MAX   最值
 * @retval  None
**********************************************************************************************************************/
void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}


/**********************************************************************************************************************
 * @brief   PID参数初始化
 * @param   pid_t* pid                  PID结构体
 *          uint32_t mode               PID模式选择，分位置式PID和增量式PID
 *          uint32_t maxout             输出限幅
 *          uint32_t intergral_limit    积分限幅
 *          float kp, ki, kd            PID的三个参数
 * @retval  None
**********************************************************************************************************************/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd )
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}


/*************************调试参数赋值******************************/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}


/**********************************************************************************************************************
 * @brief   calculate delta PID and position PID
 * @param   pid_t* pid：PID结构体
 *          set: target
 *          get：measure
 * @retval  None
 * @others  PID相关资料：https://blog.csdn.net/as480133937/article/details/89508034
**********************************************************************************************************************/
float pid_calc(pid_t* pid, float get, float set)
{
    
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	/*set - measure，得到偏差*/
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;   /*如果偏差大于最大偏差就跳出PID计算*/
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;  /*偏差在死区范围内不作处理，跳出PID计算*/
    
    if(pid->pid_mode == POSITION_PID) /*位置式PID计算*/
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);  /*积分限幅*/
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);  /*限定输出值的大小*/
        pid->last_pos_out = pid->pos_out;	/*update last time*/ 
    }
    else if(pid->pid_mode == DELTA_PID) /*增量式PID计算*/
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit); /*积分限幅*/
        pid->delta_u = pid->pout + pid->iout + pid->dout; /*本次增量值*/
        pid->delta_out = pid->last_delta_out + pid->delta_u; /*本次增量式输出*/
        abs_limit(&(pid->delta_out), pid->MaxOutput); /*输出限幅*/
        pid->last_delta_out = pid->delta_out;	/*update last time*/
    }
    
    /*更新数据*/
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out; /*PID输出*/
	
}



/**********************************************************************************************************************
 * @brief   PID总体的初始化
 * @param   pid_t* pid                  PID结构体
 *          uint32_t mode               PID模式选择，分位置式PID和增量式PID
 *          uint32_t maxout             输出限幅
 *          uint32_t intergral_limit    积分限幅
 *          float kp, ki, kd            PID的三个参数
 * @retval  None
**********************************************************************************************************************/
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init; /*将pid_param_init函数的首地址赋值给指针变量f_param_init*/
    pid->f_pid_reset = pid_reset;
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}

