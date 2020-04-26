#ifndef __pid_H
#define __pid_H


#include "stm32f4xx_hal.h"


enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};


/*PID结构体*/
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];	//目标值，NOW, LAST, LLAST
    float get[3];	//测量值
    float err[3];	//误差
	
    
    float pout;		//P输出				
    float iout;		//I输出					
    float dout;		//D输出		
    
    float pos_out;			//本次位置式输出，即 pos_out = pout + iout + dout
    float last_pos_out;		//上次位置式输出
    float delta_u;			//本次增量值
    float delta_out;		//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;   //上次增量式输出
    
    float max_err;              //最大偏差
    float deadband;				//err < deadband return
    uint32_t pid_mode;          //PID模式，分位置式和增量式
    uint32_t MaxOutput;			//输出限幅
    uint32_t IntegralLimit;		//积分限幅
    
    void (*f_param_init)(struct __pid_t *pid,  /*定义PID参数初始化的函数指针*/
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);   //pid参数修改

}pid_t;


/*定义PID结构体初始化函数*/
void PID_struct_init(pid_t* pid, uint32_t mode, uint32_t maxout, uint32_t intergral_limit, float kp, float ki, float kd);
/*PID计算函数*/
float pid_calc(pid_t* pid, float fdb, float ref);
		
#endif

