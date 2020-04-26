#ifndef LIMIT_SWITCH_H__
#define LIMIT_SWITCH_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "mytype.h"
/* 类型定义 --------------------------------------------------------------*/
typedef enum
{
  KEY_UP   = 1,
  KEY_DOWN = 0,
}KEYState_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/



#define KEY0_GPIO_PIN                 GPIO_PIN_15
#define KEY0_GPIO                     GPIOD
#define KEY0_DOWN_LEVEL               0  /* 根据原理图设计，KEY3按下时引脚为低电平，所以这里设置为0 */


/* 函数声明 ------------------------------------------------------------------*/
void KEY0_StateRead(void);



#endif  

