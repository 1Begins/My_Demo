#ifndef LIMIT_SWITCH_H__
#define LIMIT_SWITCH_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "mytype.h"
/* ���Ͷ��� --------------------------------------------------------------*/
typedef enum
{
  KEY_UP   = 1,
  KEY_DOWN = 0,
}KEYState_TypeDef;

/* �궨�� --------------------------------------------------------------------*/



#define KEY0_GPIO_PIN                 GPIO_PIN_15
#define KEY0_GPIO                     GPIOD
#define KEY0_DOWN_LEVEL               0  /* ����ԭ��ͼ��ƣ�KEY3����ʱ����Ϊ�͵�ƽ��������������Ϊ0 */


/* �������� ------------------------------------------------------------------*/
void KEY0_StateRead(void);



#endif  

