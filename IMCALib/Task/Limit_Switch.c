#include "Limit_Switch.h"
#include "main.h"
#include "Gimbal_Task.h"

void KEY0_StateRead(void)
{
  /* ��ȡ��ʱ����ֵ���ж��Ƿ��Ǳ�����״̬������Ǳ�����״̬���뺯���� */
  if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==KEY0_DOWN_LEVEL)
  {
    /* ��ʱһС��ʱ�䣬�������� */
    HAL_Delay(10);
    /* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
    if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==KEY0_DOWN_LEVEL)
    {
      /* �ȴ������������˳�����ɨ�躯�� */
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
      //return KEY_DOWN;
    }
  }
  /* ����û�����£�����û������״̬ */
  //return KEY_UP;
}


