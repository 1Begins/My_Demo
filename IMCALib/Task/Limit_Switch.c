#include "Limit_Switch.h"
#include "main.h"
#include "Gimbal_Task.h"

void KEY0_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==KEY0_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==KEY0_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
      //return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  //return KEY_UP;
}


