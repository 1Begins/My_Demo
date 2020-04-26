#include "PrintfInfo.h"
#include "CanBus_Task.h"
#include "RC_Task.h"
#include "stdio.h"


int a = 0;

void PrintfInfo(void)
{
	if(a == 100)
	{
        
	/*打印遥控器各通道信息*/
//	printf("\n\r The RC ch1.ch2.ch3.ch4 is:%d  %d  %d  %d\n\r",remote_control.ch1,remote_control.ch2,remote_control.ch3,remote_control.ch4);
//	printf("\n\r The RC switch_left and switch_right is:%d  %d\n\r",remote_control.switch_left,remote_control.switch_right);
  
	/*打印电机的信息*/
//  printf("\n\r The motor angle and speed_rpm is:%d  %d\n\r",moto_chassis[0].angle,moto_chassis[0].speed_rpm);  //打印电机的转子机械角度和转子转度
//	printf("\n\r The motor current is:%f\n\r",moto_chassis[0].real_current);  //打印电机转矩电流
//	printf("\n\r The motor hall is:%d\n\r",moto_chassis[0].hall);  //打印电机温度
		
	a = 0;
	}
	
	/*注：电机转子机械角度值范围：0 ~ 8192
	      转子转速值的单位为：rpm
	      电机温度的单位为：摄氏度
	*/

}

