#include "PrintfInfo.h"
#include "CanBus_Task.h"
#include "RC_Task.h"
#include "stdio.h"


int a = 0;

void PrintfInfo(void)
{
	if(a == 100)
	{
        
	/*��ӡң������ͨ����Ϣ*/
//	printf("\n\r The RC ch1.ch2.ch3.ch4 is:%d  %d  %d  %d\n\r",remote_control.ch1,remote_control.ch2,remote_control.ch3,remote_control.ch4);
//	printf("\n\r The RC switch_left and switch_right is:%d  %d\n\r",remote_control.switch_left,remote_control.switch_right);
  
	/*��ӡ�������Ϣ*/
//  printf("\n\r The motor angle and speed_rpm is:%d  %d\n\r",moto_chassis[0].angle,moto_chassis[0].speed_rpm);  //��ӡ�����ת�ӻ�е�ǶȺ�ת��ת��
//	printf("\n\r The motor current is:%f\n\r",moto_chassis[0].real_current);  //��ӡ���ת�ص���
//	printf("\n\r The motor hall is:%d\n\r",moto_chassis[0].hall);  //��ӡ����¶�
		
	a = 0;
	}
	
	/*ע�����ת�ӻ�е�Ƕ�ֵ��Χ��0 ~ 8192
	      ת��ת��ֵ�ĵ�λΪ��rpm
	      ����¶ȵĵ�λΪ�����϶�
	*/

}

