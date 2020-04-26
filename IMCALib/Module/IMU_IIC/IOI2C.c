/******************************************************************************
*功　能:读取陀螺仪模块
输入	Get_Imu_Data(&ImuData);
输出	模块解算后数据（JY901/HWT101）后者漂移更低精度更高
IIC接口	 IIC_SCL    PBout(8) 
		 IIC_SDA    PBout(9) 	 
说明	使用需移植IOI2C.c和IOI2C.h两文件，并在要使用的文件里包含该头文件，
		直接调用函数Get_Imu_Data();使用（调用一次函数，数据更新一次）
		或者放在定时器里在给定的频率上更新数据。
*******************************************************************************/ 



#include "IOI2C.h"

unsigned char chrTemp[30];
stuff ImuData;

//JY901模块数据读取函数
void Get_Imu_Data(stuff *s)
{
	
	//IIC读取命令  IIC从AX即 0x34 X轴加速度开始读取数据，一直到0x3f Z轴角度
	//24→字长，读取到的数据分高位和低位，读取了12个寄存器的数据
	IICreadBytes(0x50, AX, 24,&chrTemp[0]);
	
	s->a[0] = (float)CharToShort(&chrTemp[0])/32768*16;          //计算加速度 （x、y、z轴）
	s->a[1] = (float)CharToShort(&chrTemp[2])/32768*16;
	s->a[2] = (float)CharToShort(&chrTemp[4])/32768*16;
    
	s->w[0] = (float)CharToShort(&chrTemp[6])/32768*2000;		  //计算角速度
	s->w[1] = (float)CharToShort(&chrTemp[8])/32768*2000;
	s->w[2] = (float)CharToShort(&chrTemp[10])/32768*2000;
    
	s->h[0] = CharToShort(&chrTemp[12]);							//计算磁场
	s->h[1] = CharToShort(&chrTemp[14]);
	s->h[2] = CharToShort(&chrTemp[16]);
    
	s->Angle[0] = (float)CharToShort(&chrTemp[18])/32768*180;		//计算角度
	s->Angle[1] = (float)CharToShort(&chrTemp[20])/32768*180;
	s->Angle[2] = (float)CharToShort(&chrTemp[22])/32768*180;
    
}

	
void ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}


short CharToShort(unsigned char cData[])
{
	return (short)(cData[1]<<8|cData[0]);
} 


void Delay(u32 count)//用于产生400KHzIIC信号所需要的延时
{
	count = count*10;
	while (count--);
}

/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{			
	//SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
}


/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	
	Delay(5);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	
	Delay(5);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}


/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	
		Delay(5);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	
		Delay(5);							   	
}


/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	SDA_IN();      //SDA设置为输入  
 	IIC_SDA=1;
		Delay(5);	  
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
		Delay(5);
	}  
	IIC_SCL=1;
	Delay(5); 
	IIC_SCL=0;//时钟输出0  
	return 0;  
} 


/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
		Delay(5);
	IIC_SCL=1;
		Delay(5);
	IIC_SCL=0;
}
	

/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	
		Delay(5);
	IIC_SCL=1;
		Delay(5);
	IIC_SCL=0;
}					 				     


/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t; 
		SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
			
		Delay(2);   
		IIC_SCL=1;
		Delay(5);
		IIC_SCL=0;	
		Delay(3);
    }	 
} 	 
   

/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        
		Delay(5);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		
		Delay(5); 
    }					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
}


/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)+1);  //进入接收模式	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
		 	else  data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
	}
    IIC_Stop();//产生一个停止条件
    return count;
}


/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
 }
	IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
	
}
