#include "user_lib.h"

int32_t catch_angle;






/*************************Sin函数插值*********************************/
float Sin_up_table[200+2] = {0,6.1684e-05,0.00024672,0.00055506,0.00098664,0.0015413,0.002219,0.0030195,0.0039426,0.0049882,0.0061558,0.0074453,0.0088564,0.010389,0.012042,0.013815,0.015708,0.017721,0.019853,0.022103,0.024472,0.026957,0.02956,0.032278,0.035112,0.03806,0.041123,0.044298,0.047586,0.050986,0.054497,0.058117,0.061847,0.065684,0.069629,0.07368,0.077836,0.082096,0.08646,0.090925,0.095492,0.10016,0.10492,0.10978,0.11474,0.1198,0.12494,0.13018,0.13552,0.14094,0.14645,0.15204,0.15773,0.16349,0.16934,0.17528,0.18129,0.18738,0.19355,0.19979,0.20611,0.2125,0.21896,0.22549,0.23209,0.23875,0.24548,0.25227,0.25912,0.26604,0.273,0.28003,0.28711,0.29424,0.30143,0.30866,0.31594,0.32326,0.33063,0.33804,0.34549,0.35298,0.3605,0.36806,0.37566,0.38328,0.39093,0.39861,0.40631,0.41404,0.42178,0.42955,0.43733,0.44513,0.45295,0.46077,0.4686,0.47645,0.48429,0.49215,0.5,0.50785,0.51571,0.52355,0.5314,0.53923,0.54705,0.55487,0.56267,0.57045,0.57822,0.58596,0.59369,0.60139,0.60907,0.61672,0.62434,0.63194,0.6395,0.64702,0.65451,0.66196,0.66937,0.67674,0.68406,0.69134,0.69857,0.70576,0.71289,0.71997,0.727,0.73396,0.74088,0.74773,0.75452,0.76125,0.76791,0.77451,0.78104,0.7875,0.79389,0.80021,0.80645,0.81262,0.81871,0.82472,0.83066,0.83651,0.84227,0.84796,0.85355,0.85906,0.86448,0.86982,0.87506,0.8802,0.88526,0.89022,0.89508,0.89984,0.90451,0.90907,0.91354,0.9179,0.92216,0.92632,0.93037,0.93432,0.93815,0.94188,0.9455,0.94901,0.95241,0.9557,0.95888,0.96194,0.96489,0.96772,0.97044,0.97304,0.97553,0.9779,0.98015,0.98228,0.98429,0.98618,0.98796,0.98961,0.99114,0.99255,0.99384,0.99501,0.99606,0.99698,0.99778,0.99846,0.99901,0.99944,0.99975,0.99994,1,1};
int32_t Sin_up_y1;  
int32_t Sin_up_y0;
int32_t Sin_up_x1; 
int32_t Sin_up_x0;//每一段都清零
//int32_t Sin_up_x;   //每一段都清零
//int32_t Sin_up_y;
//int Sin_up_count;   //S型曲线上升计数 
/*******************Lifting_DOWN************************/	
float Sin_down_table[200+2] = {1,0.99994,0.99975,0.99944,0.99901,0.99846,0.99778,0.99698,0.99606,0.99501,0.99384,0.99255,0.99114,0.98961,0.98796,0.98618,0.98429,0.98228,0.98015,0.9779,0.97553,0.97304,0.97044,0.96772,0.96489,0.96194,0.95888,0.9557,0.95241,0.94901,0.9455,0.94188,0.93815,0.93432,0.93037,0.92632,0.92216,0.9179,0.91354,0.90907,0.90451,0.89984,0.89508,0.89022,0.88526,0.8802,0.87506,0.86982,0.86448,0.85906,0.85355,0.84796,0.84227,0.83651,0.83066,0.82472,0.81871,0.81262,0.80645,0.80021,0.79389,0.7875,0.78104,0.77451,0.76791,0.76125,0.75452,0.74773,0.74088,0.73396,0.727,0.71997,0.71289,0.70576,0.69857,0.69134,0.68406,0.67674,0.66937,0.66196,0.65451,0.64702,0.6395,0.63194,0.62434,0.61672,0.60907,0.60139,0.59369,0.58596,0.57822,0.57045,0.56267,0.55487,0.54705,0.53923,0.5314,0.52355,0.51571,0.50785,0.5,0.49215,0.48429,0.47645,0.4686,0.46077,0.45295,0.44513,0.43733,0.42955,0.42178,0.41404,0.40631,0.39861,0.39093,0.38328,0.37566,0.36806,0.3605,0.35298,0.34549,0.33804,0.33063,0.32326,0.31594,0.30866,0.30143,0.29424,0.28711,0.28003,0.273,0.26604,0.25912,0.25227,0.24548,0.23875,0.23209,0.22549,0.21896,0.2125,0.20611,0.19979,0.19355,0.18738,0.18129,0.17528,0.16934,0.16349,0.15773,0.15204,0.14645,0.14094,0.13552,0.13018,0.12494,0.1198,0.11474,0.10978,0.10492,0.10016,0.095492,0.090925,0.08646,0.082096,0.077836,0.07368,0.069629,0.065684,0.061847,0.058117,0.054497,0.050986,0.047586,0.044298,0.041123,0.03806,0.035112,0.032278,0.02956,0.026957,0.024472,0.022103,0.019853,0.017721,0.015708,0.013815,0.012042,0.010389,0.0088564,0.0074453,0.0061558,0.0049882,0.0039426,0.0030195,0.002219,0.0015413,0.00098664,0.00055506,0.00024672,6.1684e-05,0,0};
int32_t Sin_down_x1;
int32_t Sin_down_x0; //每一段都清零
//int32_t Sin_down_x;  //每一段都清零
//int32_t Sin_down_y;
//int Sin_down_count;   //S型曲线下降计数
/*********************************************************************/	
    
//int Sin_point = 200;  //查表法为200个点
//int Sin_control_time_up = 3;  //S型曲线，每经过Sin_control_time  ms就跳到下一个插值
//int Sin_control_time_down = 3;  //S型曲线，每经过Sin_control_time  ms就跳到下一个插值


    
//void Sin_up_y_calc(void)
//{
//	/*公式
//		     (y1-y0)(x-x0)         (y1-y0)=(Sin_up_table[Sin_up_count+1]-Sin_up_table[Sin_up_count])   (x-x0)=(Sin_up_x-Sin_up_x0)=Sin_up_x   
//		y = ---------------	+ y0   y0=Sin_up_table[Sin_up_count]     
//					 (x1-x0)             (x1-x0)= Sin_control_time  
//	*/

//    
//			Sin_up_y = catch_angle*( ((Sin_up_table[Sin_up_count+1]-Sin_up_table[Sin_up_count])*(Sin_up_x))/Sin_control_time_up + Sin_up_table[Sin_up_count] );
//			Sin_up_x++;
//			if(Sin_up_x >Sin_control_time_up)
//			{
//				 Sin_up_x = 1;
//				 Sin_up_count++;
//			}
//			if(Sin_up_count>Sin_point)
//			{
//					Sin_up_count = Sin_point;
//			}
//			ref_angle0 =  Sin_up_y; //+ lifting_angle_small;
////			ref_angle3_lifting =  Sin_up_y ;//+ lifting_angle_small;
//			Sin_down_count = 0;
//			Sin_down_x = 0;
//}



//void Sin_down_y_calc(void)
//{

//			Sin_down_y = (-catch_angle)*( ((Sin_down_table[Sin_down_count+1]-Sin_down_table[Sin_down_count])*(Sin_down_x))/Sin_control_time_down + Sin_down_table[Sin_down_count] );  
//			Sin_down_x++;
//			if(Sin_down_x >Sin_control_time_down)
//			{
//				 Sin_down_x = 1;
//				 Sin_down_count++;
//			}
//			if(Sin_down_count>Sin_point)
//			{
//					Sin_down_count = Sin_point;
//			}
//			ref_angle0 =  Sin_down_y ;//+ lifting_angle_small;
////			ref_angle3_lifting =  Sin_down_y + lifting_angle_small;
//			Sin_up_count = 0;
//			Sin_up_x = 0;
//}



//void Sin_Up_or_Down_switch(int32_t variation)   //抬升下降选择函数
//{
//    if(variation > 0)//上升
//    {
//        
//        Sin_up_y = variation*( ((Sin_up_table[Sin_up_count+1]-Sin_up_table[Sin_up_count])*(Sin_up_x))/Sin_control_time_up + Sin_up_table[Sin_up_count] );
//        Sin_up_x++;
//        if(Sin_up_x >Sin_control_time_up)
//        {
//             Sin_up_x = 1;
//             Sin_up_count++;
//        }
//        if(Sin_up_count>Sin_point)
//        {
//            Sin_up_count = Sin_point;
//        }
//        ref_angle0 =  Sin_up_y; 
//        Sin_down_count = 0;
//        Sin_down_x = 0;
//        
//    }
//    else if (variation < 0)//下降  
//    {
//        
//        Sin_down_y = (-variation)*( ((Sin_down_table[Sin_down_count+1]-Sin_down_table[Sin_down_count])*(Sin_down_x))/Sin_control_time_down + Sin_down_table[Sin_down_count] );  
//        Sin_down_x++;
//        if(Sin_down_x >Sin_control_time_down)
//        {
//             Sin_down_x = 1;
//             Sin_down_count++;
//        }
//        if(Sin_down_count>Sin_point)
//        {
//            Sin_down_count = Sin_point;
//        }
//        ref_angle0 =  Sin_down_y ;
//        Sin_up_count = 0;
//        Sin_up_x = 0;
//            
//    }
//    
//}



//int32_t last_variation_out;

/*所给的值必须是先正后负*/
int32_t SinRampVariation_1(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down)
{
    
    int32_t variation_out;
    const int Sin_point = 200;
    static int Sin_up_count, Sin_down_count, Sin_up_x, Sin_down_x;
    
    
    if(variation > 0)//上升
    {
        
        variation_out = variation*( ((Sin_up_table[Sin_up_count+1]-Sin_up_table[Sin_up_count])*(Sin_up_x))/Sin_control_time_up + Sin_up_table[Sin_up_count] );
        Sin_up_x++;
        if(Sin_up_x >Sin_control_time_up)
        {
             Sin_up_x = 1;
             Sin_up_count++;
        }
        if(Sin_up_count>Sin_point)
        {
            Sin_up_count = Sin_point;
        }
        if(variation_out == variation)
        {
           Sin_down_count = 0;
           Sin_down_x = 0;
            
        }
        
    }
    else if (variation < 0)//下降  
    {
        
        variation_out = (-variation)*( ((Sin_down_table[Sin_down_count+1]-Sin_down_table[Sin_down_count])*(Sin_down_x))/Sin_control_time_down + Sin_down_table[Sin_down_count] );  
        Sin_down_x++;
        if(Sin_down_x >Sin_control_time_down)
        {
             Sin_down_x = 1;
             Sin_down_count++;
        }
        if(Sin_down_count>Sin_point)
        {
            Sin_down_count = Sin_point;
        }
        if(variation_out == 0)
        {
            Sin_up_count = 0;
            Sin_up_x = 0;
            
        }
            
    }
    
    return variation_out;
    
}



/*先负后正*/
int32_t SinRampVariation_2(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down)
{
    
    int32_t variation_out;
    const int Sin_point = 200;
    static int Sin_up_count, Sin_down_count, Sin_up_x, Sin_down_x;
    
    
    if(variation < 0)//上升
    {
        
        variation_out = variation*( ((Sin_up_table[Sin_up_count+1]-Sin_up_table[Sin_up_count])*(Sin_up_x))/Sin_control_time_up + Sin_up_table[Sin_up_count] );
        Sin_up_x++;
        if(Sin_up_x >Sin_control_time_up)
        {
             Sin_up_x = 1;
             Sin_up_count++;
        }
        if(Sin_up_count>Sin_point)
        {
            Sin_up_count = Sin_point;
        }
        if(variation_out == variation)
        {
           Sin_down_count = 0;
           Sin_down_x = 0;
            
        }
        
    }
    else if (variation > 0)//下降  
    {
        
        variation_out = (-variation)*( ((Sin_down_table[Sin_down_count+1]-Sin_down_table[Sin_down_count])*(Sin_down_x))/Sin_control_time_down + Sin_down_table[Sin_down_count] );  
        Sin_down_x++;
        if(Sin_down_x >Sin_control_time_down)
        {
             Sin_down_x = 1;
             Sin_down_count++;
        }
        if(Sin_down_count>Sin_point)
        {
            Sin_down_count = Sin_point;
        }
        if(variation_out == 0)
        {
            Sin_up_count = 0;
            Sin_up_x = 0;
            
        }
            
    }
    
    return variation_out;
    
}






SinRampState  sin_ramp_state;   //斜坡函数状态标记

/**********************************************************************************************************************
 * @brief    sin斜坡函数
 * @param    int32_t variation                需要变化的值的大小，正数表示变量增大，负数表示变量减小
 *           uint8_t Sin_control_time_up      相当于上升的坡度
 *           uint8_t Sin_control_time_down    相当于下降的坡度
 * @other    可以实现连续的斜坡，正负都可以，但使用时要做相应的标记位设置
**********************************************************************************************************************/
int32_t SinRampVariation_3(int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down)
{
    
    int32_t variation_out;
    const int Sin_point = 200;
    static int Sin_up_count, Sin_down_count, Sin_up_x, Sin_down_x, last_out;
    
    
    if(variation > 0)//正转
    {
        if((sin_ramp_state.compare_value_up < variation)&&(sin_ramp_state.sin_ramp_switch == 1))
        {
            
            variation_out = variation*(((Sin_up_table[Sin_up_count+1]-Sin_up_table[Sin_up_count])*(Sin_up_x))/Sin_control_time_up + Sin_up_table[Sin_up_count]) + last_out;
            Sin_up_x++;
            if(Sin_up_x >Sin_control_time_up)
            {
                 Sin_up_x = 1;
                 Sin_up_count++;
            }
            if(Sin_up_count>Sin_point)
            {
                Sin_up_count = Sin_point;
            }
            
            if(variation_out == variation + last_out)
            {
               Sin_up_count = 0;
               Sin_up_x = 0;
                
               sin_ramp_state.compare_value_up = variation;
               last_out += variation;//每次的变化量累加
            }
            
        }
        else
        {
            variation_out = last_out;//保持到达设定的位置
        }
        
    }
    else if (variation < 0)//反转
    {
        if((sin_ramp_state.compare_value_dowm > variation)&&(sin_ramp_state.sin_ramp_switch == 1))
        {
            variation_out = (-variation)*(((Sin_down_table[Sin_down_count+1]-Sin_down_table[Sin_down_count])*(Sin_down_x))/Sin_control_time_down + Sin_down_table[Sin_down_count]) + (last_out + variation);  
            Sin_down_x++;
            if(Sin_down_x >Sin_control_time_down)
            {
                 Sin_down_x = 1;
                 Sin_down_count++;
            }
            if(Sin_down_count>Sin_point)
            {
                Sin_down_count = Sin_point;
            }
            if(variation_out == last_out + variation)
            {
                Sin_down_count = 0;
                Sin_down_x = 0;
                
                sin_ramp_state.compare_value_dowm = variation;
                last_out += variation;
            }
            
        }
        else
        {
            variation_out = last_out;
        }
            
    }
    return variation_out;
    
}


/*int16_t类型限幅函数*/
int16_t Int16Constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}


/*int32_t类型限幅函数*/
int32_t Int32Constrain(int32_t Value, int32_t minValue, int32_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}


/*float类型限幅函数*/
float FloatConstrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}


/*float类型循环限幅函数*/
float LoopFloatConstrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}


