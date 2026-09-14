#include "encoder.h"
#include "led.h"
uint32_t gpio_interrup1,gpio_interrup2;
int Get_Encoder_countA,Get_Encoder_countB;
/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
void GROUP1_IRQHandler(void)
{
	//获取中断信号
    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
    
    
	/*
	 * A 侧编码器正交解码。
	 * 任一相产生边沿中断时读取另一相电平，据此判断当前转动方向，
	 * 并对本控制周期内的 A 侧脉冲计数执行加一或减一。
	 */
	if((gpio_interrup1 & ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1B_PIN))
		{
			Get_Encoder_countA--;
		}
		else
		{
			Get_Encoder_countA++;
		}
	}
	else if((gpio_interrup1 & ENCODERA_E1B_PIN)==ENCODERA_E1B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1A_PIN))
		{
			Get_Encoder_countA++;
		}
		else
		{
			Get_Encoder_countA--;
		}
	}
	
	/*
	 * B 侧编码器正交解码。
	 * B 侧机械安装方向与 A 侧不同，因此主控制程序读取计数后会统一修正符号；
	 * 此处仍按编码器两相信号的实际相位关系累计原始方向计数。
	 */
	if((gpio_interrup2 & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2B_PIN))
		{
			Get_Encoder_countB--;
		}
		else
		{
			Get_Encoder_countB++;
		}
	}
	else if((gpio_interrup2 & ENCODERB_E2B_PIN)==ENCODERB_E2B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2A_PIN))
		{
			Get_Encoder_countB++;
		}                 
		else              
		{                 
			Get_Encoder_countB--;
		}
	}
	DL_GPIO_clearInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
	DL_GPIO_clearInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
}
/*******************************************************
函数功能：计算编码器转速 (RPM) 
入口参数：encoder_count - 编码器计数值
         sample_time_ms - 采样时间间隔(毫秒)
返回  值：转速值(RPM)
说明：基于2倍频解码和13线编码器计算转速，30减速比
***********************************************************/
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms) 
{
	//更换电机需修改此处参数
    const int ENCODER_LINES = 13;        // 编码器线数 (每转13个脉冲)
    const int MULTIPLY_FACTOR = 2;       // 2倍频系数 (只检测上升沿)
    const int GEAR_RATIO = 30;           // 减速比 30:1
    // 计算每转的脉冲数 = 线数 × 倍频系数
    int pulses_per_revolution = ENCODER_LINES * MULTIPLY_FACTOR; // 13 × 2 = 26
    
    // 电机轴转速计算公式：RPM = (脉冲计数 × 60000) / (每转脉冲数 × 采样时间ms)
    // 60000 = 60秒 × 1000毫秒，用于单位转换
    float motor_rpm = (float)encoder_count * 60000.0f / (pulses_per_revolution * sample_time_ms);
    
    return motor_rpm/GEAR_RATIO;//电机转速除以减速比得到输出轴的转速
}
