#include "motor.h"

/*
 * 速度环每 10ms 执行一次，误差单位为“本周期编码器计数”。
 *
 * 原来的 Kp=1.3、Ki=0.4 对整数 PWM（范围约为 ±7000）来说过小：
 * 目标速度改变 1~2 个计数时，每个周期只能增加几个 PWM，重车需要很长时间
 * 才能真正改变轮速，所以传感器虽然已经识别到偏差，实车仍表现为微调迟钝。
 *
 * 当前参数让比例项在一个控制周期内给出明显的 PWM 改变量，积分项用于
 * 补偿车重、摩擦和左右电机差异。参数从过于激进的 250/15 适当降低，
 * 避免中心附近的微小目标变化导致左右轮反复过度修正。
 */
float Velcity_Kp = 150.0f;
float Velcity_Ki = 12.0f;
float Velcity_Kd = 0.0f;

/*
 * 增量式 PI 的内部状态放在文件作用域，便于停车或重新启动时统一清零。
 * 如果不复位，A/B 电机可能继承上一次转弯留下的不同累计 PWM，即使新一次
 * 启动时目标速度完全相同，小车也会在起步瞬间向一侧偏转。
 */
static int g_control_velocity_a = 0;
static int g_control_velocity_b = 0;
static int g_last_bias_a = 0;
static int g_last_bias_b = 0;
/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2024-07-019

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2024-07-019

All rights reserved
***********************************************/
int limit_PWM(int value,int low,int high)
{
	if(value>high) return high;
	else if(value<low) return low;
	else return value;
}

void Motor_PID_Reset(void)
{
    /*
     * 同时清除左右电机的 PWM 累计值和历史误差。
     * 调用后下一次 Velocity_A/B() 会从相同的零状态重新建立闭环输出。
     */
    g_control_velocity_a = 0;
    g_control_velocity_b = 0;
    g_last_bias_a = 0;
    g_last_bias_b = 0;
}


void Set_PWM(int pwmA,int pwmB)
{
	 if(pwmA>0)
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C0_IDX);
    }
    else
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C0_IDX);
    }
    if(pwmB>0)
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C1_IDX);
    }
    else
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN2_PIN);
		 DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C1_IDX);
    }
   

}

/***************************************************************************
函数功能：电机的PID闭环控制
入口参数：左右电机的编码器值
返回值  ：电机的PWM
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //定义相关变量
	Bias=TargetVelocity-CurrentVelocity; //求速度偏差
	/*
	 * 标准增量式 PI：
	 *   PWM(k) = PWM(k-1) + Kp[e(k)-e(k-1)] + Ki*e(k)
	 * Kp 让目标速度或实际速度发生变化时立即调整 PWM；
	 * Ki 持续消除负载和摩擦造成的稳态速度误差。
	 */
	g_control_velocity_a +=
	    Velcity_Kp * (Bias - g_last_bias_a) + Velcity_Ki * Bias;
	g_last_bias_a = Bias;
	if(g_control_velocity_a > 7000) g_control_velocity_a = 7000;
	else if(g_control_velocity_a < -7000) g_control_velocity_a = -7000;
	return g_control_velocity_a; //返回速度控制值
}

/***************************************************************************
函数功能：电机的PID闭环控制
入口参数：左右电机的编码器值
返回值  ：电机的PWM
***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
	int Bias;  //定义相关变量
	Bias=TargetVelocity-CurrentVelocity; //求速度偏差
	/*
	 * B 电机使用与 A 电机相同的增量式 PI 结构和参数。
	 * 两侧分别保存历史误差和 PWM 累计值，不会互相影响。
	 */
	g_control_velocity_b +=
	    Velcity_Kp * (Bias - g_last_bias_b) + Velcity_Ki * Bias;
	g_last_bias_b = Bias;
	if(g_control_velocity_b > 7000) g_control_velocity_b = 7000;
	else if(g_control_velocity_b < -7000) g_control_velocity_b = -7000;
	return g_control_velocity_b; //返回速度控制值
}

