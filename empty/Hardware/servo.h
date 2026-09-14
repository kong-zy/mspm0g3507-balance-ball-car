#ifndef _SERVO_H
#define _SERVO_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 舵机脉宽范围，单位为 us；与原工程 750~2250 的控制范围保持一致。 */
#define SERVO_MIN_PULSE_US      (750U)
#define SERVO_CENTER_PULSE_US   (1500U)
#define SERVO_MAX_PULSE_US      (2250U)
#define SERVO_MIN_ANGLE_DEG     (0U)
#define SERVO_CENTER_ANGLE_DEG  (90U)
#define SERVO_MAX_ANGLE_DEG     (180U)

/*
 * 1号滚球舵机机械安装旋转了180度，因此逻辑角度与PWM方向相反。
 * 测试页面仍显示原来的逻辑角度：角度增加时，1号舵机实际脉宽减小。
 * 2号舵机没有改变安装方向，继续使用正常映射。
 */
#define SERVO_1_ANGLE_REVERSED  (1U)
#define SERVO_2_ANGLE_REVERSED  (0U)

extern float Servo_Position1;
extern float Servo_Position2;
extern float Servo_Target1;
extern float Servo_Target2;

void Servo_Init(void);
void Servo_SetPulse(uint16_t servo1_us, uint16_t servo2_us);
void Servo_SetAngle(uint8_t servo1_deg, uint8_t servo2_deg);
void Servo_SetTarget(float target1, float target2);
void Servo_Update(void);
uint16_t Servo_AngleToPulse(uint8_t angle_deg);
float Servo_PositionPID1(float position, float target);
float Servo_PositionPID2(float position, float target);
uint16_t Servo_LimitPulse(uint16_t value, uint16_t min, uint16_t max);

#endif
