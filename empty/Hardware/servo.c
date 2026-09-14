#include "servo.h"

float Servo_Position_KP = 20.0f;
float Servo_Position_KD = 5.0f;

/* 两个舵机的当前位置和目标位置，单位为 us 脉宽。 */
float Servo_Position1 = SERVO_CENTER_PULSE_US;
float Servo_Position2 = SERVO_CENTER_PULSE_US;
float Servo_Target1 = SERVO_CENTER_PULSE_US;
float Servo_Target2 = SERVO_CENTER_PULSE_US;

static float s_servo_last_bias1 = 0.0f;
static float s_servo_last_bias2 = 0.0f;

static float Servo_LimitPosition(float value)
{
    if (value > (float)SERVO_MAX_PULSE_US) {
        return (float)SERVO_MAX_PULSE_US;
    }
    if (value < (float)SERVO_MIN_PULSE_US) {
        return (float)SERVO_MIN_PULSE_US;
    }
    return value;
}

/*
 * 只在写PWM比较寄存器时四舍五入到整数微秒。
 * Servo_Position1/2始终保留小数部分，避免0.2~0.9us的连续微调每次都
 * 被强制截断为0，造成“长时间不动，误差变大后突然跳一下”的卡顿。
 */
static void Servo_WriteCurrentPosition(void)
{
    uint32_t pulse1 = (uint32_t)(Servo_Position1 + 0.5f);
    uint32_t pulse2 = (uint32_t)(Servo_Position2 + 0.5f);

    DL_Timer_setCaptureCompareValue(
        SERVO_1_PWM_INST, pulse1, GPIO_SERVO_1_PWM_C2_IDX);
    DL_Timer_setCaptureCompareValue(
        SERVO_2_PWM_INST, pulse2, GPIO_SERVO_2_PWM_C1_IDX);
}

uint16_t Servo_LimitPulse(uint16_t value, uint16_t min, uint16_t max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

uint16_t Servo_AngleToPulse(uint8_t angle_deg)
{
    uint32_t pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

    if (angle_deg > SERVO_MAX_ANGLE_DEG) {
        angle_deg = SERVO_MAX_ANGLE_DEG;
    }

    return (uint16_t)(SERVO_MIN_PULSE_US +
        ((pulse_range * (uint32_t)angle_deg) / SERVO_MAX_ANGLE_DEG));
}

float Servo_PositionPID1(float position, float target)
{
    float bias = target - position;
    float pwm = (
        Servo_Position_KP * bias +
        Servo_Position_KD * (bias - s_servo_last_bias1)) / 100.0f;

    s_servo_last_bias1 = bias;
    return pwm;
}

float Servo_PositionPID2(float position, float target)
{
    float bias = target - position;
    float pwm = (
        Servo_Position_KP * bias +
        Servo_Position_KD * (bias - s_servo_last_bias2)) / 100.0f;

    s_servo_last_bias2 = bias;
    return pwm;
}

void Servo_SetPulse(uint16_t servo1_us, uint16_t servo2_us)
{
    Servo_Position1 = Servo_LimitPulse(servo1_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    Servo_Position2 = Servo_LimitPulse(servo2_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    s_servo_last_bias1 = 0.0f;
    s_servo_last_bias2 = 0.0f;
    Servo_WriteCurrentPosition();
}

void Servo_SetTarget(float target1, float target2)
{
    Servo_Target1 = Servo_LimitPosition(target1);
    Servo_Target2 = Servo_LimitPosition(target2);
}

void Servo_SetAngle(uint8_t servo1_deg, uint8_t servo2_deg)
{
    uint8_t mapped_angle1 = servo1_deg;
    uint8_t mapped_angle2 = servo2_deg;

    if (mapped_angle1 > SERVO_MAX_ANGLE_DEG) {
        mapped_angle1 = SERVO_MAX_ANGLE_DEG;
    }
    if (mapped_angle2 > SERVO_MAX_ANGLE_DEG) {
        mapped_angle2 = SERVO_MAX_ANGLE_DEG;
    }

#if SERVO_1_ANGLE_REVERSED
    mapped_angle1 = (uint8_t)(SERVO_MAX_ANGLE_DEG - mapped_angle1);
#endif
#if SERVO_2_ANGLE_REVERSED
    mapped_angle2 = (uint8_t)(SERVO_MAX_ANGLE_DEG - mapped_angle2);
#endif

    Servo_SetTarget(
        Servo_AngleToPulse(mapped_angle1),
        Servo_AngleToPulse(mapped_angle2));
}

void Servo_Update(void)
{
    float pwm1 = Servo_PositionPID1(Servo_Position1, Servo_Target1);
    float pwm2 = Servo_PositionPID2(Servo_Position2, Servo_Target2);
    float next1 = Servo_LimitPosition(Servo_Position1 + pwm1);
    float next2 = Servo_LimitPosition(Servo_Position2 + pwm2);

    /* 防止D项在目标附近造成越界；越过目标时直接贴到目标值。 */
    if (((Servo_Target1 >= Servo_Position1) && (next1 > Servo_Target1)) ||
        ((Servo_Target1 <= Servo_Position1) && (next1 < Servo_Target1))) {
        next1 = Servo_Target1;
    }
    if (((Servo_Target2 >= Servo_Position2) && (next2 > Servo_Target2)) ||
        ((Servo_Target2 <= Servo_Position2) && (next2 < Servo_Target2))) {
        next2 = Servo_Target2;
    }

    Servo_Position1 = next1;
    Servo_Position2 = next2;
    Servo_WriteCurrentPosition();
}

void Servo_Init(void)
{
    DL_Timer_startCounter(SERVO_1_PWM_INST);
    DL_Timer_startCounter(SERVO_2_PWM_INST);
    Servo_SetPulse(SERVO_CENTER_PULSE_US, SERVO_CENTER_PULSE_US);
}
