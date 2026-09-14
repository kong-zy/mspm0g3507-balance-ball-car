#include "Application/app_ball_pid.h"

#include <limits.h>

/*
 * PID内部最多按150ms处理一次视觉间隔。
 *
 * 正常情况下K230每20～40ms发送一帧。如果通信恢复后的dt异常大而不限制，
 * 积分量会在一次调用中突然增加，导致舵机跳动。
 */
#define APP_BALL_PID_MAX_DT_TICKS_10MS    (15U)

static int32_t AppBallPid_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t AppBallPid_Limit32(
    int32_t value,
    int32_t min_value,
    int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/*
 * 使用64位中间结果完成乘除。
 *
 * PID最终数据仍然保存为32位，但积分累计量与系数相乘时可能暂时超过
 * 32位范围。使用64位中间结果可以避免参数稍微调大后发生整数溢出。
 */
static int32_t AppBallPid_MultiplyDivide(
    int32_t value,
    int32_t numerator,
    int32_t denominator)
{
    int64_t result;

    if (denominator <= 0) {
        return 0;
    }

    result = (int64_t)value * (int64_t)numerator;
    result /= (int64_t)denominator;

    if (result > INT32_MAX) {
        return INT32_MAX;
    }
    if (result < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)result;
}

void AppBallPid_Reset(AppBallPid_Controller *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral_0p1mm_ms = 0;
    pid->last_error_0p1mm = 0;
    pid->p_output_us = 0;
    pid->i_output_us = 0;
    pid->d_output_us = 0;
    pid->output_us = 0;
}

void AppBallPid_Init(
    AppBallPid_Controller *pid,
    const AppBallPid_Config *config)
{
    if ((pid == 0) || (config == 0)) {
        return;
    }

    pid->config = *config;
    AppBallPid_Reset(pid);
}

void AppBallPid_ResetIntegral(AppBallPid_Controller *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral_0p1mm_ms = 0;
    pid->i_output_us = 0;
}

int32_t AppBallPid_Update(
    AppBallPid_Controller *pid,
    int16_t reference_0p1mm,
    int16_t position_0p1mm,
    int16_t speed_0p1mm_per_s,
    uint32_t dt_ticks_10ms,
    uint8_t integral_allowed)
{
    int32_t error;
    int32_t p_error;
    int32_t p_output;
    int32_t i_output;
    int32_t d_output;
    int32_t output;
    int32_t integral_limit;
    int32_t dt_ms;
    uint8_t integral_enabled;

    if (pid == 0) {
        return 0;
    }

    /*
     * dt为0通常表示同一视觉帧被重复处理。为了保证积分时间含义正确，
     * 将它修正为一个10ms节拍；异常过大的dt则限制为150ms。
     */
    if (dt_ticks_10ms == 0U) {
        dt_ticks_10ms = 1U;
    } else if (dt_ticks_10ms > APP_BALL_PID_MAX_DT_TICKS_10MS) {
        dt_ticks_10ms = APP_BALL_PID_MAX_DT_TICKS_10MS;
    }
    dt_ms = (int32_t)dt_ticks_10ms * 10;

    error = (int32_t)reference_0p1mm - (int32_t)position_0p1mm;
    p_error = error;

    /*
     * 小误差、低速度时关闭P项的微小来回修正，降低舵机抖动。
     * 注意积分仍使用原始error，因此真实的固定静差仍会被慢慢消除。
     */
    if ((AppBallPid_Abs32(error) <=
         pid->config.position_deadband_0p1mm) &&
        (AppBallPid_Abs32(speed_0p1mm_per_s) <=
         pid->config.deadband_speed_0p1mm_per_s)) {
        p_error = 0;
    }

    /*
     * 条件积分：
     *
     * 只有视觉有效、位置已接近目标、钢球速度也较低时才累积积分。
     * 远距离运动和高速制动阶段由PD完成，避免大误差造成积分饱和。
     */
    integral_enabled =
        ((integral_allowed != 0U) &&
         (AppBallPid_Abs32(error) <=
          pid->config.integral_enable_error_0p1mm) &&
         (AppBallPid_Abs32(speed_0p1mm_per_s) <=
          pid->config.integral_enable_speed_0p1mm_per_s)) ? 1U : 0U;

    if (integral_enabled != 0U) {
        int64_t next_integral =
            (int64_t)pid->integral_0p1mm_ms +
            (int64_t)error * (int64_t)dt_ms;

        /*
         * 根据允许的最大I输出反推积分累计量上限：
         *
         *   integral_limit =
         *       I最大输出 * Ki分母 / Ki分子
         *
         * 这样调Ki以后，I项的最大舵机作用仍由integral_output_limit_us控制。
         */
        if ((pid->config.ki_numerator > 0) &&
            (pid->config.ki_denominator > 0)) {
            int64_t limit64 =
                (int64_t)pid->config.integral_output_limit_us *
                (int64_t)pid->config.ki_denominator /
                (int64_t)pid->config.ki_numerator;

            if (limit64 > INT32_MAX) {
                integral_limit = INT32_MAX;
            } else {
                integral_limit = (int32_t)limit64;
            }

            if (next_integral > integral_limit) {
                next_integral = integral_limit;
            } else if (next_integral < -integral_limit) {
                next_integral = -integral_limit;
            }
        }

        /*
         * 即使Ki暂时被配置为0，也保证累计值能安全存入32位变量。
         * 这样现场把Ki关闭用于单独调P、D时，不会因为长时间误差而溢出。
         */
        if (next_integral > INT32_MAX) {
            next_integral = INT32_MAX;
        } else if (next_integral < INT32_MIN) {
            next_integral = INT32_MIN;
        }

        pid->integral_0p1mm_ms = (int32_t)next_integral;
    } else if (pid->config.integral_decay_denominator > 0) {
        /*
         * 离开积分区时让旧积分逐渐衰减，不在一帧内突然清零。
         * 目标折返、边界保护和故障状态会由上层主动立即清零。
         */
        pid->integral_0p1mm_ms = AppBallPid_MultiplyDivide(
            pid->integral_0p1mm_ms,
            pid->config.integral_decay_numerator,
            pid->config.integral_decay_denominator);
    } else {
        pid->integral_0p1mm_ms = 0;
    }

    p_output = AppBallPid_MultiplyDivide(
        p_error,
        pid->config.kp_numerator,
        pid->config.kp_denominator);

    i_output = AppBallPid_MultiplyDivide(
        pid->integral_0p1mm_ms,
        pid->config.ki_numerator,
        pid->config.ki_denominator);
    i_output = AppBallPid_Limit32(
        i_output,
        -pid->config.integral_output_limit_us,
        pid->config.integral_output_limit_us);

    d_output = AppBallPid_MultiplyDivide(
        speed_0p1mm_per_s,
        pid->config.kd_numerator,
        pid->config.kd_denominator);

    /*
     * speed带有方向：
     *
     *   speed > 0：钢球向正方向运动，-D会减小正向输出；
     *   speed < 0：钢球向负方向运动，-D会形成正向制动力。
     */
    output = p_output + i_output - d_output;

    pid->last_error_0p1mm = (int16_t)AppBallPid_Limit32(
        error, INT16_MIN, INT16_MAX);
    pid->p_output_us = (int16_t)AppBallPid_Limit32(
        p_output, INT16_MIN, INT16_MAX);
    pid->i_output_us = (int16_t)AppBallPid_Limit32(
        i_output, INT16_MIN, INT16_MAX);
    pid->d_output_us = (int16_t)AppBallPid_Limit32(
        d_output, INT16_MIN, INT16_MAX);
    pid->output_us = (int16_t)AppBallPid_Limit32(
        output, INT16_MIN, INT16_MAX);

    return output;
}
