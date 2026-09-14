#ifndef APPLICATION_APP_BALL_PID_H_
#define APPLICATION_APP_BALL_PID_H_

#include <stdint.h>

/*
 * 钢球位置PID控制器
 * ============================================================================
 *
 * 本模块只负责根据“参考位置、实际位置、钢球速度”计算舵机控制偏移量，
 * 不负责状态机、串口通信、舵机方向映射和画面边界保护。
 *
 * 为了与K230通信数据和现有要求三程序直接兼容，本模块统一采用以下单位：
 *
 *   位置：       0.1mm
 *   速度：       0.1mm/s
 *   时间：       10ms节拍
 *   控制输出：   舵机脉宽偏移量，单位us
 *
 * 例如：
 *
 *   50.0mm 传入 500
 *   15.0mm/s 传入 150
 *   两帧间隔30ms，dt_ticks_10ms传入3
 *
 * 控制公式为：
 *
 *   error = reference - position
 *   output = Kp * error + Ki * integral - Kd * speed
 *
 * D项直接使用钢球实际速度，不对error求差分。这样从正目标折返到负目标时，
 * 不会因为目标值突然改变而产生很大的“微分冲击”。
 */

typedef struct
{
    /*
     * 比例系数使用“分子/分母”表示，避免Cortex-M0+执行浮点运算。
     *
     * P输出(us) = error(0.1mm) * kp_numerator / kp_denominator
     */
    int32_t kp_numerator;
    int32_t kp_denominator;

    /*
     * 积分累计量的单位是“0.1mm * ms”。
     *
     * I输出(us) =
     *     integral_0p1mm_ms * ki_numerator / ki_denominator
     */
    int32_t ki_numerator;
    int32_t ki_denominator;

    /*
     * 速度微分项：
     *
     * D输出(us) =
     *     speed(0.1mm/s) * kd_numerator / kd_denominator
     *
     * 最终计算时从P和I中减去D，因此钢球向正方向运动得越快，
     * 控制器越早减小正向倾角；向负方向运动时同理。
     */
    int32_t kd_numerator;
    int32_t kd_denominator;

    /*
     * 只有同时满足以下两个条件才允许累积积分：
     *
     *   |error| <= integral_enable_error_0p1mm
     *   |speed| <= integral_enable_speed_0p1mm_per_s
     *
     * 这样远离目标时主要由PD负责快速移动，接近目标后才由I消除静差。
     */
    int32_t integral_enable_error_0p1mm;
    int32_t integral_enable_speed_0p1mm_per_s;

    /*
     * 积分项最终最多允许产生多少us的舵机偏移量。
     * 这是防止积分饱和的核心限制，建议明显小于总输出限幅。
     */
    int32_t integral_output_limit_us;

    /*
     * 不满足积分开启条件时，不立刻清空历史积分，而是按以下比例衰减：
     *
     *   integral = integral * numerator / denominator
     *
     * 推荐95/100。目标方向切换、任务重新开始、故障保护时仍应主动清零。
     */
    int32_t integral_decay_numerator;
    int32_t integral_decay_denominator;

    /*
     * 当误差和速度都非常小时，将P项使用的误差视为0，减少视觉坐标
     * 轻微抖动造成的舵机来回动作。真实误差仍可通过小积分慢慢修正。
     */
    int32_t position_deadband_0p1mm;
    int32_t deadband_speed_0p1mm_per_s;
} AppBallPid_Config;

typedef struct
{
    AppBallPid_Config config;

    /* 积分累计量，单位：0.1mm * ms。 */
    int32_t integral_0p1mm_ms;

    /*
     * 保存最近一次PID三个分量，便于后续OLED显示或串口调参。
     * 当前要求三控制器只读取最终输出，但保留这些数据能明显方便排查。
     */
    int16_t last_error_0p1mm;
    int16_t p_output_us;
    int16_t i_output_us;
    int16_t d_output_us;
    int16_t output_us;
} AppBallPid_Controller;

/*
 * 使用指定参数初始化控制器，并清除积分和上一次计算结果。
 * config必须在整个调用过程中保持参数合法，所有分母必须大于0。
 */
void AppBallPid_Init(
    AppBallPid_Controller *pid,
    const AppBallPid_Config *config);

/*
 * 清除全部PID历史，适用于：
 *
 *   1. 系统上电初始化；
 *   2. 重新开始一次要求三测试；
 *   3. 通信故障或长时间丢球后进入安全状态。
 */
void AppBallPid_Reset(AppBallPid_Controller *pid);

/*
 * 只清除积分，保留其它参数。
 *
 * 从正目标切换到负目标时必须调用，避免正向阶段积累的积分继续推动钢球
 * 向正方向运动。进入软/硬边界保护时也应该清除积分。
 */
void AppBallPid_ResetIntegral(AppBallPid_Controller *pid);

/*
 * 计算一次PID输出。
 *
 * integral_allowed：
 *   1 = 上层确认视觉位置可信，而且钢球已接近最终目标，允许按条件积分；
 *   0 = 禁止继续积分，并让旧积分衰减。
 *
 * 上层除了检查视觉有效，还应使用“实际位置与最终目标的误差”判断是否
 * 已进入目标附近。不能只看参考位置误差，否则参考轨迹经过钢球当前位置时，
 * 可能在距离最终目标仍很远的情况下提前积累积分。
 *
 * dt_ticks_10ms：
 *   当前视觉帧与上一有效视觉帧之间的时间，单位10ms。
 *   函数内部会限制异常过大的时间，避免恢复通信后一次性累积大量积分。
 *
 * 返回值：
 *   尚未执行边界保护和分区限幅的舵机抽象控制偏移量，单位us。
 *   正输出表示希望钢球向坐标正方向运动，实际舵机正反向由app_ball.c映射。
 */
int32_t AppBallPid_Update(
    AppBallPid_Controller *pid,
    int16_t reference_0p1mm,
    int16_t position_0p1mm,
    int16_t speed_0p1mm_per_s,
    uint32_t dt_ticks_10ms,
    uint8_t integral_allowed);

#endif /* APPLICATION_APP_BALL_PID_H_ */
