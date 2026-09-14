#include "Hardware/board.h"
#include "Application/app_ball.h"
#include "Application/app_ball_pid.h"

/*
 * ============================================================================
 * H题要求3：静止小车上的一维滚球控制
 * ============================================================================
 *
 * 坐标和单位：
 *   摆杆中心O为0；
 *   朝题目规定的+5cm方向为正；
 *   K230传入位置和本文件目标值都使用0.1mm作为一个计数。
 *
 * 因此：
 *   +5cm = +50mm = +500个0.1mm计数；
 *   -5cm = -50mm = -500个0.1mm计数；
 *   允许误差1cm = 10mm = 100个0.1mm计数。
 *
 * 控制结构：
 *   O点快速确认 -> K230摄像头位置 -> 预测位置 -> 分阶段PID
 *   -> 摆杆舵机目标脉宽
 *
 * D项直接使用相邻视觉帧估算的钢球速度，作用是抑制钢球冲过目标点。
 * 不在串口中断里计算PID，避免图像数据到达时阻塞按键和电机中断。
 */

/* ------------------------- 机械安装相关参数 ------------------------- */

/*
 * 默认使用1号舵机控制摆杆，2号舵机保持中位。
 * 如果实物接在2号舵机，把这里改成2。
 */
#define BALL_CONTROL_SERVO_ID                 (1U)

/*
 * 摆杆水平时的舵机脉宽。1500us只是常见中位，必须在SERVO页面实测：
 * 当凹槽真正水平且钢球放在中间不会明显滚动时，把对应脉宽写到这里。
 */
#define BALL_SERVO_CENTER_US                  ((int16_t)SERVO_CENTER_PULSE_US)

/*
 * The cam/preload mechanism does not have one repeatable mechanical center.
 * Requirement 3 starts from the common 1500 us center and lets the final
 * -5 cm hold
 * trim its own balance point slowly inside this safe interval.
 */
#define BALL_NEGATIVE_BALANCE_MIN_US          (BALL_SERVO_CENTER_US - 75)
#define BALL_NEGATIVE_BALANCE_MAX_US          (BALL_SERVO_CENTER_US + 15)
#define BALL_CENTER_TUNE_MIN_US               (BALL_SERVO_CENTER_US - 175)
#define BALL_CENTER_TUNE_MAX_US               (BALL_SERVO_CENTER_US + 75)

/*
 * 舵机方向：
 *   +1：脉宽增大时，钢球应向题目+方向运动；
 *   -1：脉宽增大时，钢球向题目-方向运动。
 *
 * 第一次必须架车低幅测试。如果钢球越控越远，立即把+1改成-1。
 */
#define BALL_SERVO_DIRECTION                  (1)

/*
 * 舵机标定为750~2250us对应0~180度。
 *
 * 正常MOVE阶段先使用5度（约41us）；只有视觉连续确认钢球没有运动时，
 * 才按0.5度小步提高，推动最高8度（约66us）。一旦钢球开始运动，额外
 * 推动力立即逐步撤回。只有反向制动和越界保护允许达到12度（约100us）。
 */
#define BALL_SERVO_BASE_MAX_ANGLE_DEG         (8U)
#define BALL_SERVO_ADAPTIVE_MAX_ANGLE_DEG     (9U)
#define BALL_SERVO_BRAKE_MAX_ANGLE_DEG        (30U)
#define BALL_SERVO_BASE_MAX_OFFSET_US         \
    (((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * \
      BALL_SERVO_BASE_MAX_ANGLE_DEG) / SERVO_MAX_ANGLE_DEG)
#define BALL_SERVO_ADAPTIVE_MAX_OFFSET_US     \
    (((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * \
      BALL_SERVO_ADAPTIVE_MAX_ANGLE_DEG) / SERVO_MAX_ANGLE_DEG)
#define BALL_SERVO_MAX_OFFSET_US              \
    (((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * \
      BALL_SERVO_BRAKE_MAX_ANGLE_DEG) / SERVO_MAX_ANGLE_DEG)

/* -------------------------- 启动自动调平 -------------------------- */

/*
 * 自动调平原理：
 *
 * 水管有微小倾角时，即使不给位置PID，小球也会持续向较低一端漂移。
 * 程序在要求三正式运动前观察这个漂移速度：
 *
 *   小球向正方向漂移 -> 水平补偿向负方向增加1us；
 *   小球向负方向漂移 -> 水平补偿向正方向增加1us。
 *
 * 补偿量会加入后续PID的舵机中值，因此能修正“1500us并不真正水平”和
 * 左侧机械死区造成的中值偏差。补偿限制为±35us，防止识别异常时越调越偏。
 */
#define BALL_LEVEL_TRIM_LIMIT_US              (35)
#define BALL_LEVEL_TRIM_STEP_US               (1)
#define BALL_LEVEL_ADJUST_INTERVAL_TICKS_10MS (2U)
#define BALL_LEVEL_DRIFT_SPEED_0P1MM_PER_S    (80)

/*
 * 钢球已在O点时只快速复核水管水平：最短观察100ms，稳定确认80ms，
 * 最长300ms。原来的600ms调平加上严格到达判定容易挤占5秒运动时间。
 */
#define BALL_LEVEL_MIN_TICKS_10MS             (10U)
#define BALL_LEVEL_CONFIRM_TICKS_10MS         (8U)
#define BALL_LEVEL_MAX_TICKS_10MS             (30U)

/*
 * 调平时只允许小球位于中心±15mm内参与“稳定完成”判断，避免在远离
 * 中心的位置错误学习水平中值。
 */
#define BALL_LEVEL_POSITION_WINDOW_0P1MM      (150)

/* -------------------------- 要求3目标与判定 -------------------------- */

#define BALL_CENTER_TARGET_0P1MM              (0)
#define BALL_POSITIVE_TARGET_0P1MM            (500)
#define BALL_NEGATIVE_TARGET_0P1MM            (-500)

/*
 * 当前测试开始时钢球已经放在O点。中心确认收紧到±5mm，并把确认时间
 * 缩短为50ms，避免在正确起点上无意义地消耗要求三的5秒预算。
 */
#define BALL_CENTER_TOLERANCE_0P1MM           (50)
#define BALL_CENTER_SPEED_MAX_0P1MM_PER_S     (200)
#define BALL_CENTER_CONFIRM_TICKS_10MS        (5U)

/*
 * 摄像头、串口和舵机存在延迟，因此K230实测位置达到+47mm便认为已经
 * 到达+5cm允许误差带，下一控制拍立即切换到-5cm。这里不再等待钢球先
 * 在正端停稳；目标切换后的强反制同时承担“正端刹车”和“返程启动”。
 */
#define BALL_POSITIVE_SWITCH_POSITION_0P1MM   (450)
#define BALL_POSITIVE_MOVE_LIMIT_US           (45)

/*
 * -5cm最终同样按±5mm、速度不超过10mm/s连续稳定300ms判定完成。
 * 完成后仍以-50mm为设定值继续闭环，不会因为判定带放宽而改变目标。
 */
#define BALL_NEGATIVE_CAPTURE_TOLERANCE_0P1MM (50)
#define BALL_NEGATIVE_CAPTURE_SPEED_0P1MM_PER_S (150)
#define BALL_FINAL_TOLERANCE_0P1MM            (30)
#define BALL_FINAL_SPEED_MAX_0P1MM_PER_S      (60)
#define BALL_FINAL_STABLE_TICKS_10MS          (40U)
#define BALL_COMPLETE_HOLD_TICKS_10MS         (300U)

/* Final -5 cm balance: continuous position-to-speed cascade. */
#define BALL_NEGATIVE_HOLD_DEADBAND_0P1MM     (30)
#define BALL_NEGATIVE_HOLD_TRIM_LIMIT_0P1MM   (60)
#define BALL_NEGATIVE_HOLD_TRIM_SPEED_0P1MM_PER_S (150)
#define BALL_NEGATIVE_HOLD_TRIM_TICKS_10MS    (8U)
#define BALL_NEGATIVE_HOLD_POSITION_KP_NUM    (3)
#define BALL_NEGATIVE_HOLD_POSITION_KP_DEN    (2)
#define BALL_NEGATIVE_HOLD_SPEED_MAX_0P1MM_PER_S (300)
#define BALL_NEGATIVE_HOLD_SPEED_KP_DEN       (12)
#define BALL_NEGATIVE_HOLD_DEADZONE_TRIGGER_0P1MM (60)
#define BALL_NEGATIVE_HOLD_DEADZONE_US        (35)
#define BALL_NEGATIVE_HOLD_MAX_OFFSET_US      (50)
#define BALL_NEGATIVE_HOLD_DRIVE_STEP_US      (2)
#define BALL_NEGATIVE_HOLD_BRAKE_STEP_US      (4)
#define BALL_NEGATIVE_HOLD_EXIT_0P1MM         (100)

/* 按键启动即开始计时，达到500个10ms节拍就是赛题规定的5秒。 */
#define BALL_TASK_TIMEOUT_TICKS_10MS          (500U)

/* 启动后最多等待500ms的第一帧有效图像，随后进入通信故障保护。 */
#define BALL_START_WAIT_TICKS_10MS            (50U)

/* BALL帧持续到达但连续200ms没有识别到钢球，进入丢球保护。 */
#define BALL_INVALID_HOLD_TICKS_10MS          (8U)
#define BALL_LOST_TICKS_10MS                  (20U)

/* 质量低于40的识别结果不参与闭环，减少阴影误识别造成的舵机突跳。 */
#define BALL_MIN_QUALITY                      (40U)

/* ---------------------- PID外围控制与安全参数 ---------------------- */

/*
 * 速度轨迹参数，单位均使用0.1mm、0.1mm/s和0.1mm/s^2。
 *
 * O->+5cm最高60mm/s，+5cm->-5cm最高75mm/s。允许速度按
 * v=sqrt(2*a*d)随剩余距离自动下降，制动能力保守按70mm/s^2估计，
 * 让程序比原方案更早进入反向制动。
 */
#define BALL_CRUISE_POS_SPEED_0P1MM_PER_S     (450)
#define BALL_CRUISE_NEG_SPEED_0P1MM_PER_S     (700)
#define BALL_PLANNER_BRAKE_ACCEL_0P1MM_PER_S2 (700)
#define BALL_OVERSPEED_MARGIN_0P1MM_PER_S     (100)
#define BALL_TARGET_BRAKE_MIN_SPEED_0P1MM_PER_S (100)

/*
 * 推动和制动使用非对称角度：正常推动最多8度，只有确认钢球正在冲向
 * 目标/越过目标时，反向制动才允许达到12度。
 */
#define BALL_SPEED_BRAKE_MIN_US               (75)
#define BALL_SPEED_BRAKE_MAX_US               (100)

/* Fast upward/negative kick immediately after +5 cm turns toward -5 cm. */
#define BALL_NEGATIVE_LIFT_US                 (90)
#define BALL_NEGATIVE_LIFT_TICKS_10MS         (12U)

/*
 * 从+5cm切换到-5cm后，如果小球仍在正侧且尚未形成至少5mm/s的负向
 * 返回速度，说明舵机还处在克服正向惯性的阶段。此时直接给12度反制，
 * 直到小球明确开始返程，再交回正常返程轨迹控制。
 */
/*
 * 四个控制阶段由“最终目标误差和钢球速度”共同决定：
 *
 *   MOVE：    |误差|>20mm，允许快速接近；
 *   BRAKE：   尚未进入±5mm，或速度仍大于20mm/s，提前制动；
 *   CAPTURE： 已接近目标但误差/速度还没有进入保持带；
 *   HOLD：    |误差|<=4mm且|速度|<=8mm/s，使用小幅PID和条件积分保持；
 *             进入后采用8mm/20mm/s滞回退出门限，避免视觉噪声反复切换。
 *
 * 每个阶段使用独立PID参数、输出限幅和变化率，避免移动阶段的大输出
 * 被带进目标附近。向外加速时慢变，输出与当前速度反向时允许更快制动。
 */
#define BALL_PHASE_MOVE_ERROR_0P1MM           (200)
#define BALL_PHASE_BRAKE_ERROR_0P1MM          (50)
#define BALL_PHASE_CAPTURE_ERROR_0P1MM        (40)
#define BALL_PHASE_BRAKE_SPEED_0P1MM_PER_S    (200)
#define BALL_PHASE_CAPTURE_SPEED_0P1MM_PER_S  (80)
#define BALL_HOLD_EXIT_ERROR_0P1MM            (80)
#define BALL_HOLD_EXIT_SPEED_0P1MM_PER_S      (200)

/*
 * 远处使用接近5度的最大倾角提高运动幅度；越接近目标，允许角度越小。
 * 这样既能缩短0->+5cm和+5cm->-5cm的移动时间，又不会把MOVE的大角度
 * 带进最终-5cm保持阶段。
 */
#define BALL_OUTPUT_MOVE_US                   (70)
#define BALL_OUTPUT_BRAKE_US                  (85)
#define BALL_OUTPUT_CAPTURE_US                (35)
#define BALL_OUTPUT_HOLD_US                   (12)

/*
 * -5cm保持阶段不使用固定频率的盲目震荡。K230检测到小球仍有超过
 * 5mm/s的微小速度时，按速度方向叠加5~10us反向阻尼；速度反向时舵机
 * 修正方向也随之反向，形成以视觉反馈为依据的小幅往返动作。
 */
#define BALL_HOLD_DAMPING_MIN_SPEED_0P1MM_PER_S (50)
#define BALL_HOLD_DAMPING_MIN_US              (2)
#define BALL_HOLD_DAMPING_MAX_US              (4)

/*
 * 视觉不动检测与破静摩擦补偿：
 *   目标仍相距15mm以上；
 *   连续120ms位置总变化不足1.5mm且慢速估计低于8mm/s；
 *   从5度开始，每100ms增加约0.5度（4us），最高8度。
 * 识别到速度超过12mm/s或位移超过1.5mm后，每50ms撤掉约0.5度。
 */
#define BALL_BREAKAWAY_MIN_ERROR_0P1MM        (150)
#define BALL_STATIONARY_POSITION_WINDOW_0P1MM (15)
#define BALL_STATIONARY_SPEED_0P1MM_PER_S     (80)
#define BALL_MOVING_SPEED_0P1MM_PER_S         (120)
#define BALL_STATIONARY_CONFIRM_TICKS_10MS    (6U)
#define BALL_BREAKAWAY_INCREASE_TICKS_10MS    (10U)
#define BALL_BREAKAWAY_DECAY_TICKS_10MS       (5U)
#define BALL_BREAKAWAY_STEP_US                (4)
#define BALL_BREAKAWAY_MAX_BOOST_US           \
    (BALL_SERVO_ADAPTIVE_MAX_OFFSET_US - BALL_SERVO_BASE_MAX_OFFSET_US)
#define BALL_BREAKAWAY_OUTPUT_STEP_US         (7)

/*
 * 要求三只在±50mm之间运动，因此把±58mm设为向外运动软制动区，
 * ±65mm设为硬制动区。它们不会改变精确±50mm目标，只负责拦截失控超调。
 */
#define BALL_SOFT_EDGE_0P1MM                  (750)
#define BALL_HARD_EDGE_0P1MM                  (900)
#define BALL_LOST_EDGE_TRIGGER_0P1MM          (700)

/*
 * 视觉、串口和控制存在约几十毫秒延迟。PID的P项使用80ms预测位置，
 * D项仍直接使用钢球速度，避免目标从+50mm切到-50mm时产生微分冲击。
 */
#define BALL_DELAY_COMPENSATION_MS            (80)

/* Edge recovery outputs are expressed in the abstract ball direction. */
#define BALL_EDGE_BRAKE_US                    (40)
#define BALL_EMERGENCY_BRAKE_US               (BALL_SERVO_MAX_OFFSET_US)

/* 各阶段每个10ms控制周期允许的加速/制动最大脉宽变化。 */
#define BALL_MOVE_ACCEL_STEP_US               (3)
#define BALL_MOVE_BRAKE_STEP_US               (15)
#define BALL_BRAKE_ACCEL_STEP_US              (4)
#define BALL_BRAKE_BRAKE_STEP_US              (20)
#define BALL_CAPTURE_ACCEL_STEP_US            (2)
#define BALL_CAPTURE_BRAKE_STEP_US            (12)
#define BALL_HOLD_ACCEL_STEP_US               (2)
#define BALL_HOLD_BRAKE_STEP_US               (8)
#define BALL_EDGE_OUTPUT_STEP_US              (20)
#define BALL_EMERGENCY_OUTPUT_STEP_US         (25)

/*
 * 双速度估计：快速速度用于预测和紧急制动，慢速速度用于到达/稳定判定。
 * K230约30ms一帧时，这比只使用一个强滤波速度兼顾了响应和抗像素抖动。
 */
#define BALL_FAST_SPEED_NEW_NUMERATOR         (30)
#define BALL_FAST_SPEED_DENOMINATOR           (100)
#define BALL_SLOW_SPEED_NEW_NUMERATOR         (20)
#define BALL_SLOW_SPEED_DENOMINATOR           (100)
#define BALL_SPEED_LIMIT_0P1MM_PER_S          (4000)

static volatile uint8_t s_start_request = 0U;
static volatile uint8_t s_stop_request = 0U;

static AppBall_Status s_status;
static AppBallPid_Controller s_ball_pid;

typedef enum
{
    BALL_CONTROL_MOVE = 0,
    BALL_CONTROL_BRAKE,
    BALL_CONTROL_CAPTURE,
    BALL_CONTROL_HOLD,
    BALL_CONTROL_PHASE_COUNT
} AppBall_ControlPhase;

/*
 * 分阶段PID初值
 * ============================================================================
 *
 * MOVE：P较大、D适中、I关闭，正常依靠约5度完成主要移动；
 *       只有确认静止时才由上层临时提高到最多8度。
 * BRAKE：P减小、D增大，在球仍有速度时尽快反向制动。
 * CAPTURE：P、D和限幅都进一步减小，避免在目标附近大角度来回打舵。
 * HOLD：只允许15us总输出和10us条件积分，用小PID消除机械中位静差。
 *
 * 系数仍使用整数分数，单位定义见app_ball_pid.h。以下是安全起调值，
 * 应按“先MOVE，再BRAKE，再CAPTURE，最后HOLD”的顺序单独微调。
 */
static const AppBallPid_Config s_ball_pid_configs[BALL_CONTROL_PHASE_COUNT] = {
    /* MOVE */
    {1, 10, 0, 1, 1, 40, 80, 400, 0, 0, 1, 10, 80},
    /* BRAKE */
    {1, 20, 0, 1, 1, 10, 80, 300, 0, 0, 1, 10, 80},
    /* CAPTURE */
    {1, 25, 0, 1, 1, 15, 50, 150, 0, 0, 1, 10, 80},
    /* HOLD */
    {1, 30, 0, 1, 1, 20, 60, 120, 0, 0, 1, 8, 60}
};
static AppBall_ControlPhase s_control_phase = BALL_CONTROL_PHASE_COUNT;
static uint32_t s_task_start_tick = 0U;
static uint32_t s_condition_start_tick = 0U;
static uint32_t s_state_start_tick = 0U;
static uint32_t s_complete_hold_start_tick = 0U;
static uint32_t s_invalid_start_tick = 0U;
static uint32_t s_last_ball_frame_count = 0U;
static uint32_t s_last_measurement_tick = 0U;
static uint32_t s_last_control_tick = 0U;
static uint32_t s_last_recovery_tick = 0U;
static uint32_t s_level_start_tick = 0U;
static uint32_t s_level_last_adjust_tick = 0U;
static uint32_t s_stationary_start_tick = 0U;
static uint32_t s_last_breakaway_adjust_tick = 0U;
static uint32_t s_negative_balance_last_trim_tick = 0U;
static int16_t s_last_position_0p1mm = 0;
static int16_t s_stationary_anchor_0p1mm = 0;
static int16_t s_reference_0p1mm = 0;
static int16_t s_fast_speed_0p1mm_per_s = 0;
static int16_t s_speed_0p1mm_per_s = 0;
static int16_t s_last_control_offset_us = 0;
static int16_t s_level_trim_us = 0;
static int16_t s_breakaway_boost_us = 0;
static int16_t s_negative_balance_us = BALL_SERVO_CENTER_US;
static int16_t s_negative_hold_control_us = 0;
static uint8_t s_have_last_position = 0U;
static uint8_t s_breakaway_active = 0U;

/* Runtime copies changed by the OLED BALL SET page. */
static int16_t s_tune_center_us = BALL_SERVO_CENTER_US;
static int16_t s_tune_move_us = BALL_OUTPUT_MOVE_US;
static int16_t s_tune_brake_us = BALL_OUTPUT_BRAKE_US;
static int16_t s_tune_capture_us = BALL_OUTPUT_CAPTURE_US;
static int16_t s_tune_hold_us = BALL_OUTPUT_HOLD_US;
static int16_t s_tune_speed_brake_us = BALL_SPEED_BRAKE_MAX_US;
static int16_t s_tune_pos_speed_0p1mm_per_s =
    BALL_CRUISE_POS_SPEED_0P1MM_PER_S;
static int16_t s_tune_neg_speed_0p1mm_per_s =
    BALL_CRUISE_NEG_SPEED_0P1MM_PER_S;
static int16_t s_tune_predict_ms = BALL_DELAY_COMPENSATION_MS;

static int16_t AppBall_Abs16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static int32_t AppBall_Limit32(int32_t value, int32_t min, int32_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

/* 32位无符号整数平方根，供v=sqrt(2*a*d)速度轨迹使用。 */
static uint32_t AppBall_IntegerSqrt32(uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0U) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

static void AppBall_ResetBreakaway(uint32_t now, int16_t position)
{
    s_stationary_start_tick = now;
    s_last_breakaway_adjust_tick = now;
    s_stationary_anchor_0p1mm = position;
    s_breakaway_boost_us = 0;
    s_breakaway_active = 0U;
}

/*
 * 返回当前剩余距离对应的允许速度绝对值，单位0.1mm/s。
 * 参数使用64位中间值，随后限制到32位平方根输入范围。
 */
static int32_t AppBall_GetAllowedSpeed(
    int32_t distance_0p1mm,
    int32_t cruise_speed_0p1mm_per_s)
{
    uint64_t squared_speed;
    uint32_t allowed_speed;

    if ((distance_0p1mm <= 0) || (cruise_speed_0p1mm_per_s <= 0)) {
        return 0;
    }

    squared_speed =
        2ULL * (uint64_t)BALL_PLANNER_BRAKE_ACCEL_0P1MM_PER_S2 *
        (uint64_t)distance_0p1mm;
    if (squared_speed > UINT32_MAX) {
        squared_speed = UINT32_MAX;
    }

    allowed_speed = AppBall_IntegerSqrt32((uint32_t)squared_speed);
    if (allowed_speed > (uint32_t)cruise_speed_0p1mm_per_s) {
        allowed_speed = (uint32_t)cruise_speed_0p1mm_per_s;
    }
    return (int32_t)allowed_speed;
}

/*
 * 只修改摆杆舵机的目标值，实际脉宽由原有10ms Servo_Update()平滑跟随。
 * 这样不会因为一帧视觉位置变化就突然跳动。
 */
static void AppBall_SetServoTarget(uint16_t pulse_us)
{
#if BALL_CONTROL_SERVO_ID == 1U
    Servo_SetTarget((float)pulse_us, (float)SERVO_CENTER_PULSE_US);
#else
    Servo_SetTarget((float)SERVO_CENTER_PULSE_US, (float)pulse_us);
#endif
    s_status.servo_target_us = pulse_us;
}

/*
 * A 20 ms hold pulse must reach the PWM output immediately.  The normal
 * Servo_SetTarget()/Servo_Update() path intentionally moves gradually and
 * would absorb most of a 35 us pulse before it reached the mechanism.
 */
static void AppBall_SetServoImmediate(uint16_t pulse_us)
{
    pulse_us = Servo_LimitPulse(
        pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

#if BALL_CONTROL_SERVO_ID == 1U
    Servo_SetPulse(pulse_us, (uint16_t)(Servo_Position2 + 0.5f));
    Servo_SetTarget((float)pulse_us, Servo_Target2);
#else
    Servo_SetPulse((uint16_t)(Servo_Position1 + 0.5f), pulse_us);
    Servo_SetTarget(Servo_Target1, (float)pulse_us);
#endif
    s_status.servo_target_us = pulse_us;
}

static void AppBall_SetServoNeutral(void)
{
    int32_t pulse_us;

    s_last_control_offset_us = 0;
    s_status.control_offset_us = 0;
    s_status.level_trim_us = s_level_trim_us;

    /*
     * s_level_trim_us使用“钢球抽象运动方向”表示，必须经过
     * BALL_SERVO_DIRECTION转换成真实舵机脉宽方向。
     */
    pulse_us = s_tune_center_us +
        (BALL_SERVO_DIRECTION * (int32_t)s_level_trim_us);
    pulse_us = AppBall_Limit32(
        pulse_us,
        s_tune_center_us - BALL_SERVO_MAX_OFFSET_US,
        s_tune_center_us + BALL_SERVO_MAX_OFFSET_US);
    AppBall_SetServoTarget((uint16_t)pulse_us);
}

static void AppBall_SetState(AppBall_State state, int16_t target)
{
    uint32_t now = K230_GetTick10ms();

    /*
     * 当最终目标从正侧切换到负侧（或反向切换）时，旧积分的方向已经
     * 不再正确，必须立即清零。否则正目标阶段积累的补偿会在折返瞬间
     * 继续推球，造成额外超调。
     */
    if (((s_status.target_0p1mm > 0) && (target < 0)) ||
        ((s_status.target_0p1mm < 0) && (target > 0))) {
        AppBallPid_ResetIntegral(&s_ball_pid);
        /*
         * Start the new reference at the measured position.  The trajectory
         * generator then moves it smoothly toward the opposite target.  This
         * avoids the former 30 mm reference jump and the resulting full-angle
         * reversal seen in the video.
         */
        s_reference_0p1mm = s_status.position_0p1mm;
    }

    s_status.state = state;
    s_status.target_0p1mm = target;
    s_state_start_tick = now;
    s_condition_start_tick = now;
    AppBall_ResetBreakaway(now, s_status.position_0p1mm);

    if (state == BALL_TASK_HOLD_NEGATIVE) {
        int32_t current_pulse_us;

#if BALL_CONTROL_SERVO_ID == 1U
        current_pulse_us = (int32_t)(Servo_Position1 + 0.5f);
#else
        current_pulse_us = (int32_t)(Servo_Position2 + 0.5f);
#endif
        s_negative_balance_last_trim_tick = now;
        s_negative_hold_control_us = (int16_t)AppBall_Limit32(
            (current_pulse_us - (int32_t)s_negative_balance_us) *
                BALL_SERVO_DIRECTION,
            -BALL_NEGATIVE_HOLD_MAX_OFFSET_US,
            BALL_NEGATIVE_HOLD_MAX_OFFSET_US);
    }
}

static void AppBall_StopInternal(void)
{
    s_status.state = BALL_TASK_IDLE;
    s_status.running = 0U;
    s_status.completed = 0U;
    s_status.target_0p1mm = 0;
    s_status.reference_0p1mm = 0;
    s_status.speed_0p1mm_per_s = 0;
    s_status.control_offset_us = 0;
    s_status.level_trim_us = s_level_trim_us;
    s_status.elapsed_ms = 0U;
    s_status.completion_ms = 0U;
    s_reference_0p1mm = 0;
    s_fast_speed_0p1mm_per_s = 0;
    s_speed_0p1mm_per_s = 0;
    s_last_measurement_tick = 0U;
    s_last_control_tick = 0U;
    s_last_recovery_tick = 0U;
    s_state_start_tick = 0U;
    s_complete_hold_start_tick = 0U;
    s_level_start_tick = 0U;
    s_level_last_adjust_tick = 0U;
    s_stationary_start_tick = 0U;
    s_last_breakaway_adjust_tick = 0U;
    s_negative_balance_last_trim_tick = 0U;
    s_stationary_anchor_0p1mm = 0;
    s_breakaway_boost_us = 0;
    s_negative_balance_us = (int16_t)AppBall_Limit32(
        s_tune_center_us,
        BALL_NEGATIVE_BALANCE_MIN_US,
        BALL_NEGATIVE_BALANCE_MAX_US);
    s_negative_hold_control_us = 0;
    s_breakaway_active = 0U;
    s_have_last_position = 0U;
    AppBallPid_Reset(&s_ball_pid);
    AppBall_SetServoNeutral();
}

static void AppBall_StartInternal(void)
{
    uint32_t now = K230_GetTick10ms();

    s_task_start_tick = now;
    s_condition_start_tick = now;
    s_state_start_tick = now;
    s_complete_hold_start_tick = 0U;
    s_invalid_start_tick = now;
    s_last_ball_frame_count = 0U;
    s_last_measurement_tick = 0U;
    s_last_control_tick = now - 1U;
    s_last_recovery_tick = now - 1U;
    s_level_start_tick = now;
    s_level_last_adjust_tick = now;
    s_stationary_start_tick = now;
    s_last_breakaway_adjust_tick = now;
    s_negative_balance_last_trim_tick = now;
    s_have_last_position = 0U;
    s_stationary_anchor_0p1mm = 0;
    s_reference_0p1mm = 0;
    s_fast_speed_0p1mm_per_s = 0;
    s_speed_0p1mm_per_s = 0;
    s_breakaway_boost_us = 0;
    s_negative_balance_us = (int16_t)AppBall_Limit32(
        s_tune_center_us,
        BALL_NEGATIVE_BALANCE_MIN_US,
        BALL_NEGATIVE_BALANCE_MAX_US);
    s_negative_hold_control_us = 0;
    s_breakaway_active = 0U;
    s_last_control_offset_us = 0;
    AppBallPid_Reset(&s_ball_pid);

    /*
     * 赛题要求三明确规定钢球从中心点O开始。按键计时后不再重复执行
     * 自动调平，直接进入O点快速确认；否则调平会占用5秒预算，并且视觉
     * 速度的少量抖动可能反复重置调平确认计时。
     *
     * s_level_trim_us仍作为已经标定好的水平中值补偿保留。需要重新学习
     * 水平中值时，应在正式测试前完成，不放进要求三计时流程。
     */
    s_status.state = BALL_TASK_WAIT_CENTER;
    s_status.running = 1U;
    s_status.completed = 0U;
    s_status.target_0p1mm = BALL_CENTER_TARGET_0P1MM;
    s_status.reference_0p1mm = BALL_CENTER_TARGET_0P1MM;
    s_status.speed_0p1mm_per_s = 0;
    s_status.control_offset_us = 0;
    s_status.level_trim_us = s_level_trim_us;
    s_status.elapsed_ms = 0U;
    s_status.completion_ms = 0U;
    AppBall_SetServoNeutral();
}

/*
 * 按视觉帧间隔平滑移动内部参考位置。
 *
 * 状态机给出最终目标（例如+50mm或-50mm），本函数让PID参考值按限定速度
 * 逐步接近最终目标，避免目标突变造成舵机瞬间打到最大角度。
 */
static int16_t AppBall_MoveReference(
    int16_t current,
    int16_t target,
    int32_t rate_0p1mm_per_s,
    uint32_t dt_ticks_10ms)
{
    int32_t step = (rate_0p1mm_per_s * (int32_t)dt_ticks_10ms) / 100;
    int32_t next = current;

    if (rate_0p1mm_per_s <= 0) {
        return current;
    }

    if (step < 1) {
        step = 1;
    }

    if (current < target) {
        next += step;
        if (next > target) {
            next = target;
        }
    } else if (current > target) {
        next -= step;
        if (next < target) {
            next = target;
        }
    }

    return (int16_t)next;
}

static int32_t AppBall_RateLimitControl(
    int32_t target,
    int32_t max_step_us)
{
    int32_t min = (int32_t)s_last_control_offset_us -
        max_step_us;
    int32_t max = (int32_t)s_last_control_offset_us +
        max_step_us;

    return AppBall_Limit32(target, min, max);
}

/*
 * 选择控制阶段并装入对应PID参数。阶段发生变化时清空积分，保证MOVE/BRAKE
 * 的历史不会带入HOLD，也避免球被扰动离开HOLD后旧积分继续推球。
 */
static void AppBall_SelectControlPhase(AppBall_ControlPhase phase)
{
    if (phase >= BALL_CONTROL_PHASE_COUNT) {
        phase = BALL_CONTROL_MOVE;
    }

    if (phase != s_control_phase) {
        s_control_phase = phase;
        s_ball_pid.config = s_ball_pid_configs[phase];
        AppBallPid_ResetIntegral(&s_ball_pid);
    }
}

/*
 * 非对称输出变化率限制：
 *   输出方向与钢球速度相反时属于制动，允许使用较大的变化步长；
 *   输出仍在推动钢球当前运动方向时属于加速，使用较小步长。
 */
static int32_t AppBall_SmartRateLimitControl(
    int32_t target,
    int16_t speed_0p1mm_per_s,
    int32_t acceleration_step_us,
    int32_t braking_step_us)
{
    uint8_t braking =
        (((target > 0) && (speed_0p1mm_per_s < 0)) ||
         ((target < 0) && (speed_0p1mm_per_s > 0))) ? 1U : 0U;
    uint8_t releasing_brake =
        (((s_last_control_offset_us > 0) &&
          (target >= 0) &&
          (target < s_last_control_offset_us)) ||
         ((s_last_control_offset_us < 0) &&
          (target <= 0) &&
          (target > s_last_control_offset_us)) ||
         (((int32_t)s_last_control_offset_us * target) < 0)) ? 1U : 0U;

    return AppBall_RateLimitControl(
        target,
        ((braking != 0U) || (releasing_brake != 0U)) ?
        braking_step_us : acceleration_step_us);
}

/*
 * 依据K230位置和双速度估计处理静摩擦。
 *
 * 未确认静止时返回原PID输出；确认静止后先至少给5度，再每100ms增加约
 * 0.5度，最高8度。检测到位移或速度后不再强制起步，并在运动过程中逐步
 * 撤销额外角度，避免钢球刚克服静摩擦便突然加速。
 */
static int32_t AppBall_ApplyBreakawayControl(
    uint32_t now,
    int16_t position,
    int32_t target_error,
    int32_t normal_control_us)
{
    int32_t direction;
    int32_t position_change;
    int32_t fast_speed_abs = AppBall_Abs16(s_fast_speed_0p1mm_per_s);
    int32_t slow_speed_abs = AppBall_Abs16(s_speed_0p1mm_per_s);
    uint8_t moving_state =
        ((s_status.state == BALL_TASK_MOVE_POSITIVE) ||
         (s_status.state == BALL_TASK_MOVE_NEGATIVE)) ? 1U : 0U;

    if ((moving_state == 0U) ||
        (AppBall_Abs16((int16_t)target_error) <
         BALL_BREAKAWAY_MIN_ERROR_0P1MM)) {
        AppBall_ResetBreakaway(now, position);
        return normal_control_us;
    }

    direction = (target_error >= 0) ? 1 : -1;
    position_change = AppBall_Abs16(
        (int16_t)(position - s_stationary_anchor_0p1mm));

    if ((position_change > BALL_STATIONARY_POSITION_WINDOW_0P1MM) ||
        (fast_speed_abs >= BALL_MOVING_SPEED_0P1MM_PER_S)) {
        s_stationary_anchor_0p1mm = position;
        s_stationary_start_tick = now;
        s_breakaway_active = 0U;

        if ((s_breakaway_boost_us > 0) &&
            ((uint32_t)(now - s_last_breakaway_adjust_tick) >=
             BALL_BREAKAWAY_DECAY_TICKS_10MS)) {
            s_breakaway_boost_us -= BALL_BREAKAWAY_STEP_US;
            if (s_breakaway_boost_us < 0) {
                s_breakaway_boost_us = 0;
            }
            s_last_breakaway_adjust_tick = now;
        }

        return normal_control_us +
            direction * (int32_t)s_breakaway_boost_us;
    }

    if (slow_speed_abs > BALL_STATIONARY_SPEED_0P1MM_PER_S) {
        s_stationary_anchor_0p1mm = position;
        s_stationary_start_tick = now;
        s_breakaway_active = 0U;
        return normal_control_us;
    }

    if ((uint32_t)(now - s_stationary_start_tick) <
        BALL_STATIONARY_CONFIRM_TICKS_10MS) {
        return normal_control_us;
    }

    if (s_breakaway_active == 0U) {
        s_breakaway_active = 1U;
        s_last_breakaway_adjust_tick = now;
    } else if (
        (uint32_t)(now - s_last_breakaway_adjust_tick) >=
        BALL_BREAKAWAY_INCREASE_TICKS_10MS) {
        int32_t next_boost =
            (int32_t)s_breakaway_boost_us + BALL_BREAKAWAY_STEP_US;

        s_breakaway_boost_us = (int16_t)AppBall_Limit32(
            next_boost, 0, BALL_BREAKAWAY_MAX_BOOST_US);
        s_last_breakaway_adjust_tick = now;
    }

    {
        int32_t breakaway_output =
            BALL_SERVO_BASE_MAX_OFFSET_US + s_breakaway_boost_us;

        /*
         * MOVE on the OLED must also control the static-friction release.
         * Previously this path was fixed near 41 us, so raising MOVE above
         * that value had no effect while the ball was still stationary.
         */
        if (breakaway_output < s_tune_move_us) {
            breakaway_output = s_tune_move_us;
        }
        return direction * breakaway_output;
    }
}

/*
 * 根据调平阶段观测到的钢球漂移速度，小步修正水管水平中值。
 *
 * s_level_trim_us是抽象控制方向：
 *   正值会使钢球向正坐标运动；
 *   负值会使钢球向负坐标运动。
 *
 * 所以钢球正在向正方向漂移时要减小trim，反之增加trim。
 */
static void AppBall_UpdateLeveling(
    uint32_t measurement_tick,
    int16_t position)
{
    if (((uint32_t)(measurement_tick - s_level_last_adjust_tick) >=
         BALL_LEVEL_ADJUST_INTERVAL_TICKS_10MS) &&
        (AppBall_Abs16(position) <= BALL_LEVEL_POSITION_WINDOW_0P1MM)) {
        int32_t trim = s_level_trim_us;

        s_level_last_adjust_tick = measurement_tick;

        if (s_speed_0p1mm_per_s > BALL_LEVEL_DRIFT_SPEED_0P1MM_PER_S) {
            trim -= BALL_LEVEL_TRIM_STEP_US;
        } else if (
            s_speed_0p1mm_per_s < -BALL_LEVEL_DRIFT_SPEED_0P1MM_PER_S) {
            trim += BALL_LEVEL_TRIM_STEP_US;
        }

        trim = AppBall_Limit32(
            trim,
            -BALL_LEVEL_TRIM_LIMIT_US,
            BALL_LEVEL_TRIM_LIMIT_US);
        s_level_trim_us = (int16_t)trim;
    }

    /*
     * 调平阶段不允许普通PID和积分介入，否则无法区分钢球漂移究竟来自
     * 水管倾斜还是位置控制。舵机只使用“基础中位+调平补偿”。
     */
    AppBallPid_Reset(&s_ball_pid);
    s_reference_0p1mm = BALL_CENTER_TARGET_0P1MM;
    s_status.reference_0p1mm = BALL_CENTER_TARGET_0P1MM;
    s_status.speed_0p1mm_per_s = s_speed_0p1mm_per_s;
    s_status.level_trim_us = s_level_trim_us;
    AppBall_SetServoNeutral();
}

static void AppBall_CommandControlOffset(int32_t control_us)
{
    int32_t pulse_us;
    int32_t total_abstract_us;
    int32_t center_us = s_tune_center_us;

    control_us = AppBall_Limit32(
        control_us,
        -BALL_SERVO_MAX_OFFSET_US,
        BALL_SERVO_MAX_OFFSET_US);
    s_last_control_offset_us = (int16_t)control_us;

    /*
     * 正常PID输出以自动调平得到的中值为零点。
     * 这样PID只负责球的位置变化，不必长期用积分抵消水管固有倾斜。
     */
    total_abstract_us =
        (int32_t)s_level_trim_us + control_us;
    total_abstract_us = AppBall_Limit32(
        total_abstract_us,
        -BALL_SERVO_MAX_OFFSET_US,
        BALL_SERVO_MAX_OFFSET_US);

    pulse_us = center_us +
        (BALL_SERVO_DIRECTION * total_abstract_us);
    pulse_us = AppBall_Limit32(
        pulse_us,
        center_us - BALL_SERVO_MAX_OFFSET_US,
        center_us + BALL_SERVO_MAX_OFFSET_US);

    s_status.control_offset_us = s_last_control_offset_us;
    s_status.level_trim_us = s_level_trim_us;
    AppBall_SetServoTarget((uint16_t)pulse_us);
}

/*
 * Smooth final -5 cm controller.
 *
 * The outer position loop produces a desired ball speed which naturally
 * approaches zero at the target.  The inner speed loop produces a servo
 * offset.  The actual PWM is then slewed by only 2 us per 10 ms while driving
 * and 4 us while braking, so the former 35 us pulse/80 ms wait jerk is gone.
 *
 * A 35 us dead-zone term is still used outside 6 mm, but only in the desired
 * value; it is never written to PWM in one step.  Inside 3..6 mm, a persistent
 * low-speed error slowly learns the mechanical balance by 1 us every 80 ms.
 */
static void AppBall_UpdateNegativeHold(
    uint32_t now,
    uint8_t new_frame,
    int16_t position)
{
    int32_t error =
        (int32_t)BALL_NEGATIVE_TARGET_0P1MM - (int32_t)position;
    int32_t error_abs = (error < 0) ? -error : error;
    int32_t speed_abs = AppBall_Abs16(s_speed_0p1mm_per_s);
    int32_t target_speed;
    int32_t speed_error;
    int32_t desired_control_us;
    int32_t max_step_us;
    int32_t pulse_us;
    uint8_t is_braking;

    AppBallPid_Reset(&s_ball_pid);
    AppBall_ResetBreakaway(now, position);
    s_reference_0p1mm = BALL_NEGATIVE_TARGET_0P1MM;
    s_status.reference_0p1mm = BALL_NEGATIVE_TARGET_0P1MM;
    s_status.speed_0p1mm_per_s = s_speed_0p1mm_per_s;

    if ((new_frame != 0U) &&
        (error_abs > BALL_NEGATIVE_HOLD_DEADBAND_0P1MM) &&
        (error_abs <= BALL_NEGATIVE_HOLD_TRIM_LIMIT_0P1MM) &&
        (speed_abs <= BALL_NEGATIVE_HOLD_TRIM_SPEED_0P1MM_PER_S) &&
        ((uint32_t)(now - s_negative_balance_last_trim_tick) >=
         BALL_NEGATIVE_HOLD_TRIM_TICKS_10MS)) {
        int32_t trim_direction = (error > 0) ? 1 : -1;

        s_negative_balance_us = (int16_t)AppBall_Limit32(
            (int32_t)s_negative_balance_us +
                BALL_SERVO_DIRECTION * trim_direction,
            BALL_NEGATIVE_BALANCE_MIN_US,
            BALL_NEGATIVE_BALANCE_MAX_US);
        s_negative_balance_last_trim_tick = now;
    }

    if (error_abs <= BALL_NEGATIVE_HOLD_DEADBAND_0P1MM) {
        target_speed = 0;
    } else {
        target_speed =
            error * BALL_NEGATIVE_HOLD_POSITION_KP_NUM /
            BALL_NEGATIVE_HOLD_POSITION_KP_DEN;
        target_speed = AppBall_Limit32(
            target_speed,
            -BALL_NEGATIVE_HOLD_SPEED_MAX_0P1MM_PER_S,
            BALL_NEGATIVE_HOLD_SPEED_MAX_0P1MM_PER_S);
    }

    speed_error = target_speed - (int32_t)s_speed_0p1mm_per_s;
    desired_control_us =
        speed_error / BALL_NEGATIVE_HOLD_SPEED_KP_DEN;

    /*
     * Compensate the measured mechanism dead zone only when the ball is more
     * than 6 mm away.  The following slew limiter turns this into a smooth
     * ramp rather than a visible 35 us jump.
     */
    if (error_abs > BALL_NEGATIVE_HOLD_DEADZONE_TRIGGER_0P1MM) {
        desired_control_us +=
            (error > 0) ? BALL_NEGATIVE_HOLD_DEADZONE_US :
                          -BALL_NEGATIVE_HOLD_DEADZONE_US;
    }
    desired_control_us = AppBall_Limit32(
        desired_control_us,
        -BALL_NEGATIVE_HOLD_MAX_OFFSET_US,
        BALL_NEGATIVE_HOLD_MAX_OFFSET_US);

    is_braking =
        ((desired_control_us > 0) && (s_fast_speed_0p1mm_per_s < 0)) ||
        ((desired_control_us < 0) && (s_fast_speed_0p1mm_per_s > 0));
    max_step_us = (is_braking != 0U) ?
        BALL_NEGATIVE_HOLD_BRAKE_STEP_US :
        BALL_NEGATIVE_HOLD_DRIVE_STEP_US;

    s_negative_hold_control_us = (int16_t)AppBall_Limit32(
        desired_control_us,
        (int32_t)s_negative_hold_control_us - max_step_us,
        (int32_t)s_negative_hold_control_us + max_step_us);

    pulse_us = (int32_t)s_negative_balance_us +
        BALL_SERVO_DIRECTION * (int32_t)s_negative_hold_control_us;
    pulse_us = AppBall_Limit32(
        pulse_us,
        BALL_NEGATIVE_BALANCE_MIN_US - BALL_NEGATIVE_HOLD_MAX_OFFSET_US,
        BALL_NEGATIVE_BALANCE_MAX_US + BALL_NEGATIVE_HOLD_MAX_OFFSET_US);

    s_last_control_offset_us = s_negative_hold_control_us;
    s_status.control_offset_us = s_negative_hold_control_us;
    s_status.level_trim_us =
        (int16_t)((int32_t)s_negative_balance_us - s_tune_center_us);
    AppBall_SetServoImmediate((uint16_t)pulse_us);
}

static void AppBall_ApplyLostRecovery(uint32_t now)
{
    int32_t recovery_us;

    if (now == s_last_recovery_tick) {
        return;
    }
    s_last_recovery_tick = now;

    if ((s_last_position_0p1mm >= BALL_LOST_EDGE_TRIGGER_0P1MM) &&
        (s_fast_speed_0p1mm_per_s > 0)) {
        recovery_us = -BALL_EMERGENCY_BRAKE_US;
    } else if (
        (s_last_position_0p1mm <= -BALL_LOST_EDGE_TRIGGER_0P1MM) &&
        (s_fast_speed_0p1mm_per_s < 0)) {
        recovery_us = BALL_EMERGENCY_BRAKE_US;
    } else {
        return;
    }

    /*
     * 视觉短暂丢失时采用的是固定方向的应急制动，不属于正常PID控制。
     * 清除积分，防止找回小球后旧积分与应急制动方向叠加。
     */
    AppBallPid_ResetIntegral(&s_ball_pid);

    recovery_us = AppBall_RateLimitControl(
        recovery_us,
        BALL_EMERGENCY_OUTPUT_STEP_US);
    AppBall_CommandControlOffset(recovery_us);
}

static void AppBall_UpdateController(const K230_Status *k230)
{
    int16_t position;
    uint32_t now_tick = K230_GetTick10ms();
    uint32_t control_dt_ticks_10ms;
    uint32_t measurement_tick;
    uint32_t measurement_dt_ticks_10ms;
    int32_t raw_speed = 0;
    int32_t target_error;
    int32_t target_error_abs;
    int32_t predicted_target_error;
    int32_t predicted_error_abs;
    int32_t phase_speed_abs;
    int32_t allowed_speed;
    int32_t cruise_speed;
    int32_t speed_excess;
    int32_t speed_brake_us;
    int32_t hold_damping_us;
    int16_t controller_speed;
    int32_t output_limit;
    int32_t control_us;
    int32_t predicted_position;
    int32_t acceleration_step_us = BALL_MOVE_ACCEL_STEP_US;
    int32_t braking_step_us = BALL_MOVE_BRAKE_STEP_US;
    uint8_t integral_allowed;
    uint8_t new_frame;
    uint8_t moving_toward_target;
    uint8_t predicted_crossed_target;
    uint8_t beyond_target_outward;
    AppBall_ControlPhase phase;

    /*
     * 外环固定每10ms运行。K230位置帧通常约30ms一帧；若把PID直接绑定
     * 在新视觉帧上，舵机目标就会以约30ms为台阶更新，表现为一卡一卡。
     */
    if (now_tick == s_last_control_tick) {
        return;
    }
    control_dt_ticks_10ms = (uint32_t)(now_tick - s_last_control_tick);
    if (control_dt_ticks_10ms > 5U) {
        control_dt_ticks_10ms = 5U;
    }
    s_last_control_tick = now_tick;

    /*
     * 位置和速度只在收到新K230帧时刷新，避免把同一位置在10ms任务中
     * 重复求差分；两帧之间保留最新速度，由固定周期控制器连续输出。
     */
    new_frame =
        (k230->ball_frame_count != s_last_ball_frame_count) ? 1U : 0U;
    if (new_frame != 0U) {
        position = k230->ball_position_0p1mm;
        measurement_tick = k230->ball_last_rx_tick_10ms;
        s_last_ball_frame_count = k230->ball_frame_count;

        if (s_have_last_position != 0U) {
            measurement_dt_ticks_10ms =
                (uint32_t)(measurement_tick - s_last_measurement_tick);
            if (measurement_dt_ticks_10ms == 0U) {
                measurement_dt_ticks_10ms = 1U;
            } else if (
                measurement_dt_ticks_10ms > K230_BALL_OFFLINE_TICKS_10MS) {
                measurement_dt_ticks_10ms = K230_BALL_OFFLINE_TICKS_10MS;
            }

            raw_speed =
                ((int32_t)position - (int32_t)s_last_position_0p1mm) * 100 /
                (int32_t)measurement_dt_ticks_10ms;
            raw_speed = AppBall_Limit32(
                raw_speed,
                -BALL_SPEED_LIMIT_0P1MM_PER_S,
                BALL_SPEED_LIMIT_0P1MM_PER_S);

            s_fast_speed_0p1mm_per_s = (int16_t)(
                ((int32_t)s_fast_speed_0p1mm_per_s *
                 (BALL_FAST_SPEED_DENOMINATOR -
                  BALL_FAST_SPEED_NEW_NUMERATOR) +
                 raw_speed * BALL_FAST_SPEED_NEW_NUMERATOR) /
                BALL_FAST_SPEED_DENOMINATOR);

            s_speed_0p1mm_per_s = (int16_t)(
                ((int32_t)s_speed_0p1mm_per_s *
                 (BALL_SLOW_SPEED_DENOMINATOR -
                  BALL_SLOW_SPEED_NEW_NUMERATOR) +
                 raw_speed * BALL_SLOW_SPEED_NEW_NUMERATOR) /
                BALL_SLOW_SPEED_DENOMINATOR);
        } else {
            s_fast_speed_0p1mm_per_s = 0;
            s_speed_0p1mm_per_s = 0;
        }
        s_last_position_0p1mm = position;
        s_last_measurement_tick = measurement_tick;
        s_have_last_position = 1U;
    }

    if (s_have_last_position == 0U) {
        return;
    }
    position = s_last_position_0p1mm;

    /*
     * 自动调平只使用视觉速度修正舵机中值，不运行位置PID。
     * 速度计算仍复用上面的滤波逻辑，因此不会被单帧像素抖动误导。
     */
    if (s_status.state == BALL_TASK_LEVEL_PIPE) {
        if (new_frame != 0U) {
            AppBall_UpdateLeveling(
                k230->ball_last_rx_tick_10ms,
                position);
        }
        return;
    }

    /*
     * At the final -5 cm point, continuous PID cannot command a useful value
     * inside the measured 32 us mechanism dead zone.  Use the dedicated
     * dynamic-balance/pulse controller instead, including during the three
     * seconds after the timer has stopped.
     */
    if ((s_status.state == BALL_TASK_HOLD_NEGATIVE) ||
        (s_status.state == BALL_TASK_COMPLETE)) {
        AppBall_UpdateNegativeHold(now_tick, new_frame, position);
        return;
    }

    target_error = (int32_t)s_status.target_0p1mm - (int32_t)position;
    target_error_abs = AppBall_Abs16((int16_t)target_error);
    predicted_position = (int32_t)position +
        ((int32_t)s_fast_speed_0p1mm_per_s *
         s_tune_predict_ms) / 1000;
    predicted_position = AppBall_Limit32(
        predicted_position, -1250, 1250);
    predicted_target_error =
        (int32_t)s_status.target_0p1mm - predicted_position;
    predicted_error_abs =
        (predicted_target_error < 0) ?
        -predicted_target_error : predicted_target_error;

    cruise_speed =
        (s_status.target_0p1mm == BALL_NEGATIVE_TARGET_0P1MM) ?
        s_tune_neg_speed_0p1mm_per_s :
        s_tune_pos_speed_0p1mm_per_s;
    allowed_speed = AppBall_GetAllowedSpeed(
        predicted_error_abs, cruise_speed);

    /*
     * 参考轨迹不再用固定高速直接冲向目标，而是按剩余距离对应的允许速度
     * 推进。越接近目标，参考位置移动越慢；预测已经越过目标时暂时冻结
     * 参考，让下面的反向制动先把速度降下来。
     */
    if ((s_status.state == BALL_TASK_HOLD_NEGATIVE) ||
        (s_status.state == BALL_TASK_COMPLETE)) {
        s_reference_0p1mm = s_status.target_0p1mm;
    } else {
        s_reference_0p1mm = AppBall_MoveReference(
            s_reference_0p1mm,
            s_status.target_0p1mm,
            allowed_speed,
            control_dt_ticks_10ms);
    }

    phase_speed_abs = AppBall_Abs16(s_fast_speed_0p1mm_per_s);
    moving_toward_target =
        (((target_error > 0) && (s_fast_speed_0p1mm_per_s > 0)) ||
         ((target_error < 0) && (s_fast_speed_0p1mm_per_s < 0))) ? 1U : 0U;
    predicted_crossed_target =
        (((target_error > 0) && (predicted_target_error <= 0) &&
          (s_fast_speed_0p1mm_per_s >=
           BALL_TARGET_BRAKE_MIN_SPEED_0P1MM_PER_S)) ||
         ((target_error < 0) && (predicted_target_error >= 0) &&
          (s_fast_speed_0p1mm_per_s <=
           -BALL_TARGET_BRAKE_MIN_SPEED_0P1MM_PER_S))) ? 1U : 0U;
    beyond_target_outward =
        ((((int32_t)position >= (int32_t)s_status.target_0p1mm) &&
          (s_status.target_0p1mm > 0) &&
          (s_fast_speed_0p1mm_per_s >=
           BALL_TARGET_BRAKE_MIN_SPEED_0P1MM_PER_S)) ||
         (((int32_t)position <= (int32_t)s_status.target_0p1mm) &&
          (s_status.target_0p1mm < 0) &&
          (s_fast_speed_0p1mm_per_s <=
           -BALL_TARGET_BRAKE_MIN_SPEED_0P1MM_PER_S))) ? 1U : 0U;

    /*
     * 边界保护优先级高于PID。抽象控制量为正时，小球向正坐标方向运动；
     * 所以小球接近正边界时必须给负控制量，接近负边界时反之。
     * 边界保护期间同时清除积分，避免退出保护后积分再次把球推向边缘。
     */
    if (position >= BALL_HARD_EDGE_0P1MM) {
        AppBallPid_ResetIntegral(&s_ball_pid);
        control_us = -BALL_EMERGENCY_BRAKE_US;
        output_limit = BALL_SERVO_MAX_OFFSET_US;
        acceleration_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
        braking_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
    } else if (position <= -BALL_HARD_EDGE_0P1MM) {
        AppBallPid_ResetIntegral(&s_ball_pid);
        control_us = BALL_EMERGENCY_BRAKE_US;
        output_limit = BALL_SERVO_MAX_OFFSET_US;
        acceleration_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
        braking_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
    } else if (
        (s_status.state == BALL_TASK_MOVE_NEGATIVE) &&
        ((uint32_t)(now_tick - s_state_start_tick) <
         BALL_NEGATIVE_LIFT_TICKS_10MS) &&
        (position > BALL_CENTER_TARGET_0P1MM)) {
        /*
         * The +5 -> -5 reversal must first overcome the servo/linkage dead
         * zone.  Apply a short, fast negative-direction lift while the ball
         * is still on the positive side, then hand control back to PD.
         */
        AppBallPid_ResetIntegral(&s_ball_pid);
        control_us = -BALL_NEGATIVE_LIFT_US;
        output_limit = BALL_SERVO_MAX_OFFSET_US;
        acceleration_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
        braking_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
    } else if ((beyond_target_outward != 0U) ||
               (predicted_crossed_target != 0U)) {
        /*
         * 实际位置或80ms预测位置已经到达目标且仍向外运动，立即使用12度
         * 反向制动。这一层优先于普通PID，避免+5cm和-5cm处继续冲出。
         */
        AppBallPid_ResetIntegral(&s_ball_pid);
        control_us =
            (s_fast_speed_0p1mm_per_s > 0) ?
            -s_tune_speed_brake_us : s_tune_speed_brake_us;
        output_limit = s_tune_speed_brake_us;
        acceleration_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
        braking_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
    } else if (
        (moving_toward_target != 0U) &&
        (phase_speed_abs >
         (allowed_speed + BALL_OVERSPEED_MARGIN_0P1MM_PER_S))) {
        /*
         * 还没越过目标，但当前速度已经高于制动曲线。反向角度随超速量
         * 从约6度增加到约12度，速度越危险，制动越强。
         */
        speed_excess = phase_speed_abs - allowed_speed;
        speed_brake_us = BALL_SPEED_BRAKE_MIN_US + speed_excess / 10;
        speed_brake_us = AppBall_Limit32(
            speed_brake_us,
            BALL_SPEED_BRAKE_MIN_US,
            s_tune_speed_brake_us);

        AppBallPid_ResetIntegral(&s_ball_pid);
        control_us =
            (s_fast_speed_0p1mm_per_s > 0) ?
            -speed_brake_us : speed_brake_us;
        output_limit = s_tune_speed_brake_us;
        acceleration_step_us = BALL_BRAKE_BRAKE_STEP_US;
        braking_step_us = BALL_EMERGENCY_OUTPUT_STEP_US;
    } else if ((position >= BALL_SOFT_EDGE_0P1MM) &&
               (s_fast_speed_0p1mm_per_s > 0)) {
        AppBallPid_ResetIntegral(&s_ball_pid);
        control_us = -BALL_EDGE_BRAKE_US;
        output_limit = BALL_SERVO_MAX_OFFSET_US;
        acceleration_step_us = BALL_EDGE_OUTPUT_STEP_US;
        braking_step_us = BALL_EDGE_OUTPUT_STEP_US;
    } else if ((position <= -BALL_SOFT_EDGE_0P1MM) &&
               (s_fast_speed_0p1mm_per_s < 0)) {
        AppBallPid_ResetIntegral(&s_ball_pid);
        control_us = BALL_EDGE_BRAKE_US;
        output_limit = BALL_SERVO_MAX_OFFSET_US;
        acceleration_step_us = BALL_EDGE_OUTPUT_STEP_US;
        braking_step_us = BALL_EDGE_OUTPUT_STEP_US;
    } else {
        /*
         * 使用最终目标误差和速度选择四个控制阶段。阶段判断不用渐变参考误差，
         * 防止参考轨迹刚好经过钢球当前位置时误进入HOLD。
         */
        /*
         * HOLD使用滞回：进入条件严格（4mm/8mm/s），退出条件放宽到
         * 8mm/20mm/s。这样K230在边界附近的微小抖动不会让
         * CAPTURE/HOLD每帧切换并反复清空积分。
         */
        if ((s_control_phase == BALL_CONTROL_HOLD) &&
            (target_error_abs <= BALL_HOLD_EXIT_ERROR_0P1MM) &&
            (phase_speed_abs <= BALL_HOLD_EXIT_SPEED_0P1MM_PER_S)) {
            phase = BALL_CONTROL_HOLD;
            output_limit = s_tune_hold_us;
            acceleration_step_us = BALL_HOLD_ACCEL_STEP_US;
            braking_step_us = BALL_HOLD_BRAKE_STEP_US;
        } else if (target_error_abs > BALL_PHASE_MOVE_ERROR_0P1MM) {
            phase = BALL_CONTROL_MOVE;
            output_limit = s_tune_move_us;
            acceleration_step_us = BALL_MOVE_ACCEL_STEP_US;
            braking_step_us = BALL_MOVE_BRAKE_STEP_US;
        } else if (
            (target_error_abs > BALL_PHASE_BRAKE_ERROR_0P1MM) ||
            (phase_speed_abs > BALL_PHASE_BRAKE_SPEED_0P1MM_PER_S)) {
            phase = BALL_CONTROL_BRAKE;
            output_limit = s_tune_brake_us;
            acceleration_step_us = BALL_BRAKE_ACCEL_STEP_US;
            braking_step_us = BALL_BRAKE_BRAKE_STEP_US;
        } else if (
            (target_error_abs > BALL_PHASE_CAPTURE_ERROR_0P1MM) ||
            (phase_speed_abs > BALL_PHASE_CAPTURE_SPEED_0P1MM_PER_S)) {
            phase = BALL_CONTROL_CAPTURE;
            output_limit = s_tune_capture_us;
            acceleration_step_us = BALL_CAPTURE_ACCEL_STEP_US;
            braking_step_us = BALL_CAPTURE_BRAKE_STEP_US;
        } else {
            phase = BALL_CONTROL_HOLD;
            output_limit = s_tune_hold_us;
            acceleration_step_us = BALL_HOLD_ACCEL_STEP_US;
            braking_step_us = BALL_HOLD_BRAKE_STEP_US;
        }
        AppBall_SelectControlPhase(phase);

        /*
         * 只有HOLD阶段且小球已经进入最终目标附近时才允许积分。
         * PID模块内部还会继续检查预测参考误差和速度，因此必须同时满足：
         *   1. 实际位置接近最终目标；
         *   2. 实际位置接近渐变参考；
         *   3. 小球速度足够低。
         * 这样积分只消除目标附近的静差，不参与长距离加速。
         */
        /* Keep I disabled until the PD motion and braking are repeatable. */
        integral_allowed = 0U;

        controller_speed =
            (phase == BALL_CONTROL_HOLD) ?
            s_speed_0p1mm_per_s : s_fast_speed_0p1mm_per_s;

        /*
         * P项使用80ms预测位置，能在画面显示到达目标之前开始减小倾角；
         * D项仍使用滤波速度。这里不再叠加固定的60us预测刹车脉冲，
         * 避免在目标附近产生过大的离散反向动作。
         */
        control_us = AppBallPid_Update(
            &s_ball_pid,
            s_reference_0p1mm,
            (int16_t)predicted_position,
            controller_speed,
            control_dt_ticks_10ms,
            integral_allowed);

        /*
         * 负端低速保持使用K230快速速度估计做附加阻尼。它不是固定周期
         * 抖舵：只有视觉确认小球仍在运动时才动作，且方向始终与速度相反。
         * 小球速度换向后，阻尼方向也会换向，因此会形成幅度受限的主动平衡。
         */
        if ((phase == BALL_CONTROL_HOLD) &&
            (s_status.target_0p1mm == BALL_NEGATIVE_TARGET_0P1MM) &&
            (phase_speed_abs >=
             BALL_HOLD_DAMPING_MIN_SPEED_0P1MM_PER_S)) {
            hold_damping_us =
                BALL_HOLD_DAMPING_MIN_US +
                (phase_speed_abs -
                 BALL_HOLD_DAMPING_MIN_SPEED_0P1MM_PER_S) / 10;
            hold_damping_us = AppBall_Limit32(
                hold_damping_us,
                BALL_HOLD_DAMPING_MIN_US,
                BALL_HOLD_DAMPING_MAX_US);

            control_us +=
                (s_fast_speed_0p1mm_per_s > 0) ?
                -hold_damping_us : hold_damping_us;
        }

        control_us = AppBall_ApplyBreakawayControl(
            now_tick,
            position,
            target_error,
            control_us);

        /*
         * 自适应起步可能临时超过正常MOVE的5度限幅，但始终不会越过8度
         * 硬限制。进入运动后extra boost按0.5度小步衰减。
         */
        if ((s_breakaway_active != 0U) ||
            (s_breakaway_boost_us > 0)) {
            int32_t breakaway_limit =
                BALL_SERVO_BASE_MAX_OFFSET_US +
                (int32_t)s_breakaway_boost_us;

            if (breakaway_limit > output_limit) {
                output_limit = breakaway_limit;
            }
            acceleration_step_us = BALL_BREAKAWAY_OUTPUT_STEP_US;
            braking_step_us = BALL_MOVE_BRAKE_STEP_US;
        }

        /* Keep the first 0 -> +5 cm segment gentle; braking is not capped. */
        if ((s_status.state == BALL_TASK_MOVE_POSITIVE) &&
            (phase == BALL_CONTROL_MOVE) &&
            (output_limit > BALL_POSITIVE_MOVE_LIMIT_US)) {
            output_limit = BALL_POSITIVE_MOVE_LIMIT_US;
        }
    }

    /*
     * 先把“上一帧输出基准”限制在当前控制区间，再做变化率限制。
     *
     * 原实现只在变化率限制前限制本帧目标。如果上一帧仍是150us，
     * 刚进入28us精细区时，变化率限制可能返回146us，等于绕过了精细区
     * 限幅，这正是目标附近微调幅度仍然很大的主要原因。
     */
    s_last_control_offset_us = (int16_t)AppBall_Limit32(
        s_last_control_offset_us,
        -output_limit,
        output_limit);
    control_us = AppBall_Limit32(
        control_us,
        -output_limit,
        output_limit);
    control_us = AppBall_SmartRateLimitControl(
        control_us,
        s_fast_speed_0p1mm_per_s,
        acceleration_step_us,
        braking_step_us);
    control_us = AppBall_Limit32(
        control_us,
        -output_limit,
        output_limit);

    s_status.reference_0p1mm = s_reference_0p1mm;
    s_status.speed_0p1mm_per_s = s_speed_0p1mm_per_s;
    AppBall_CommandControlOffset(control_us);
}

void AppBall_Init(void)
{
    s_start_request = 0U;
    s_stop_request = 0U;
    s_level_trim_us = 0;
    s_status.communication_ok = 0U;
    s_status.measurement_valid = 0U;
    s_status.position_0p1mm = 0;
    s_status.level_trim_us = 0;
    s_control_phase = BALL_CONTROL_PHASE_COUNT;
    AppBallPid_Init(&s_ball_pid, &s_ball_pid_configs[BALL_CONTROL_MOVE]);
    AppBall_SelectControlPhase(BALL_CONTROL_MOVE);
    AppBall_StopInternal();
}

void AppBall_RequestStart(void)
{
    s_start_request = 1U;
}

void AppBall_RequestStop(void)
{
    s_stop_request = 1U;
}

int16_t AppBall_GetTuneParameter(AppBall_TuneParameter parameter)
{
    switch (parameter) {
        case APP_BALL_TUNE_CENTER_US:      return s_tune_center_us;
        case APP_BALL_TUNE_MOVE_US:        return s_tune_move_us;
        case APP_BALL_TUNE_BRAKE_US:       return s_tune_brake_us;
        case APP_BALL_TUNE_CAPTURE_US:     return s_tune_capture_us;
        case APP_BALL_TUNE_HOLD_US:        return s_tune_hold_us;
        case APP_BALL_TUNE_SPEED_BRAKE_US: return s_tune_speed_brake_us;
        case APP_BALL_TUNE_POS_SPEED:      return s_tune_pos_speed_0p1mm_per_s;
        case APP_BALL_TUNE_NEG_SPEED:      return s_tune_neg_speed_0p1mm_per_s;
        case APP_BALL_TUNE_PREDICT_MS:     return s_tune_predict_ms;
        default:                           return 0;
    }
}

void AppBall_AdjustTuneParameter(
    AppBall_TuneParameter parameter,
    int8_t direction)
{
    int32_t value;
    int32_t step;
    int32_t minimum;
    int32_t maximum;
    int16_t *destination;

    if (direction == 0) {
        return;
    }

    destination = 0;
    step = 1;
    minimum = 0;
    maximum = 0;

    switch (parameter) {
        case APP_BALL_TUNE_CENTER_US:
            destination = &s_tune_center_us;
            step = 5;
            minimum = BALL_CENTER_TUNE_MIN_US;
            maximum = BALL_CENTER_TUNE_MAX_US;
            break;
        case APP_BALL_TUNE_MOVE_US:
            destination = &s_tune_move_us;
            minimum = 10;
            maximum = 80;
            break;
        case APP_BALL_TUNE_BRAKE_US:
            destination = &s_tune_brake_us;
            minimum = 8;
            maximum = BALL_SERVO_MAX_OFFSET_US;
            break;
        case APP_BALL_TUNE_CAPTURE_US:
            destination = &s_tune_capture_us;
            minimum = 4;
            maximum = 60;
            break;
        case APP_BALL_TUNE_HOLD_US:
            destination = &s_tune_hold_us;
            minimum = 2;
            maximum = 30;
            break;
        case APP_BALL_TUNE_SPEED_BRAKE_US:
            destination = &s_tune_speed_brake_us;
            minimum = BALL_SPEED_BRAKE_MIN_US;
            maximum = BALL_SERVO_MAX_OFFSET_US;
            break;
        case APP_BALL_TUNE_POS_SPEED:
            destination = &s_tune_pos_speed_0p1mm_per_s;
            step = 50;
            minimum = 300;
            maximum = 900;
            break;
        case APP_BALL_TUNE_NEG_SPEED:
            destination = &s_tune_neg_speed_0p1mm_per_s;
            step = 50;
            minimum = 300;
            maximum = 1000;
            break;
        case APP_BALL_TUNE_PREDICT_MS:
            destination = &s_tune_predict_ms;
            step = 10;
            minimum = 20;
            maximum = 250;
            break;
        default:
            return;
    }

    value = (int32_t)(*destination) +
        ((direction > 0) ? step : -step);
    value = AppBall_Limit32(value, minimum, maximum);
    *destination = (int16_t)value;

    /* CENTER is applied immediately while the tuning page owns a stopped task. */
    if ((parameter == APP_BALL_TUNE_CENTER_US) &&
        (s_status.running == 0U)) {
        AppBall_SetServoNeutral();
    }
}

void AppBall_BackgroundTask(const K230_Status *k230)
{
    uint32_t now;
    uint32_t elapsed_ticks;
    uint8_t measurement_valid;
    int16_t error_abs;
    int16_t speed_abs;

    if (k230 == 0) {
        return;
    }

    now = K230_GetTick10ms();

    if (s_stop_request != 0U) {
        s_stop_request = 0U;
        s_start_request = 0U;
        AppBall_StopInternal();
    } else if (s_start_request != 0U) {
        s_start_request = 0U;
        AppBall_StartInternal();
    }

    s_status.communication_ok = k230->ball_online;
    measurement_valid =
        ((k230->ball_online != 0U) &&
         (k230->ball_valid != 0U) &&
         (k230->ball_quality >= BALL_MIN_QUALITY)) ? 1U : 0U;
    s_status.measurement_valid = measurement_valid;

    if (measurement_valid != 0U) {
        s_status.position_0p1mm = k230->ball_position_0p1mm;
        s_invalid_start_tick = now;
    }

    /*
     * IDLE only means that requirement 3 does not own the servo.
     * AppBall_StopInternal() has already commanded neutral once, so do not
     * repeatedly overwrite targets issued by the independent SERVO test page.
     */
    if (s_status.state == BALL_TASK_IDLE) {
        return;
    }

    if ((s_status.state == BALL_TASK_COMM_FAULT) ||
        (s_status.state == BALL_TASK_BALL_LOST) ||
        (s_status.state == BALL_TASK_TIMEOUT)) {
        AppBallPid_Reset(&s_ball_pid);
        AppBall_SetServoNeutral();
        return;
    }

    elapsed_ticks = (uint32_t)(now - s_task_start_tick);

    /*
     * Requirement 3 keeps timing beyond five seconds.  Five seconds is only
     * the scoring target, not a control timeout: the ball controller must
     * continue moving toward -5 cm until it is actually stable there.
     * Once COMPLETE is reached, keep the recorded time frozen.
     */
    if (s_status.state != BALL_TASK_COMPLETE) {
        s_status.elapsed_ms = elapsed_ticks * 10U;
    }

    /*
     * COMPLETE状态仍继续闭环保持-5cm；其它运行状态若视觉数据失效，
     * 先把摆杆恢复水平，再根据持续时间决定是否进入故障状态。
     */
    if (measurement_valid == 0U) {
        s_condition_start_tick = now;

        if ((k230->ball_online != 0U) &&
            ((uint32_t)(now - s_invalid_start_tick) <
             BALL_INVALID_HOLD_TICKS_10MS)) {
            AppBall_ApplyLostRecovery(now);
        } else if ((uint32_t)(now - s_invalid_start_tick) >=
                   BALL_INVALID_HOLD_TICKS_10MS) {
            AppBallPid_Reset(&s_ball_pid);
            AppBall_SetServoNeutral();
            s_have_last_position = 0U;
            s_fast_speed_0p1mm_per_s = 0;
            s_speed_0p1mm_per_s = 0;
            s_status.speed_0p1mm_per_s = 0;
            AppBall_ResetBreakaway(now, s_last_position_0p1mm);
        }

        if (k230->ball_online == 0U) {
            if (((s_status.state != BALL_TASK_LEVEL_PIPE) &&
                 (s_status.state != BALL_TASK_WAIT_CENTER)) ||
                (elapsed_ticks >= BALL_START_WAIT_TICKS_10MS)) {
                s_status.state = BALL_TASK_COMM_FAULT;
                s_status.running = 0U;
                AppBallPid_Reset(&s_ball_pid);
            }
        } else if ((uint32_t)(now - s_invalid_start_tick) >=
                   BALL_LOST_TICKS_10MS) {
            s_status.state = BALL_TASK_BALL_LOST;
            s_status.running = 0U;
            AppBallPid_Reset(&s_ball_pid);
        }
        return;
    }

    AppBall_UpdateController(k230);
    error_abs = AppBall_Abs16(
        (int16_t)(s_status.target_0p1mm - s_status.position_0p1mm));
    speed_abs = AppBall_Abs16(s_status.speed_0p1mm_per_s);

    switch (s_status.state) {
        case BALL_TASK_LEVEL_PIPE:
        {
            uint32_t level_elapsed =
                (uint32_t)(now - s_level_start_tick);

            /*
             * 水平完成条件：
             *   1. 已至少观察100ms；
             *   2. 小球仍位于中心可见区域；
             *   3. 漂移速度连续80ms不超过8mm/s。
             *
             * 300ms达到硬上限时无条件进入WAIT_CENTER，避免自动调平
             * 占用过多要求三的5秒时间。
             */
            if (level_elapsed >= BALL_LEVEL_MAX_TICKS_10MS) {
                s_reference_0p1mm = s_status.position_0p1mm;
                AppBall_SetState(
                    BALL_TASK_WAIT_CENTER,
                    BALL_CENTER_TARGET_0P1MM);
            } else if (
                (level_elapsed >= BALL_LEVEL_MIN_TICKS_10MS) &&
                (AppBall_Abs16(s_status.position_0p1mm) <=
                 BALL_LEVEL_POSITION_WINDOW_0P1MM) &&
                (speed_abs <= BALL_LEVEL_DRIFT_SPEED_0P1MM_PER_S)) {
                if ((uint32_t)(now - s_condition_start_tick) >=
                    BALL_LEVEL_CONFIRM_TICKS_10MS) {
                    s_reference_0p1mm = s_status.position_0p1mm;
                    AppBall_SetState(
                        BALL_TASK_WAIT_CENTER,
                        BALL_CENTER_TARGET_0P1MM);
                }
            } else {
                s_condition_start_tick = now;
            }
            break;
        }

        case BALL_TASK_WAIT_CENTER:
            if ((error_abs <= BALL_CENTER_TOLERANCE_0P1MM) &&
                (speed_abs <= BALL_CENTER_SPEED_MAX_0P1MM_PER_S)) {
                /*
                 * 只要中心位置有效并进入±5mm范围，就开始+5cm动作。
                 * 中心阶段不额外长时间停留，把5秒预算留给往返运动。
                 */
                if ((uint32_t)(now - s_condition_start_tick) >=
                    BALL_CENTER_CONFIRM_TICKS_10MS) {
                    s_reference_0p1mm = s_status.position_0p1mm;
                    AppBall_SetState(
                        BALL_TASK_MOVE_POSITIVE,
                        BALL_POSITIVE_TARGET_0P1MM);
                }
            } else {
                s_condition_start_tick = now;
            }
            break;

        case BALL_TASK_MOVE_POSITIVE:
            /*
             * Segment 1: use a smaller tilt and reverse at +45 mm, before the
             * ball reaches +5 cm with excessive outward speed.
             */
            if (s_status.position_0p1mm >=
                BALL_POSITIVE_SWITCH_POSITION_0P1MM) {
                AppBall_SetState(
                    BALL_TASK_MOVE_NEGATIVE,
                    BALL_NEGATIVE_TARGET_0P1MM);
            }
            break;

        case BALL_TASK_MOVE_NEGATIVE:
            /*
             * Segment 2: +5 cm -> -5 cm.  Enter HOLD with a practical
             * capture window first; completion is checked with the tighter
             * final window in BALL_TASK_HOLD_NEGATIVE.
             */
            if ((error_abs <= BALL_NEGATIVE_CAPTURE_TOLERANCE_0P1MM) &&
                (speed_abs <= BALL_NEGATIVE_CAPTURE_SPEED_0P1MM_PER_S)) {
                AppBall_SetState(
                    BALL_TASK_HOLD_NEGATIVE,
                    BALL_NEGATIVE_TARGET_0P1MM);
            }
            break;

        case BALL_TASK_HOLD_NEGATIVE:
            /*
             * Keep the pulse holder latched while it corrects 3..10 mm
             * errors.  Only a genuine escape beyond 10 mm returns to the
             * stronger MOVE_NEGATIVE controller.
             */
            if (error_abs > BALL_NEGATIVE_HOLD_EXIT_0P1MM) {
                AppBall_SetState(
                    BALL_TASK_MOVE_NEGATIVE,
                    BALL_NEGATIVE_TARGET_0P1MM);
            } else if ((error_abs > BALL_FINAL_TOLERANCE_0P1MM) ||
                       (speed_abs > BALL_FINAL_SPEED_MAX_0P1MM_PER_S)) {
                s_condition_start_tick = now;
            } else if ((uint32_t)(now - s_condition_start_tick) >=
                       BALL_FINAL_STABLE_TICKS_10MS) {
                s_status.state = BALL_TASK_COMPLETE;
                s_status.running = 0U;
                s_status.completed = 1U;
                s_status.completion_ms = s_status.elapsed_ms;
                s_complete_hold_start_tick = now;
            }
            break;

        case BALL_TASK_COMPLETE:
            /*
             * Timing has stopped.  AppBall_UpdateController() continues the
             * dynamic -5 cm holder indefinitely; only an explicit STOP or a
             * new START request may leave this state.
             */
            break;

        default:
            break;
    }
}

void AppBall_GetStatus(AppBall_Status *status)
{
    if (status != 0) {
        *status = s_status;
    }
}
