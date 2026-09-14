#include "Hardware/board.h"
#include "Application/app_state.h"
#include "Application/app_req4.h"

/* ------------------------- A到B行驶参数 ------------------------- */

/* AB为1.5m；3820 count/m来自现有编码器标定。达到此值即认为车体通过B点。 */
#define REQ4_B_DISTANCE_COUNTS               \
    ((ENCODER_COUNTS_PER_METER * 3U) / 2U)
#define REQ4_TIMEOUT_MS                       (8000U)

/*
 * 目标11 count/10ms约等于0.288m/s，理论AB时间约5.2s，给8s限制保留较大余量。
 * 从4逐级升至11，每约150ms增加1，整个软启动约1s，减小启动加速度对钢球的扰动。
 */
#define REQ4_START_SPEED                      (4)
#define REQ4_CRUISE_SPEED                    (11)
#define REQ4_CURVE_SPEED                      (9)
#define REQ4_RECOVERY_SPEED                   (7)
#define REQ4_RAMP_SAMPLE_TICKS_3MS           (50U)

/*
 * 要求四只走AB直线，转向输出应比一圈模式柔和。这里仍使用OLED中已经调好的
 * g_track_kp_x10/g_track_kd_x10，但将结果再缩小并限制为±4个速度计数。
 */
#define REQ4_STEER_SOFTEN_DIVISOR             (3)
#define REQ4_STEER_LIMIT                      (4)
#define REQ4_ERROR_SLOW_MM                    (17)
#define REQ4_ERROR_RECOVERY_MM                (28)
#define REQ4_LINE_LOST_TICKS_3MS              (67U)

/* ------------------------- 中心稳球参数 ------------------------- */

#define REQ4_MIN_BALL_QUALITY                 (40U)
#define REQ4_BALANCE_INITIAL_US               SERVO_CENTER_PULSE_US
#define REQ4_SERVO_RANGE_US                   (35U)
#define REQ4_SERVO_MIN_US                     \
    (REQ4_BALANCE_INITIAL_US - REQ4_SERVO_RANGE_US)
#define REQ4_SERVO_MAX_US                     \
    (REQ4_BALANCE_INITIAL_US + REQ4_SERVO_RANGE_US)
#define REQ4_BALANCE_MIN_US                   REQ4_SERVO_MIN_US
#define REQ4_BALANCE_MAX_US                   REQ4_SERVO_MAX_US
#define REQ4_BALANCE_TRIM_STEP_US             (1)
#define REQ4_POSITION_KP_NUM                  (3)
#define REQ4_POSITION_KP_DEN                  (1)
#define REQ4_TARGET_SPEED_MAX_0P1MM_PER_S     (400)
#define REQ4_TARGET_SPEED_NEAR_0P1MM_PER_S    (150)
#define REQ4_NEAR_ERROR_0P1MM                 (50)
#define REQ4_SPEED_LOOP_DEN                   (18)
#define REQ4_EFFECTIVE_DEADZONE_US            (6)
#define REQ4_MAX_OFFSET_US                    ((int16_t)REQ4_SERVO_RANGE_US)
#define REQ4_DRIVE_SLEW_US_PER_FRAME          (4)
#define REQ4_BRAKE_SLEW_US_PER_FRAME          (10)
#define REQ4_HOLD_ENTER_ERROR_0P1MM           (25)
#define REQ4_HOLD_ENTER_SPEED_0P1MM_PER_S     (80)
#define REQ4_HOLD_ENTER_FRAMES                (3U)
#define REQ4_HOLD_EXIT_ERROR_0P1MM            (60)
#define REQ4_HOLD_EXIT_FRAMES                 (2U)
#define REQ4_HOLD_DAMPING_DEN                 (30)
#define REQ4_HOLD_MAX_OFFSET_US               (5)
#define REQ4_FRAME_AGE_FALLBACK_MS            (35)
#define REQ4_SERVO_RESPONSE_MS                (25)
#define REQ4_PREDICTION_MAX_MS                (120)
#define REQ4_BALANCE_TRIM_TICKS_10MS          (12U)  /* 每120ms最多微调1us */
#define REQ4_TRIM_SPEED_MAX_0P1MM_PER_S        (150)  /* 15mm/s */

#define REQ4_CENTER_DEADBAND_0P1MM            (30)   /* ±3mm内不动作 */
#define REQ4_CORRECT_THRESHOLD_0P1MM          (60)   /* ±6mm开始修正 */
#define REQ4_EMERGENCY_THRESHOLD_0P1MM        (80)   /* ±8mm加强修正 */
#define REQ4_ALLOWED_ERROR_0P1MM              (100)  /* 题目最大±10mm */
#define REQ4_LIMIT_CONFIRM_FRAMES             (2U)
#define REQ4_OUTWARD_CONFIRM_FRAMES           (2U)
#define REQ4_OUTWARD_CONFIRM_FRAMES           (2U)
#define REQ4_SPEED_DEADBAND_0P1MM_PER_S       (50)
#define REQ4_SPEED_LIMIT_0P1MM_PER_S          (3000)

#undef REQ4_CENTER_DEADBAND_0P1MM
#define REQ4_CENTER_DEADBAND_0P1MM            (20)

/*
 * 皮筋预紧后机构最小有效控制量约32us，使用35us留出摩擦余量。
 * 目标附近不连续保持±35us，而是动作20ms后回到动态平衡值等待80ms。
 * 超过8mm时使用45us短脉冲加强拉回，但不改变电机循迹状态。
 */
#define REQ4_CORRECT_PULSE_US                 (35)
#define REQ4_EMERGENCY_PULSE_US               (45)
#define REQ4_CORRECT_PULSE_TICKS_10MS         (2U)
#define REQ4_EMERGENCY_PULSE_TICKS_10MS       (2U)
#define REQ4_SETTLE_TICKS_10MS                (8U)

/* 与要求三当前实车方向保持一致：正控制量使球向POS10正方向运动。 */
#define REQ4_SERVO_DIRECTION                  (-1)
#define REQ4_CONTROL_SERVO_ID                 (1U)

static AppReq4_Status s_status;
static volatile uint8_t s_start_request = 0U;
static volatile uint8_t s_stop_request = 0U;
static uint8_t s_limit_confirm_count = 0U;
static uint8_t s_hold_enter_count = 0U;
static uint8_t s_hold_exit_count = 0U;
static uint8_t s_have_measurement = 0U;
static uint32_t s_last_ball_sequence = 0U;
static uint32_t s_last_measurement_tick = 0U;
static uint32_t s_key_start_tick = 0U;
static uint16_t s_balance_pulse_us = REQ4_BALANCE_INITIAL_US;
static int16_t s_last_position_0p1mm = 0;
static int16_t s_cascade_offset_us = 0;
static uint8_t s_hold_latched = 0U;
static int16_t s_tune_position_kp_x10 = 30;
static int16_t s_tune_speed_kp_x100 = 6;
static int16_t s_tune_max_offset_us = 35;
static int16_t s_tune_drive_step_us = 4;
static int16_t s_tune_brake_step_us = 10;
/* Retained only so the old pulse controller can remain available for A/B tests. */
static uint8_t s_outward_confirm_count = 0U;
static uint32_t s_correction_start_tick = 0U;
static uint32_t s_last_trim_tick = 0U;
static uint16_t s_correction_duration_ticks = 0U;
static int8_t s_pulse_direction = 0;

static void AppReq4_CommandServoOffset(int16_t abstract_offset_us);

static int16_t AppReq4_Abs16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static int32_t AppReq4_Limit32(int32_t value, int32_t min, int32_t max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

static uint16_t AppReq4_GetBalancePulse(void)
{
    return s_balance_pulse_us;
}

/*
 * direction使用与短脉冲相同的抽象方向：+1使球向正方向运动，-1使球向负
 * 方向运动。动态平衡值每次只改变1us，避免凸轮回差导致基准快速漂走。
 */
static void AppReq4_TrimBalance(int8_t direction)
{
    int32_t pulse = (int32_t)s_balance_pulse_us +
        (REQ4_SERVO_DIRECTION * direction * REQ4_BALANCE_TRIM_STEP_US);

    pulse = AppReq4_Limit32(
        pulse, REQ4_BALANCE_MIN_US, REQ4_BALANCE_MAX_US);
    s_balance_pulse_us = (uint16_t)pulse;
    s_status.balance_pulse_us = s_balance_pulse_us;
    AppReq4_CommandServoOffset(0);
}

/*
 * 直接同时更新“当前位置”和“目标位置”，消除Servo_Update()渐近跟随造成的额外延迟。
 * 第二路舵机保持原来的当前位置和目标，不会被要求四意外拉回中位。
 */
static void AppReq4_SetServoTargetSmooth(uint16_t pulse_us)
{
    float other_target;

    pulse_us = Servo_LimitPulse(
        pulse_us, REQ4_SERVO_MIN_US, REQ4_SERVO_MAX_US);

#if REQ4_CONTROL_SERVO_ID == 1U
    other_target = Servo_Target2;
    Servo_SetTarget((float)pulse_us, other_target);
#else
    other_target = Servo_Target1;
    Servo_SetTarget(other_target, (float)pulse_us);
#endif
}

static void AppReq4_CommandServoOffset(int16_t abstract_offset_us)
{
    int32_t pulse = (int32_t)AppReq4_GetBalancePulse() +
        (REQ4_SERVO_DIRECTION * (int32_t)abstract_offset_us);

    pulse = AppReq4_Limit32(
        pulse, (int32_t)REQ4_SERVO_MIN_US, (int32_t)REQ4_SERVO_MAX_US);

    /*
     * 主循环频率远高于舵机20ms帧率。目标没有变化时不要反复写PWM，也不要
     * 反复清零另一只舵机的内部微分历史；状态切换时再立即写入即可。
     */
#if REQ4_CONTROL_SERVO_ID == 1U
    if ((s_status.servo_offset_us == abstract_offset_us) &&
        (s_status.servo_target_us == (uint16_t)pulse) &&
        (Servo_Target1 == (float)pulse)) {
        return;
    }
#else
    if ((s_status.servo_offset_us == abstract_offset_us) &&
        (s_status.servo_target_us == (uint16_t)pulse) &&
        (Servo_Target2 == (float)pulse)) {
        return;
    }
#endif

    s_status.servo_offset_us = abstract_offset_us;
    s_status.servo_target_us = (uint16_t)pulse;
    AppReq4_SetServoTargetSmooth((uint16_t)pulse);
}

static void AppReq4_StopCar(void)
{
    Flag_Stop = 1;
    g_track_targetA = 0;
    g_track_targetB = 0;
    PWMA = 0;
    PWMB = 0;
    Motor_PID_Reset();
    Set_PWM(0, 0);
}

static void AppReq4_StopInternal(void)
{
    AppReq4_StopCar();
    s_balance_pulse_us = REQ4_BALANCE_INITIAL_US;
    s_status.state = REQ4_STATE_IDLE;
    s_status.running = 0U;
    s_status.completed = 0U;
    s_status.elapsed_ms = 0U;
    s_status.completion_ms = 0U;
    s_status.distance_count = 0U;
    s_status.speed_0p1mm_per_s = 0;
    s_status.max_abs_position_0p1mm = 0;
    s_status.correction_state = REQ4_CORRECTION_OBSERVE;
    s_limit_confirm_count = 0U;
    s_outward_confirm_count = 0U;
    s_hold_enter_count = 0U;
    s_hold_exit_count = 0U;
    s_hold_latched = 0U;
    s_cascade_offset_us = 0;
    s_have_measurement = 0U;
    s_pulse_direction = 0;
    s_status.vision_fault = 0U;
    s_status.ball_limit_exceeded = 0U;
    s_status.line_lost = 0U;
    s_status.timed_out = 0U;
    s_status.balance_pulse_us = s_balance_pulse_us;
    AppReq4_CommandServoOffset(0);
}

static uint8_t AppReq4_IsMeasurementValid(const K230_Status *k230)
{
    if ((k230 == 0) ||
        (k230->ball_online == 0U) ||
        (k230->ball_valid == 0U) ||
        (k230->ball_quality < REQ4_MIN_BALL_QUALITY)) {
        return 0U;
    }
    return 1U;
}

static void AppReq4_StartCar(void)
{
    uint32_t now = K230_GetTick10ms();

    s_balance_pulse_us = REQ4_BALANCE_INITIAL_US;
    s_status.state = REQ4_STATE_RUNNING;
    s_status.running = 1U;
    s_status.completed = 0U;
    /* 计时从按键请求开始，而不是从电机开始转动才计时。 */
    s_status.elapsed_ms = (uint32_t)(now - s_key_start_tick) * 10U;
    s_status.completion_ms = 0U;
    s_status.distance_count = 0U;
    s_status.max_abs_position_0p1mm =
        AppReq4_Abs16(s_status.position_0p1mm);
    s_status.correction_state = REQ4_CORRECTION_OBSERVE;
    s_limit_confirm_count = 0U;
    s_outward_confirm_count = 0U;
    s_hold_enter_count = 0U;
    s_hold_exit_count = 0U;
    s_hold_latched = 0U;
    s_cascade_offset_us = 0;
    s_pulse_direction = 0;
    s_status.vision_fault = 0U;
    s_status.ball_limit_exceeded = 0U;
    s_status.line_lost = 0U;
    s_status.timed_out = 0U;
    s_status.balance_pulse_us = s_balance_pulse_us;
    s_last_trim_tick = now;

    g_motor_test_mode = 0U;
    g_follow_restart = 1U;
    g_follow_state = FOLLOW_STATE_RUNNING;
    g_lap_distance_x2 = 0U;
    g_run_time_ms = s_status.elapsed_ms;
    g_track_targetA = REQ4_START_SPEED;
    g_track_targetB = REQ4_START_SPEED;
    Motor_PID_Reset();
    Flag_Stop = 0;
    AppReq4_CommandServoOffset(0);
    g_oled_update = 1U;
}

static void AppReq4_UpdateMeasurement(
    const K230_Status *k230, uint32_t now)
{
    uint32_t dt_ticks;
    int32_t raw_speed;
    int32_t filtered_speed;
    int16_t abs_position;

    if (k230->ball_sequence == s_last_ball_sequence) {
        return;
    }

    s_last_ball_sequence = k230->ball_sequence;
    s_status.position_0p1mm = k230->ball_position_0p1mm;
    abs_position = AppReq4_Abs16(s_status.position_0p1mm);
    if (abs_position > s_status.max_abs_position_0p1mm) {
        s_status.max_abs_position_0p1mm = abs_position;
    }

    if (s_have_measurement == 0U) {
        s_have_measurement = 1U;
        s_status.speed_0p1mm_per_s = 0;
    } else {
        dt_ticks = (uint32_t)(now - s_last_measurement_tick);
        if (dt_ticks == 0U) {
            dt_ticks = 1U;
        }
        raw_speed =
            ((int32_t)s_status.position_0p1mm - s_last_position_0p1mm) *
            100 / (int32_t)dt_ticks;
        raw_speed = AppReq4_Limit32(
            raw_speed,
            -REQ4_SPEED_LIMIT_0P1MM_PER_S,
            REQ4_SPEED_LIMIT_0P1MM_PER_S);

        /* 30%新速度 + 70%历史速度，兼顾抗像素抖动和向外趋势判断。 */
        filtered_speed =
            (3 * raw_speed + 7 * (int32_t)s_status.speed_0p1mm_per_s) / 10;
        if (AppReq4_Abs16((int16_t)filtered_speed) <
            REQ4_SPEED_DEADBAND_0P1MM_PER_S) {
            filtered_speed = 0;
        }
        s_status.speed_0p1mm_per_s = (int16_t)filtered_speed;
    }

    s_last_position_0p1mm = s_status.position_0p1mm;
    s_last_measurement_tick = now;
}

static void AppReq4_StartCorrection(int8_t direction, uint8_t emergency)
{
    int16_t pulse_us;

    s_pulse_direction = direction;
    s_status.correction_state = REQ4_CORRECTION_PULSE;
    s_correction_start_tick = K230_GetTick10ms();
    if (emergency != 0U) {
        pulse_us = REQ4_EMERGENCY_PULSE_US;
        s_correction_duration_ticks = REQ4_EMERGENCY_PULSE_TICKS_10MS;
    } else {
        pulse_us = REQ4_CORRECT_PULSE_US;
        s_correction_duration_ticks = REQ4_CORRECT_PULSE_TICKS_10MS;
    }

    AppReq4_CommandServoOffset((int16_t)(direction * pulse_us));
}

static __attribute__((unused)) void AppReq4_UpdateCorrection(
    uint32_t now, uint8_t new_frame)
{
    int16_t position = s_status.position_0p1mm;
    int16_t speed = s_status.speed_0p1mm_per_s;
    int16_t abs_position = AppReq4_Abs16(position);
    uint8_t moving_outward =
        (((position > 0) && (speed > 0)) ||
         ((position < 0) && (speed < 0))) ? 1U : 0U;

    switch (s_status.correction_state) {
        case REQ4_CORRECTION_OBSERVE:
            AppReq4_CommandServoOffset(0);
            if (new_frame == 0U) {
                break;
            }

            if (abs_position <= REQ4_CENTER_DEADBAND_0P1MM) {
                s_outward_confirm_count = 0U;
                break;
            }

            /*
             * 3~6mm属于小偏差区。球速较低说明主要是当前机械平衡点发生漂移，
             * 此时不打35us脉冲，而是每120ms把动态平衡值向回中心方向微调1us。
             */
            if (abs_position < REQ4_CORRECT_THRESHOLD_0P1MM) {
                s_outward_confirm_count = 0U;
                if ((AppReq4_Abs16(speed) <=
                     REQ4_TRIM_SPEED_MAX_0P1MM_PER_S) &&
                    ((uint32_t)(now - s_last_trim_tick) >=
                     REQ4_BALANCE_TRIM_TICKS_10MS)) {
                    AppReq4_TrimBalance((position > 0) ? -1 : 1);
                    s_last_trim_tick = now;
                }
                break;
            }

            if (abs_position >= REQ4_EMERGENCY_THRESHOLD_0P1MM) {
                /* 球在正侧时给负控制量，球在负侧时给正控制量。 */
                AppReq4_StartCorrection(
                    (position > 0) ? -1 : 1, 1U);
                s_outward_confirm_count = 0U;
                break;
            }

            if ((abs_position >= REQ4_CORRECT_THRESHOLD_0P1MM) &&
                ((moving_outward != 0U) || (speed == 0))) {
                if (s_outward_confirm_count < REQ4_OUTWARD_CONFIRM_FRAMES) {
                    s_outward_confirm_count++;
                }
                if (s_outward_confirm_count >= REQ4_OUTWARD_CONFIRM_FRAMES) {
                    AppReq4_StartCorrection(
                        (position > 0) ? -1 : 1, 0U);
                    s_outward_confirm_count = 0U;
                }
            } else {
                s_outward_confirm_count = 0U;
            }
            break;

        case REQ4_CORRECTION_PULSE:
            if ((uint32_t)(now - s_correction_start_tick) >=
                s_correction_duration_ticks) {
                AppReq4_CommandServoOffset(0);
                s_status.correction_state = REQ4_CORRECTION_SETTLE;
                s_correction_start_tick = now;
            }
            break;

        case REQ4_CORRECTION_SETTLE:
            AppReq4_CommandServoOffset(0);
            if ((uint32_t)(now - s_correction_start_tick) >=
                REQ4_SETTLE_TICKS_10MS) {
                s_status.correction_state = REQ4_CORRECTION_OBSERVE;
                s_outward_confirm_count = 0U;
            }
            break;

        default:
            s_status.correction_state = REQ4_CORRECTION_OBSERVE;
            AppReq4_CommandServoOffset(0);
            break;
    }
}

static int16_t AppReq4_MoveOffsetToward(
    int16_t current, int16_t target, int16_t step)
{
    if (target > (int16_t)(current + step)) {
        return (int16_t)(current + step);
    }
    if (target < (int16_t)(current - step)) {
        return (int16_t)(current - step);
    }
    return target;
}

/*
 * Legal cascade without a gyro:
 * K230 ball position -> requested ball speed -> servo pulse target ->
 * the 180-degree servo's internal position loop.
 */
static void AppReq4_UpdateCascade(
    const K230_Status *k230, uint32_t now, uint8_t new_frame)
{
    int32_t frame_age_ms;
    int32_t predicted_position;
    int32_t position_error;
    int32_t error_abs;
    int32_t speed_abs;
    int32_t target_speed;
    int32_t speed_error;
    int32_t requested_offset;
    int16_t slew_step;
    uint8_t braking;

    if (new_frame == 0U) {
        return;
    }

    speed_abs = AppReq4_Abs16(s_status.speed_0p1mm_per_s);
    if (s_hold_latched != 0U) {
        if (AppReq4_Abs16(s_status.position_0p1mm) >
            REQ4_HOLD_EXIT_ERROR_0P1MM) {
            if (s_hold_exit_count < REQ4_HOLD_EXIT_FRAMES) {
                s_hold_exit_count++;
            }
        } else {
            s_hold_exit_count = 0U;
        }
        if (s_hold_exit_count >= REQ4_HOLD_EXIT_FRAMES) {
            s_hold_latched = 0U;
            s_hold_exit_count = 0U;
            s_hold_enter_count = 0U;
        }
    } else {
        if ((AppReq4_Abs16(s_status.position_0p1mm) <=
             REQ4_HOLD_ENTER_ERROR_0P1MM) &&
            (speed_abs <= REQ4_HOLD_ENTER_SPEED_0P1MM_PER_S)) {
            if (s_hold_enter_count < REQ4_HOLD_ENTER_FRAMES) {
                s_hold_enter_count++;
            }
        } else {
            s_hold_enter_count = 0U;
        }
        if (s_hold_enter_count >= REQ4_HOLD_ENTER_FRAMES) {
            s_hold_latched = 1U;
            s_hold_enter_count = 0U;
            s_hold_exit_count = 0U;
        }
    }

    if (s_hold_latched != 0U) {
        requested_offset =
            -(int32_t)s_status.speed_0p1mm_per_s / REQ4_HOLD_DAMPING_DEN;
        requested_offset = AppReq4_Limit32(
            requested_offset,
            -REQ4_HOLD_MAX_OFFSET_US,
            REQ4_HOLD_MAX_OFFSET_US);
        s_status.correction_state = REQ4_CORRECTION_SETTLE;
    } else {
        frame_age_ms = (k230->ball_frame_age_ms != 0U) ?
            (int32_t)k230->ball_frame_age_ms :
            REQ4_FRAME_AGE_FALLBACK_MS;
        frame_age_ms += REQ4_SERVO_RESPONSE_MS +
            (int32_t)(now - s_last_measurement_tick) * 10;
        frame_age_ms = AppReq4_Limit32(
            frame_age_ms, 0, REQ4_PREDICTION_MAX_MS);
        predicted_position = (int32_t)s_status.position_0p1mm +
            (int32_t)s_status.speed_0p1mm_per_s * frame_age_ms / 1000;
        position_error = -predicted_position;
        error_abs = (position_error < 0) ? -position_error : position_error;

        if (error_abs <= REQ4_CENTER_DEADBAND_0P1MM) {
            target_speed = 0;
        } else {
            target_speed = position_error * s_tune_position_kp_x10 / 10;
            target_speed = AppReq4_Limit32(
                target_speed,
                -REQ4_TARGET_SPEED_MAX_0P1MM_PER_S,
                REQ4_TARGET_SPEED_MAX_0P1MM_PER_S);
            if (error_abs <= REQ4_NEAR_ERROR_0P1MM) {
                target_speed = AppReq4_Limit32(
                    target_speed,
                    -REQ4_TARGET_SPEED_NEAR_0P1MM_PER_S,
                    REQ4_TARGET_SPEED_NEAR_0P1MM_PER_S);
            }
        }

        speed_error = target_speed - s_status.speed_0p1mm_per_s;
        requested_offset = speed_error * s_tune_speed_kp_x100 / 100;
        if ((error_abs > REQ4_CENTER_DEADBAND_0P1MM) &&
            (requested_offset != 0) &&
            (requested_offset < REQ4_EFFECTIVE_DEADZONE_US) &&
            (requested_offset > -REQ4_EFFECTIVE_DEADZONE_US)) {
            requested_offset = (requested_offset > 0) ?
                REQ4_EFFECTIVE_DEADZONE_US : -REQ4_EFFECTIVE_DEADZONE_US;
        }
        requested_offset = AppReq4_Limit32(
            requested_offset,
            -(int32_t)s_tune_max_offset_us,
            (int32_t)s_tune_max_offset_us);
        braking = ((requested_offset *
            (int32_t)s_status.speed_0p1mm_per_s) < 0) ? 1U : 0U;
        s_status.correction_state = (braking != 0U) ?
            REQ4_CORRECTION_PULSE : REQ4_CORRECTION_OBSERVE;
    }

    braking = ((requested_offset *
        (int32_t)s_status.speed_0p1mm_per_s) < 0) ? 1U : 0U;
    slew_step = (braking != 0U) ?
        s_tune_brake_step_us : s_tune_drive_step_us;
    s_cascade_offset_us = AppReq4_MoveOffsetToward(
        s_cascade_offset_us, (int16_t)requested_offset, slew_step);
    AppReq4_CommandServoOffset(s_cascade_offset_us);
}

void AppReq4_Init(void)
{
    s_balance_pulse_us = REQ4_BALANCE_INITIAL_US;
    s_status.state = REQ4_STATE_IDLE;
    s_status.correction_state = REQ4_CORRECTION_OBSERVE;
    s_status.running = 0U;
    s_status.completed = 0U;
    s_status.communication_ok = 0U;
    s_status.measurement_valid = 0U;
    s_status.vision_fault = 0U;
    s_status.ball_limit_exceeded = 0U;
    s_status.line_lost = 0U;
    s_status.timed_out = 0U;
    s_status.position_0p1mm = 0;
    s_status.speed_0p1mm_per_s = 0;
    s_status.max_abs_position_0p1mm = 0;
    s_status.servo_offset_us = 0;
    s_status.balance_pulse_us = s_balance_pulse_us;
    s_status.servo_target_us = AppReq4_GetBalancePulse();
    s_status.elapsed_ms = 0U;
    s_status.completion_ms = 0U;
    s_status.distance_count = 0U;
    s_hold_enter_count = 0U;
    s_hold_exit_count = 0U;
    s_hold_latched = 0U;
    s_cascade_offset_us = 0;
}

void AppReq4_RequestStart(void)
{
    s_start_request = 1U;
    s_stop_request = 0U;
}

void AppReq4_RequestStop(void)
{
    s_stop_request = 1U;
    s_start_request = 0U;
}

void AppReq4_BackgroundTask(const K230_Status *k230)
{
    uint32_t now;
    uint8_t measurement_valid;
    uint8_t new_frame = 0U;
    int16_t abs_position;

    if (k230 == 0) {
        return;
    }

    now = K230_GetTick10ms();
    measurement_valid = AppReq4_IsMeasurementValid(k230);
    s_status.communication_ok = k230->ball_online;
    s_status.measurement_valid = measurement_valid;

    if (s_stop_request != 0U) {
        s_stop_request = 0U;
        s_start_request = 0U;
        AppReq4_StopInternal();
    }

    if (measurement_valid != 0U) {
        new_frame = (k230->ball_sequence != s_last_ball_sequence) ? 1U : 0U;
        AppReq4_UpdateMeasurement(k230, now);
    }

    if (s_start_request != 0U) {
        s_start_request = 0U;
        AppReq4_StopCar();
        s_have_measurement = 0U;
        s_status.speed_0p1mm_per_s = 0;
        s_last_measurement_tick = now;
        s_key_start_tick = now;
        AppReq4_StartCar();
        return;
    }

    if (s_status.state == REQ4_STATE_IDLE) {
        if (measurement_valid != 0U) {
            s_status.position_0p1mm = k230->ball_position_0p1mm;
        }
        return;
    }

    if ((s_status.state != REQ4_STATE_RUNNING) &&
        (s_status.state != REQ4_STATE_COMPLETE) &&
        (s_status.state != REQ4_STATE_TIMEOUT)) {
        AppReq4_CommandServoOffset(0);
        return;
    }

    if (measurement_valid == 0U) {
        /* 任意一次视觉无效都先撤销修正脉冲，防止盲目保持倾角。 */
        s_status.correction_state = REQ4_CORRECTION_OBSERVE;
        s_outward_confirm_count = 0U;
        s_hold_enter_count = 0U;
        s_hold_exit_count = 0U;
        s_hold_latched = 0U;
        s_cascade_offset_us = 0;
        AppReq4_CommandServoOffset(0);
        s_status.vision_fault = 1U;
        return;
    }

    s_status.vision_fault = 0U;
    abs_position = AppReq4_Abs16(s_status.position_0p1mm);
    if ((new_frame != 0U) &&
        (abs_position > REQ4_ALLOWED_ERROR_0P1MM)) {
        if (s_limit_confirm_count < REQ4_LIMIT_CONFIRM_FRAMES) {
            s_limit_confirm_count++;
        }
        if (s_limit_confirm_count >= REQ4_LIMIT_CONFIRM_FRAMES) {
            /* 只锁存钢球越界结果；电机循迹不受钢球状态影响。 */
            s_status.ball_limit_exceeded = 1U;
        }
    } else if (new_frame != 0U) {
        s_limit_confirm_count = 0U;
    }

    AppReq4_UpdateCascade(k230, now, new_frame);
}

void AppReq4_UpdateDistance10ms(void)
{
    uint32_t countA;
    uint32_t countB;

    if ((g_menu_level != MENU_LEVEL_DETAIL) ||
        (g_menu_page != MENU_PAGE_REQ4) ||
        ((s_status.state != REQ4_STATE_RUNNING) &&
         (s_status.state != REQ4_STATE_COMPLETE) &&
         (s_status.state != REQ4_STATE_TIMEOUT)) ||
        (Flag_Stop != 0)) {
        return;
    }

    countA = (encoderA_cnt < 0) ?
        (uint32_t)(-encoderA_cnt) : (uint32_t)encoderA_cnt;
    countB = (encoderB_cnt < 0) ?
        (uint32_t)(-encoderB_cnt) : (uint32_t)encoderB_cnt;
    g_lap_distance_x2 += countA + countB;
    s_status.distance_count = g_lap_distance_x2 / 2U;

    if (s_status.elapsed_ms <= (UINT32_MAX - CONTROL_PERIOD_MS)) {
        s_status.elapsed_ms += CONTROL_PERIOD_MS;
    }
    g_run_time_ms = s_status.elapsed_ms;

    /* 先判断到达B，再判断8s超时，避免恰好在边界到达时误报超时。 */
    if ((s_status.completed == 0U) &&
        (s_status.distance_count >= REQ4_B_DISTANCE_COUNTS)) {
        s_status.state = REQ4_STATE_COMPLETE;
        s_status.running = 1U;
        s_status.completed = 1U;
        s_status.completion_ms = s_status.elapsed_ms;
        g_oled_update = 1U;
    } else if ((s_status.completed == 0U) &&
               (s_status.timed_out == 0U) &&
               (s_status.elapsed_ms >= REQ4_TIMEOUT_MS)) {
        s_status.state = REQ4_STATE_TIMEOUT;
        s_status.running = 1U;
        s_status.timed_out = 1U;
        g_oled_update = 1U;
    }
}

void AppReq4_UpdateTarget3ms(void)
{
    static int16_t last_error = 0;
    static int8_t last_direction = 0;
    static uint16_t line_lost_ticks = 0U;
    static uint16_t ramp_sample_ticks = 0U;
    static int32_t ramp_speed = REQ4_START_SPEED;
    uint8_t line_found;
    int16_t error;
    int16_t abs_error;
    int16_t derivative;
    int32_t base_speed;
    int32_t turn;
    int32_t targetA;
    int32_t targetB;

    g_track_mask = Track_ReadBlackMask();
    error = Track_GetError(g_track_mask, &line_found);
    g_track_error = error;
    g_line_found = line_found;

    if (g_follow_restart != 0U) {
        g_follow_restart = 0U;
        last_error = (line_found != 0U) ? error : 0;
        last_direction = 0;
        line_lost_ticks = 0U;
        ramp_sample_ticks = 0U;
        ramp_speed = REQ4_START_SPEED;
    }

    if (line_found == 0U) {
        if (line_lost_ticks < REQ4_LINE_LOST_TICKS_3MS) {
            line_lost_ticks++;
        }
        if (line_lost_ticks >= REQ4_LINE_LOST_TICKS_3MS) {
            /* 丢线后继续低速搜索，不让钢球或循迹异常自动关闭电机。 */
            if (s_status.line_lost == 0U) {
                s_status.line_lost = 1U;
                g_oled_update = 1U;
            }
            g_track_targetA = REQ4_RECOVERY_SPEED;
            g_track_targetB = REQ4_RECOVERY_SPEED / 2;
            return;
        }

        /* 短时丢线只沿最后方向做小幅搜索，不允许原地大角度转向扰动钢球。 */
        targetA = REQ4_RECOVERY_SPEED;
        targetB = REQ4_RECOVERY_SPEED;
        if (last_direction < 0) {
            targetA -= 2;
        } else if (last_direction > 0) {
            targetB -= 2;
        }
        g_track_targetA = targetA;
        g_track_targetB = targetB;
        return;
    }

    line_lost_ticks = 0U;
    if (s_status.line_lost != 0U) {
        s_status.line_lost = 0U;
        g_oled_update = 1U;
    }
    if (error < 0) {
        last_direction = -1;
    } else if (error > 0) {
        last_direction = 1;
    }

    abs_error = AppReq4_Abs16(error);
    if (abs_error > REQ4_ERROR_RECOVERY_MM) {
        base_speed = REQ4_RECOVERY_SPEED;
    } else if (abs_error > REQ4_ERROR_SLOW_MM) {
        base_speed = REQ4_CURVE_SPEED;
    } else {
        base_speed = REQ4_CRUISE_SPEED;
    }

    if (ramp_sample_ticks < REQ4_RAMP_SAMPLE_TICKS_3MS) {
        ramp_sample_ticks++;
    } else {
        ramp_sample_ticks = 0U;
        if (ramp_speed < REQ4_CRUISE_SPEED) {
            ramp_speed++;
        }
    }
    if (base_speed > ramp_speed) {
        base_speed = ramp_speed;
    }

    derivative = (int16_t)(error - last_error);
    last_error = error;
    turn = ((int32_t)g_track_kp_x10 * error +
            (int32_t)g_track_kd_x10 * derivative) / TRACK_PD_SCALE;
    turn /= REQ4_STEER_SOFTEN_DIVISOR;
    turn = AppReq4_Limit32(turn, -REQ4_STEER_LIMIT, REQ4_STEER_LIMIT);

    targetA = base_speed + turn;
    targetB = base_speed - turn;
    targetA = AppReq4_Limit32(targetA, 0, TRACK_TARGET_MAX);
    targetB = AppReq4_Limit32(targetB, 0, TRACK_TARGET_MAX);
    g_track_targetA = targetA;
    g_track_targetB = targetB;
}

void AppReq4_GetStatus(AppReq4_Status *status)
{
    if (status != 0) {
        *status = s_status;
    }
}

int32_t AppReq4_GetTuneParameter(AppReq4_TuneParameter parameter)
{
    switch (parameter) {
        case APP_REQ4_TUNE_POSITION_KP_X10:
            return s_tune_position_kp_x10;
        case APP_REQ4_TUNE_SPEED_KP_X100:
            return s_tune_speed_kp_x100;
        case APP_REQ4_TUNE_MAX_OFFSET_US:
            return s_tune_max_offset_us;
        case APP_REQ4_TUNE_DRIVE_STEP_US:
            return s_tune_drive_step_us;
        case APP_REQ4_TUNE_BRAKE_STEP_US:
            return s_tune_brake_step_us;
        default:
            return 0;
    }
}

void AppReq4_AdjustTuneParameter(
    AppReq4_TuneParameter parameter, int8_t direction)
{
    int32_t step = (direction >= 0) ? 1 : -1;

    switch (parameter) {
        case APP_REQ4_TUNE_POSITION_KP_X10:
            s_tune_position_kp_x10 = (int16_t)AppReq4_Limit32(
                (int32_t)s_tune_position_kp_x10 + step, 5, 60);
            break;

        case APP_REQ4_TUNE_SPEED_KP_X100:
            s_tune_speed_kp_x100 = (int16_t)AppReq4_Limit32(
                (int32_t)s_tune_speed_kp_x100 + step, 1, 15);
            break;

        case APP_REQ4_TUNE_MAX_OFFSET_US:
            s_tune_max_offset_us = (int16_t)AppReq4_Limit32(
                (int32_t)s_tune_max_offset_us + step, 5, 35);
            break;

        case APP_REQ4_TUNE_DRIVE_STEP_US:
            s_tune_drive_step_us = (int16_t)AppReq4_Limit32(
                (int32_t)s_tune_drive_step_us + step, 1, 10);
            if (s_tune_brake_step_us < s_tune_drive_step_us) {
                s_tune_brake_step_us = s_tune_drive_step_us;
            }
            break;

        case APP_REQ4_TUNE_BRAKE_STEP_US:
            s_tune_brake_step_us = (int16_t)AppReq4_Limit32(
                (int32_t)s_tune_brake_step_us + step, 2, 20);
            if (s_tune_drive_step_us > s_tune_brake_step_us) {
                s_tune_drive_step_us = s_tune_brake_step_us;
            }
            break;

        default:
            break;
    }
}
