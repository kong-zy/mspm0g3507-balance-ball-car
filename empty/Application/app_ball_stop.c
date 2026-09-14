#include "Hardware/board.h"
#include "Application/app_ball.h"
#include "Application/app_ball_stop.h"

/*
 * Independent ball brake test:
 *
 *   any visible position -> -50.0 mm -> brake -> hold indefinitely
 *
 * This module deliberately does not reuse the requirement-3 state machine.
 * Requirement 3 remains responsible for O -> +5 cm -> -5 cm, while BSTOP is
 * a small single-target controller intended for mechanical and brake tuning.
 */

#define BALL_STOP_SERVO_ID                         (1U)
#define BALL_STOP_SERVO_DIRECTION                  (1)
#define BALL_STOP_TARGET_0P1MM                     (-500)
#define BALL_STOP_MIN_QUALITY                      (40U)
#define BALL_STOP_CENTER_MIN_US                    \
    ((int32_t)SERVO_CENTER_PULSE_US - 175)
#define BALL_STOP_CENTER_MAX_US                    \
    ((int32_t)SERVO_CENTER_PULSE_US + 75)

/* Position-to-speed outer loop. */
#define BALL_STOP_POSITION_KP_NUM                  (3)
#define BALL_STOP_POSITION_KP_DEN                  (2)
#define BALL_STOP_SPEED_FAR_0P1MM_PER_S            (700)
#define BALL_STOP_SPEED_MEDIUM_0P1MM_PER_S         (550)
#define BALL_STOP_SPEED_NEAR_0P1MM_PER_S           (350)
#define BALL_STOP_SPEED_CAPTURE_0P1MM_PER_S        (100)
#define BALL_STOP_FAR_ERROR_0P1MM                  (400)
#define BALL_STOP_MEDIUM_ERROR_0P1MM               (200)
#define BALL_STOP_NEAR_ERROR_0P1MM                 (100)
#define BALL_STOP_POSITION_DEADBAND_0P1MM          (30)

/* Speed loop and smooth servo output. */
#define BALL_STOP_SPEED_KP_DEN                     (9)
#define BALL_STOP_DEADZONE_TRIGGER_0P1MM           (150)
#define BALL_STOP_DEADZONE_US                      (35)
#define BALL_STOP_MAX_OFFSET_US                    (80)
#define BALL_STOP_DRIVE_STEP_US                    (3)
#define BALL_STOP_BRAKE_STEP_US                    (6)
#define BALL_STOP_CAPTURE_BRAKE_MIN_US             (35)
#define BALL_STOP_CAPTURE_BRAKE_SPEED_0P1MM_PER_S  (120)

/* Camera frame age + measured frame period + 180-degree servo response. */
#define BALL_STOP_SERVO_RESPONSE_MS                (25)
#define BALL_STOP_UART_TRANSPORT_MS                 (5)
#define BALL_STOP_FRAME_PERIOD_INITIAL_MS          (35)
#define BALL_STOP_FRAME_PERIOD_MIN_MS              (10)
#define BALL_STOP_FRAME_PERIOD_MAX_MS              (100)
#define BALL_STOP_PREDICTION_MIN_MS                (30)
#define BALL_STOP_PREDICTION_MAX_MS                (150)

/* The static balance point may differ slightly from the tune-page center. */
#define BALL_STOP_BALANCE_TRIM_US                  (45)
#define BALL_STOP_TRIM_ERROR_MIN_0P1MM             (35)
#define BALL_STOP_TRIM_ERROR_MAX_0P1MM             (80)
#define BALL_STOP_TRIM_SPEED_MAX_0P1MM_PER_S       (80)
#define BALL_STOP_TRIM_INTERVAL_TICKS_10MS         (30U)

/* State hysteresis and completion judgment. */
#define BALL_STOP_HOLD_ENTER_ERROR_0P1MM           (60)
#define BALL_STOP_HOLD_ENTER_SPEED_0P1MM_PER_S     (100)
#define BALL_STOP_HOLD_ENTER_TICKS_10MS            (10U)
#define BALL_STOP_HOLD_EXIT_ERROR_0P1MM            (80)
#define BALL_STOP_HOLD_DAMPING_DEN                 (12)
#define BALL_STOP_HOLD_MAX_OFFSET_US               (10)
#define BALL_STOP_HOLD_STEP_US                     (1)
#define BALL_STOP_FINAL_ERROR_0P1MM                (60)
#define BALL_STOP_FINAL_SPEED_0P1MM_PER_S          (100)
#define BALL_STOP_FINAL_TICKS_10MS                 (50U)

#define BALL_STOP_INVALID_FAULT_TICKS_10MS         (20U)
#define BALL_STOP_SPEED_HISTORY                    (4U)

/* Reject one-frame UART/vision position spikes without delaying normal data. */
#define BALL_STOP_JUMP_BASE_0P1MM                  (60)
#define BALL_STOP_JUMP_PER_TICK_0P1MM              (50)
#define BALL_STOP_JUMP_MAX_0P1MM                   (300)
#define BALL_STOP_REACQUIRE_WINDOW_0P1MM           (60)

static AppBallStop_Status s_status;
static uint8_t s_start_request;
static uint8_t s_stop_request;
static uint32_t s_start_tick;
static uint32_t s_complete_condition_tick;
static uint32_t s_invalid_start_tick;
static uint32_t s_last_control_tick;
static uint32_t s_last_trim_tick;
static uint32_t s_hold_condition_tick;
static uint32_t s_last_ball_frame_count;
static int16_t s_start_center_us;
static int16_t s_balance_pulse_us;
static int16_t s_actual_offset_us;
static int16_t s_speed_0p1mm_per_s;
static int16_t s_position_history[BALL_STOP_SPEED_HISTORY];
static uint32_t s_tick_history[BALL_STOP_SPEED_HISTORY];
static uint8_t s_history_index;
static uint8_t s_history_count;
static uint8_t s_hold_latched;
static uint8_t s_have_accepted_position;
static uint8_t s_pending_outlier;
static int16_t s_last_accepted_position;
static int16_t s_pending_position;
static uint32_t s_last_accepted_tick;
static uint32_t s_last_measurement_tick;
static int32_t s_frame_period_ms;
static int32_t s_reported_frame_age_ms;

static int32_t AppBallStop_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t AppBallStop_Limit32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t AppBallStop_MoveToward(
    int32_t current,
    int32_t target,
    int32_t step)
{
    if (current < target) {
        current += step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= step;
        if (current < target) {
            current = target;
        }
    }
    return current;
}

static void AppBallStop_SetServoImmediate(uint16_t pulse_us)
{
    pulse_us = Servo_LimitPulse(
        pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

#if BALL_STOP_SERVO_ID == 1U
    Servo_SetPulse(pulse_us, (uint16_t)(Servo_Position2 + 0.5f));
    Servo_SetTarget((float)pulse_us, Servo_Target2);
#else
    Servo_SetPulse((uint16_t)(Servo_Position1 + 0.5f), pulse_us);
    Servo_SetTarget(Servo_Target1, (float)pulse_us);
#endif
    s_status.servo_pulse_us = pulse_us;
}

static void AppBallStop_SetServoTarget(uint16_t pulse_us)
{
#if BALL_STOP_SERVO_ID == 1U
    Servo_SetTarget((float)pulse_us, Servo_Target2);
#else
    Servo_SetTarget(Servo_Target1, (float)pulse_us);
#endif
    s_status.servo_pulse_us = pulse_us;
}

static void AppBallStop_ResetSpeedEstimator(void)
{
    uint8_t i;

    s_history_index = 0U;
    s_history_count = 0U;
    s_speed_0p1mm_per_s = 0;
    for (i = 0U; i < BALL_STOP_SPEED_HISTORY; i++) {
        s_position_history[i] = 0;
        s_tick_history[i] = 0U;
    }
}

/*
 * A checksum protects bytes on the wire, but a valid K230 frame may still
 * contain a one-frame recognition jump.  Accept normal motion immediately;
 * require two mutually consistent frames before accepting a large jump.
 */
static uint8_t AppBallStop_AcceptPosition(
    int16_t position,
    uint32_t tick)
{
    uint32_t elapsed_ticks;
    int32_t maximum_jump;
    int32_t jump;

    if (s_have_accepted_position == 0U) {
        s_have_accepted_position = 1U;
        s_pending_outlier = 0U;
        s_last_accepted_position = position;
        s_last_accepted_tick = tick;
        return 1U;
    }

    elapsed_ticks = (uint32_t)(tick - s_last_accepted_tick);
    maximum_jump = BALL_STOP_JUMP_BASE_0P1MM +
        (int32_t)elapsed_ticks * BALL_STOP_JUMP_PER_TICK_0P1MM;
    maximum_jump = AppBallStop_Limit32(
        maximum_jump,
        BALL_STOP_JUMP_BASE_0P1MM,
        BALL_STOP_JUMP_MAX_0P1MM);
    jump = AppBallStop_Abs32(
        (int32_t)position - (int32_t)s_last_accepted_position);

    if (jump <= maximum_jump) {
        s_pending_outlier = 0U;
        s_last_accepted_position = position;
        s_last_accepted_tick = tick;
        return 1U;
    }

    if ((s_pending_outlier != 0U) &&
        (AppBallStop_Abs32(
            (int32_t)position - (int32_t)s_pending_position) <=
         BALL_STOP_REACQUIRE_WINDOW_0P1MM)) {
        /* The large move persisted for two frames: treat it as real motion. */
        s_pending_outlier = 0U;
        s_last_accepted_position = position;
        s_last_accepted_tick = tick;
        AppBallStop_ResetSpeedEstimator();
        return 1U;
    }

    s_pending_position = position;
    s_pending_outlier = 1U;
    return 0U;
}

static void AppBallStop_UpdateSpeed(int16_t position, uint32_t tick)
{
    int32_t raw_speed;
    uint32_t elapsed_ticks;

    if (s_history_count < BALL_STOP_SPEED_HISTORY) {
        s_position_history[s_history_index] = position;
        s_tick_history[s_history_index] = tick;
        s_history_index =
            (uint8_t)((s_history_index + 1U) % BALL_STOP_SPEED_HISTORY);
        s_history_count++;
        s_speed_0p1mm_per_s = 0;
        return;
    }

    elapsed_ticks = (uint32_t)(tick - s_tick_history[s_history_index]);
    if (elapsed_ticks > 0U) {
        raw_speed =
            ((int32_t)position -
             (int32_t)s_position_history[s_history_index]) * 100L /
            (int32_t)elapsed_ticks;
        raw_speed = AppBallStop_Limit32(raw_speed, -3000, 3000);

        /* 40 percent new speed: fast enough for latency prediction. */
        s_speed_0p1mm_per_s = (int16_t)(
            ((int32_t)s_speed_0p1mm_per_s * 3L + raw_speed * 2L) / 5L);
        if (AppBallStop_Abs32(s_speed_0p1mm_per_s) < 20) {
            s_speed_0p1mm_per_s = 0;
        }
    }

    s_position_history[s_history_index] = position;
    s_tick_history[s_history_index] = tick;
    s_history_index =
        (uint8_t)((s_history_index + 1U) % BALL_STOP_SPEED_HISTORY);
}

static int32_t AppBallStop_GetDesiredSpeed(int32_t error)
{
    int32_t error_abs = AppBallStop_Abs32(error);
    int32_t limit;
    int32_t speed;

    if (error_abs <= BALL_STOP_POSITION_DEADBAND_0P1MM) {
        return 0;
    }

    if (error_abs > BALL_STOP_FAR_ERROR_0P1MM) {
        limit = BALL_STOP_SPEED_FAR_0P1MM_PER_S;
    } else if (error_abs > BALL_STOP_MEDIUM_ERROR_0P1MM) {
        limit = BALL_STOP_SPEED_MEDIUM_0P1MM_PER_S;
    } else if (error_abs > BALL_STOP_NEAR_ERROR_0P1MM) {
        limit = BALL_STOP_SPEED_NEAR_0P1MM_PER_S;
    } else {
        limit = BALL_STOP_SPEED_CAPTURE_0P1MM_PER_S;
    }

    speed = error_abs * BALL_STOP_POSITION_KP_NUM /
        BALL_STOP_POSITION_KP_DEN;
    if (speed > limit) {
        speed = limit;
    }
    return (error < 0) ? -speed : speed;
}

static void AppBallStop_StartInternal(uint32_t now)
{
    s_start_center_us =
        AppBall_GetTuneParameter(APP_BALL_TUNE_CENTER_US);
    s_start_center_us = (int16_t)AppBallStop_Limit32(
        s_start_center_us,
        BALL_STOP_CENTER_MIN_US,
        BALL_STOP_CENTER_MAX_US);
    s_balance_pulse_us = s_start_center_us;
    s_actual_offset_us = 0;
    s_start_tick = now;
    s_complete_condition_tick = now;
    s_invalid_start_tick = now;
    s_last_control_tick = now;
    s_last_trim_tick = now;
    s_hold_condition_tick = now;
    s_last_ball_frame_count = 0U;
    s_last_measurement_tick = now;
    s_frame_period_ms = BALL_STOP_FRAME_PERIOD_INITIAL_MS;
    s_reported_frame_age_ms = 0;
    s_hold_latched = 0U;
    s_have_accepted_position = 0U;
    s_pending_outlier = 0U;
    AppBallStop_ResetSpeedEstimator();

    s_status.state = BALL_STOP_RUNNING;
    s_status.running = 1U;
    s_status.completed = 0U;
    s_status.control_offset_us = 0;
    s_status.balance_pulse_us = s_balance_pulse_us;
    s_status.elapsed_ms = 0U;
    s_status.completion_ms = 0U;
    AppBallStop_SetServoImmediate((uint16_t)s_balance_pulse_us);
}

static void AppBallStop_StopInternal(void)
{
    s_start_center_us =
        AppBall_GetTuneParameter(APP_BALL_TUNE_CENTER_US);
    s_start_center_us = (int16_t)AppBallStop_Limit32(
        s_start_center_us,
        BALL_STOP_CENTER_MIN_US,
        BALL_STOP_CENTER_MAX_US);
    s_balance_pulse_us = s_start_center_us;
    s_actual_offset_us = 0;
    s_hold_latched = 0U;
    s_have_accepted_position = 0U;
    s_pending_outlier = 0U;
    s_status.state = BALL_STOP_IDLE;
    s_status.running = 0U;
    s_status.completed = 0U;
    s_status.measurement_valid = 0U;
    s_status.control_offset_us = 0;
    s_status.balance_pulse_us = s_balance_pulse_us;
    s_status.elapsed_ms = 0U;
    s_status.completion_ms = 0U;
    AppBallStop_ResetSpeedEstimator();
    AppBallStop_SetServoTarget((uint16_t)s_balance_pulse_us);
}

static void AppBallStop_RunControl(uint32_t now)
{
    int32_t position_error;
    int32_t measured_error;
    int32_t measured_error_abs;
    int32_t predicted_position;
    int32_t error_abs;
    int32_t speed_abs;
    int32_t desired_speed;
    int32_t speed_error;
    int32_t desired_offset;
    int32_t slew_step;
    int32_t pulse_us;
    int32_t prediction_ms;
    uint8_t braking;

    measured_error =
        BALL_STOP_TARGET_0P1MM - (int32_t)s_status.position_0p1mm;
    measured_error_abs = AppBallStop_Abs32(measured_error);
    position_error = measured_error;

    /*
     * The received coordinate describes an earlier camera exposure.  Predict
     * only during motion/capture; HOLD intentionally uses the measured value
     * so that velocity noise cannot shake the servo around -5 cm.
     */
    if (s_hold_latched == 0U) {
        /*
         * New K230 firmware reports exposure-to-send age.  With an older
         * packet (age=0), fall back to the measured frame period.
         */
        prediction_ms = BALL_STOP_SERVO_RESPONSE_MS +
            BALL_STOP_UART_TRANSPORT_MS +
            ((s_reported_frame_age_ms > 0) ?
                s_reported_frame_age_ms : s_frame_period_ms) +
            (int32_t)(now - s_last_measurement_tick) * 10;
        prediction_ms = AppBallStop_Limit32(
            prediction_ms,
            BALL_STOP_PREDICTION_MIN_MS,
            BALL_STOP_PREDICTION_MAX_MS);
        predicted_position = (int32_t)s_status.position_0p1mm +
            (int32_t)s_speed_0p1mm_per_s * prediction_ms / 1000L;
        position_error = BALL_STOP_TARGET_0P1MM - predicted_position;
    }
    error_abs = AppBallStop_Abs32(position_error);
    speed_abs = AppBallStop_Abs32(s_speed_0p1mm_per_s);
    /*
     * HOLD has real hysteresis.  It must remain quiet for 100 ms before
     * latching, and pixel noise inside +/-8 mm cannot immediately release it.
     */
    if (s_hold_latched == 0U) {
        if ((measured_error_abs <= BALL_STOP_HOLD_ENTER_ERROR_0P1MM) &&
            (speed_abs <= BALL_STOP_HOLD_ENTER_SPEED_0P1MM_PER_S)) {
            if ((uint32_t)(now - s_hold_condition_tick) >=
                BALL_STOP_HOLD_ENTER_TICKS_10MS) {
                s_hold_latched = 1U;
                s_status.state = BALL_STOP_HOLDING;
                s_last_trim_tick = now;
            }
        } else {
            s_hold_condition_tick = now;
        }
    } else if (measured_error_abs > BALL_STOP_HOLD_EXIT_ERROR_0P1MM) {
        s_hold_latched = 0U;
        s_hold_condition_tick = now;
        if (s_status.completed == 0U) {
            s_status.state = BALL_STOP_RUNNING;
        }
    }

    if (s_hold_latched != 0U) {
        /*
         * Once captured, stop chasing position samples.  Keep only a small
         * velocity damper around the learned balance pulse.
         */
        desired_speed = 0;
        speed_error = -(int32_t)s_speed_0p1mm_per_s;
        desired_offset = speed_error / BALL_STOP_HOLD_DAMPING_DEN;
        desired_offset = AppBallStop_Limit32(
            desired_offset,
            -BALL_STOP_HOLD_MAX_OFFSET_US,
            BALL_STOP_HOLD_MAX_OFFSET_US);
        slew_step = BALL_STOP_HOLD_STEP_US;
    } else {
        desired_speed = AppBallStop_GetDesiredSpeed(position_error);
        speed_error = desired_speed - (int32_t)s_speed_0p1mm_per_s;
        desired_offset = speed_error / BALL_STOP_SPEED_KP_DEN;

        /*
         * In the last 15 mm, a fast ball gets an explicit command opposing
         * its velocity.  The command still reaches the servo through slew
         * limiting, so braking becomes early without an instantaneous jerk.
         */
        if ((error_abs <= BALL_STOP_DEADZONE_TRIGGER_0P1MM) &&
            (speed_abs > BALL_STOP_CAPTURE_BRAKE_SPEED_0P1MM_PER_S) &&
            ((position_error * (int32_t)s_speed_0p1mm_per_s) > 0)) {
            if ((s_speed_0p1mm_per_s > 0) &&
                (desired_offset > -BALL_STOP_CAPTURE_BRAKE_MIN_US)) {
                desired_offset = -BALL_STOP_CAPTURE_BRAKE_MIN_US;
            } else if ((s_speed_0p1mm_per_s < 0) &&
                       (desired_offset < BALL_STOP_CAPTURE_BRAKE_MIN_US)) {
                desired_offset = BALL_STOP_CAPTURE_BRAKE_MIN_US;
            }
        }

        /* Full dead-zone compensation is used only outside the capture zone. */
        if ((error_abs > BALL_STOP_DEADZONE_TRIGGER_0P1MM) &&
            (speed_error != 0)) {
            if (speed_error > 0) {
                desired_offset += BALL_STOP_DEADZONE_US;
            } else {
                desired_offset -= BALL_STOP_DEADZONE_US;
            }
        }
        desired_offset = AppBallStop_Limit32(
            desired_offset,
            -BALL_STOP_MAX_OFFSET_US,
            BALL_STOP_MAX_OFFSET_US);

        braking =
            (((desired_offset > 0) && (s_speed_0p1mm_per_s < 0)) ||
             ((desired_offset < 0) && (s_speed_0p1mm_per_s > 0))) ? 1U : 0U;
        slew_step = (braking != 0U) ?
            BALL_STOP_BRAKE_STEP_US : BALL_STOP_DRIVE_STEP_US;
    }
    s_actual_offset_us = (int16_t)AppBallStop_MoveToward(
        s_actual_offset_us, desired_offset, slew_step);

    /* Slowly learn the true static balance only very close to -5 cm. */
    if ((s_hold_latched != 0U) &&
        (measured_error_abs >= BALL_STOP_TRIM_ERROR_MIN_0P1MM) &&
        (measured_error_abs <= BALL_STOP_TRIM_ERROR_MAX_0P1MM) &&
        (speed_abs <= BALL_STOP_TRIM_SPEED_MAX_0P1MM_PER_S) &&
        ((uint32_t)(now - s_last_trim_tick) >=
         BALL_STOP_TRIM_INTERVAL_TICKS_10MS)) {
        if (measured_error > 0) {
            s_balance_pulse_us += BALL_STOP_SERVO_DIRECTION;
        } else if (measured_error < 0) {
            s_balance_pulse_us -= BALL_STOP_SERVO_DIRECTION;
        }
        s_balance_pulse_us = (int16_t)AppBallStop_Limit32(
            s_balance_pulse_us,
            s_start_center_us - BALL_STOP_BALANCE_TRIM_US,
            s_start_center_us + BALL_STOP_BALANCE_TRIM_US);
        s_last_trim_tick = now;
    }

    pulse_us = (int32_t)s_balance_pulse_us +
        BALL_STOP_SERVO_DIRECTION * (int32_t)s_actual_offset_us;
    pulse_us = AppBallStop_Limit32(
        pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    AppBallStop_SetServoImmediate((uint16_t)pulse_us);

    s_status.speed_0p1mm_per_s = s_speed_0p1mm_per_s;
    s_status.control_offset_us = s_actual_offset_us;
    s_status.balance_pulse_us = s_balance_pulse_us;

    if (s_status.completed == 0U) {
        if ((s_hold_latched != 0U) &&
            (measured_error_abs <= BALL_STOP_FINAL_ERROR_0P1MM) &&
            (speed_abs <= BALL_STOP_FINAL_SPEED_0P1MM_PER_S)) {
            if ((uint32_t)(now - s_complete_condition_tick) >=
                BALL_STOP_FINAL_TICKS_10MS) {
                s_status.completed = 1U;
                s_status.state = BALL_STOP_COMPLETE;
                s_status.completion_ms =
                    (uint32_t)(now - s_start_tick) * 10U;
                s_status.elapsed_ms = s_status.completion_ms;
            }
        } else {
            s_complete_condition_tick = now;
        }
    } else {
        /* Completion freezes the timer, not the -5 cm feedback controller. */
        s_status.state = BALL_STOP_COMPLETE;
    }
}

void AppBallStop_Init(void)
{
    s_start_request = 0U;
    s_stop_request = 0U;
    s_status.position_0p1mm = 0;
    s_status.speed_0p1mm_per_s = 0;
    s_status.servo_pulse_us = SERVO_CENTER_PULSE_US;
    AppBallStop_StopInternal();
}

void AppBallStop_RequestStart(void)
{
    s_start_request = 1U;
}

void AppBallStop_RequestStop(void)
{
    s_stop_request = 1U;
}

void AppBallStop_BackgroundTask(const K230_Status *k230)
{
    uint32_t now;
    uint8_t measurement_valid;
    uint8_t new_frame;

    if (k230 == 0) {
        return;
    }

    now = K230_GetTick10ms();
    if (s_stop_request != 0U) {
        s_stop_request = 0U;
        s_start_request = 0U;
        AppBallStop_StopInternal();
    } else if (s_start_request != 0U) {
        s_start_request = 0U;
        AppBallStop_StartInternal(now);
    }

    measurement_valid =
        ((k230->ball_online != 0U) &&
         (k230->ball_valid != 0U) &&
         (k230->ball_quality >= BALL_STOP_MIN_QUALITY)) ? 1U : 0U;
    s_status.measurement_valid = measurement_valid;

    if (s_status.state == BALL_STOP_IDLE) {
        return;
    }

    if (s_status.completed == 0U) {
        s_status.elapsed_ms = (uint32_t)(now - s_start_tick) * 10U;
    }

    if (measurement_valid == 0U) {
        if ((uint32_t)(now - s_invalid_start_tick) >=
            BALL_STOP_INVALID_FAULT_TICKS_10MS) {
            int32_t safe_pulse;

            s_status.state = BALL_STOP_VISION_FAULT;
            s_status.running = 0U;
            s_actual_offset_us = (int16_t)AppBallStop_MoveToward(
                s_actual_offset_us, 0, BALL_STOP_BRAKE_STEP_US);
            safe_pulse = (int32_t)s_balance_pulse_us +
                BALL_STOP_SERVO_DIRECTION * (int32_t)s_actual_offset_us;
            safe_pulse = AppBallStop_Limit32(
                safe_pulse, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
            AppBallStop_SetServoImmediate((uint16_t)safe_pulse);
            AppBallStop_ResetSpeedEstimator();
        }
        return;
    }

    s_invalid_start_tick = now;
    new_frame =
        (k230->ball_frame_count != s_last_ball_frame_count) ? 1U : 0U;
    if (new_frame != 0U) {
        s_last_ball_frame_count = k230->ball_frame_count;
        if (AppBallStop_AcceptPosition(
                k230->ball_position_0p1mm,
                k230->ball_last_rx_tick_10ms) != 0U) {
            uint32_t frame_ticks = (uint32_t)(
                k230->ball_last_rx_tick_10ms - s_last_measurement_tick);
            int32_t frame_ms = (int32_t)frame_ticks * 10;

            if ((s_history_count != 0U) &&
                (frame_ms >= BALL_STOP_FRAME_PERIOD_MIN_MS) &&
                (frame_ms <= BALL_STOP_FRAME_PERIOD_MAX_MS)) {
                /* 25 percent new interval, 75 percent prior interval. */
                s_frame_period_ms =
                    (s_frame_period_ms * 3 + frame_ms) / 4;
            }
            s_status.position_0p1mm = k230->ball_position_0p1mm;
            s_reported_frame_age_ms = (int32_t)k230->ball_frame_age_ms;
            s_last_measurement_tick = k230->ball_last_rx_tick_10ms;
            AppBallStop_UpdateSpeed(
                k230->ball_position_0p1mm,
                k230->ball_last_rx_tick_10ms);
            if (s_status.state == BALL_STOP_VISION_FAULT) {
                s_status.state = BALL_STOP_RUNNING;
                s_status.running = 1U;
                s_complete_condition_tick = now;
            }
        }
    }

    /* Run the actuator at a deterministic 10 ms cadence. */
    if ((uint32_t)(now - s_last_control_tick) >= 1U) {
        s_last_control_tick = now;
        AppBallStop_RunControl(now);
    }
}

void AppBallStop_GetStatus(AppBallStop_Status *status)
{
    if (status != 0) {
        *status = s_status;
    }
}
