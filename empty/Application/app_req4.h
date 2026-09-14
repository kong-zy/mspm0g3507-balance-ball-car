#ifndef APPLICATION_APP_REQ4_H_
#define APPLICATION_APP_REQ4_H_

#include "Hardware/K230.h"
#include <stdint.h>

/*
 * H题要求四独立状态机。
 *
 * 电机循迹与钢球视觉/舵机控制相互独立：按键后电机立即开始运行，不等待
 * K230或钢球位于中心；视觉无效、钢球越界以及8秒超时只记录状态，不会停止
 * 电机。小车持续循迹，直到用户主动停止或退出要求四页面。
 */
typedef enum
{
    REQ4_STATE_IDLE = 0,
    REQ4_STATE_WAIT_CENTER,
    REQ4_STATE_RUNNING,
    REQ4_STATE_COMPLETE,
    REQ4_STATE_VISION_FAULT,
    REQ4_STATE_BALL_LIMIT,
    REQ4_STATE_LINE_LOST,
    REQ4_STATE_TIMEOUT
} AppReq4_State;

/*
 * 现有凸轮机构存在明显机械死区，因此要求四采用动态平衡值，并在偏差较大时
 * 执行：观察 -> 有效短脉冲 -> 等待机械与视觉反馈。
 */
typedef enum
{
    REQ4_CORRECTION_OBSERVE = 0,
    REQ4_CORRECTION_PULSE,
    REQ4_CORRECTION_SETTLE
} AppReq4_CorrectionState;

/* Requirement-4 cascade parameters exposed on the OLED tuning page. */
typedef enum
{
    APP_REQ4_TUNE_POSITION_KP_X10 = 0,
    APP_REQ4_TUNE_SPEED_KP_X100,
    APP_REQ4_TUNE_MAX_OFFSET_US,
    APP_REQ4_TUNE_DRIVE_STEP_US,
    APP_REQ4_TUNE_BRAKE_STEP_US,
    APP_REQ4_TUNE_BACK,
    APP_REQ4_TUNE_COUNT
} AppReq4_TuneParameter;

typedef struct
{
    AppReq4_State state;
    AppReq4_CorrectionState correction_state;
    uint8_t running;
    uint8_t completed;
    uint8_t communication_ok;
    uint8_t measurement_valid;
    uint8_t vision_fault;
    uint8_t ball_limit_exceeded;
    uint8_t line_lost;
    uint8_t timed_out;
    int16_t position_0p1mm;
    int16_t speed_0p1mm_per_s;
    int16_t max_abs_position_0p1mm;
    int16_t servo_offset_us;
    uint16_t balance_pulse_us;
    uint16_t servo_target_us;
    uint32_t elapsed_ms;
    uint32_t completion_ms;
    uint32_t distance_count;
} AppReq4_Status;

void AppReq4_Init(void);
void AppReq4_RequestStart(void);
void AppReq4_RequestStop(void);

/* 主循环调用：处理K230新帧、中心越界保护和舵机短脉冲。 */
void AppReq4_BackgroundTask(const K230_Status *k230);

/* 10ms编码器任务：累计AB距离和计时；通过B点后只锁存成绩，不停止电机。 */
void AppReq4_UpdateDistance10ms(void);

/* 3ms循迹任务：使用要求四专用的低冲击速度斜坡和转向限幅。 */
void AppReq4_UpdateTarget3ms(void);

void AppReq4_GetStatus(AppReq4_Status *status);
int32_t AppReq4_GetTuneParameter(AppReq4_TuneParameter parameter);
void AppReq4_AdjustTuneParameter(
    AppReq4_TuneParameter parameter, int8_t direction);

#endif /* APPLICATION_APP_REQ4_H_ */
