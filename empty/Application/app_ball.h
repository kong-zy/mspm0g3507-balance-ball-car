#ifndef APPLICATION_APP_BALL_H_
#define APPLICATION_APP_BALL_H_

#include "Hardware/K230.h"
#include <stdint.h>

/*
 * H题要求3的运行状态。
 *
 * 状态转换：
 *   IDLE
 *     -> WAIT_CENTER      按题意从中心O开始，快速确认视觉位置
 *     -> MOVE_POSITIVE    控制钢球运行到+5cm
 *     -> MOVE_NEGATIVE    到达+5cm后折返到-5cm
 *     -> HOLD_NEGATIVE    钢球进入-5cm允许误差带并连续稳定
 *     -> COMPLETE         已完成要求3，继续闭环保持在-5cm
 *
 * LEVEL_PIPE保留为调试/标定状态，不放入正式要求三的5秒计时流程。
 * COMM_FAULT、BALL_LOST、TIMEOUT均会把摆杆恢复到中位，防止失控。
 */
typedef enum
{
    BALL_TASK_IDLE = 0,
    BALL_TASK_LEVEL_PIPE,
    BALL_TASK_WAIT_CENTER,
    BALL_TASK_MOVE_POSITIVE,
    BALL_TASK_MOVE_NEGATIVE,
    BALL_TASK_HOLD_NEGATIVE,
    BALL_TASK_COMPLETE,
    BALL_TASK_COMM_FAULT,
    BALL_TASK_BALL_LOST,
    BALL_TASK_TIMEOUT
} AppBall_State;

typedef struct
{
    AppBall_State state;
    uint8_t running;
    uint8_t completed;
    uint8_t communication_ok;
    uint8_t measurement_valid;
    int16_t position_0p1mm;
    int16_t target_0p1mm;
    int16_t reference_0p1mm;
    int16_t speed_0p1mm_per_s;
    int16_t control_offset_us;
    int16_t level_trim_us;
    uint16_t servo_target_us;
    uint32_t elapsed_ms;
    uint32_t completion_ms;
} AppBall_Status;

/* Runtime parameters exposed by the OLED BALL SET page. */
typedef enum
{
    APP_BALL_TUNE_CENTER_US = 0,
    APP_BALL_TUNE_MOVE_US,
    APP_BALL_TUNE_BRAKE_US,
    APP_BALL_TUNE_CAPTURE_US,
    APP_BALL_TUNE_HOLD_US,
    APP_BALL_TUNE_SPEED_BRAKE_US,
    APP_BALL_TUNE_POS_SPEED,
    APP_BALL_TUNE_NEG_SPEED,
    APP_BALL_TUNE_PREDICT_MS,
    APP_BALL_TUNE_BACK,
    APP_BALL_TUNE_COUNT
} AppBall_TuneParameter;

/* 初始化状态机并把摆杆控制舵机恢复到中位。 */
void AppBall_Init(void);

/*
 * 按键中断只提交请求，不直接执行PID或修改舵机脉宽。
 * 请求由主循环中的 AppBall_BackgroundTask() 安全处理。
 */
void AppBall_RequestStart(void);
void AppBall_RequestStop(void);

/*
 * 主循环持续调用：
 * 1. 读取最新钢球位置；
 * 2. 推进 O -> +5cm -> -5cm 状态机；
 * 3. 计算摆杆舵机目标脉宽；
 * 4. 执行通信失联、丢球和5秒超时保护。
 */
void AppBall_BackgroundTask(const K230_Status *k230);

/* 给OLED读取一份要求3状态快照。 */
void AppBall_GetStatus(AppBall_Status *status);

/* The values remain active until reset or power-off; no flash write is used. */
int16_t AppBall_GetTuneParameter(AppBall_TuneParameter parameter);
void AppBall_AdjustTuneParameter(
    AppBall_TuneParameter parameter,
    int8_t direction);

#endif /* APPLICATION_APP_BALL_H_ */
