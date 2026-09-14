#ifndef APPLICATION_APP_BALL_STOP_H_
#define APPLICATION_APP_BALL_STOP_H_

#include "Hardware/K230.h"
#include <stdint.h>

typedef enum
{
    BALL_STOP_IDLE = 0,
    BALL_STOP_RUNNING,
    BALL_STOP_HOLDING,
    BALL_STOP_COMPLETE,
    BALL_STOP_VISION_FAULT
} AppBallStop_State;

typedef struct
{
    AppBallStop_State state;
    uint8_t running;
    uint8_t completed;
    uint8_t measurement_valid;
    int16_t position_0p1mm;
    int16_t speed_0p1mm_per_s;
    int16_t control_offset_us;
    int16_t balance_pulse_us;
    uint16_t servo_pulse_us;
    uint32_t elapsed_ms;
    uint32_t completion_ms;
} AppBallStop_Status;

void AppBallStop_Init(void);
void AppBallStop_RequestStart(void);
void AppBallStop_RequestStop(void);
void AppBallStop_BackgroundTask(const K230_Status *k230);
void AppBallStop_GetStatus(AppBallStop_Status *status);

#endif /* APPLICATION_APP_BALL_STOP_H_ */
