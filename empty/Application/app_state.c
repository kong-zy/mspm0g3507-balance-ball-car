#include "Application/app_state.h"

/*
 * 这里只定义共享变量，不执行控制逻辑。初始值与拆分前逐项一致，
 * 确保上电页面、舵机中位、循迹 PD 参数及停车状态保持不变。
 */
volatile int32_t encoderA_cnt = 0;
volatile int32_t encoderB_cnt = 0;
volatile int32_t PWMA = 0;
volatile int32_t PWMB = 0;
volatile int32_t speedA_rpm = 0;
volatile int32_t speedB_rpm = 0;

/*
 * g_lap_distance_x2 保存左右轮绝对编码器增量之和，相当于车体平均里程的
 * 2 倍。采用这种表示可以避免每 10 毫秒都进行除以 2 后损失半个计数。
 * g_run_time_ms 记录本次运行时间，单位为毫秒。
 */
volatile uint32_t g_lap_distance_x2 = 0U;
volatile uint32_t g_run_time_ms = 0U;
volatile uint8_t g_follow_state = FOLLOW_STATE_IDLE;
volatile uint8_t g_track_mask = 0U;
volatile int16_t g_track_error = 0;
volatile uint8_t g_line_found = 0U;
volatile uint8_t g_follow_restart = 1U;
volatile int32_t g_track_targetA = TRACK_START_SPEED;
volatile int32_t g_track_targetB = TRACK_START_SPEED;

/*
 * 循迹 PD 的运行时可调参数。初始 Kp=1.0、Kd=1.5，与修改前的固定参数完全
 * 一致；在 PID 页面修改后，3ms 循迹中断会立即使用新值。断电或复位后参数
 * 恢复为这里的初始值，实车调好后可把最终值写回这两个初始化常量。
 */
volatile int16_t g_track_kp_x10 = 10;
volatile int16_t g_track_kd_x10 = 15;
volatile uint8_t g_pid_select = PID_SELECT_KP;

/*
 * OLED 使用两级刷新标志：
 * g_oled_page_redraw = 1：页面发生切换，需要清屏并重新绘制固定文字；
 * g_oled_update      = 1：页面没有切换，只更新光标、角度或运行状态。
 * 普通按键只置位 g_oled_update，避免每次点击都写满整个 OLED。
 */
volatile uint8_t g_oled_update = 1U;
volatile uint8_t g_oled_page_redraw = 1U;
volatile uint8_t g_motor_test_mode = 1U;
volatile uint8_t g_menu_level = MENU_LEVEL_TOP;
volatile uint8_t g_menu_page = MENU_PAGE_CAR;

volatile uint8_t g_servo_select = SERVO_SELECT_1;
volatile uint8_t g_servo_angle1 = SERVO_CENTER_ANGLE_DEG;
volatile uint8_t g_servo_angle2 = SERVO_CENTER_ANGLE_DEG;
volatile uint8_t g_ball_tune_select = 0U;
volatile uint8_t g_req4_tune_select = 0U;

/*
 * K230通信状态快照。
 * 既保留原来的@K230/ACK压力测试计数，也保存要求3正式BALL帧中的
 * 钢球位置、识别质量、帧序号和在线状态。
 */
K230_Status g_k230_test_status;

int Flag_Stop = 1;
