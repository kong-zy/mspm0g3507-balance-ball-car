#ifndef APPLICATION_APP_STATE_H_
#define APPLICATION_APP_STATE_H_

#include "Hardware/board.h"

/*
 * 应用层共享配置与运行状态。
 *
 * Hardware 目录只负责具体外设读写；按键、OLED 和循迹模块通过本文件共享
 * 页面、车辆状态、目标速度和累计里程。所有参数均从原 app.c 原样迁移，
 * 因而拆分不会改变 PA18 按键、电机方向、PID 或终点判断阈值。
 */
#define CONTROL_PERIOD_MS       (10U)
#define OLED_REFRESH_PERIOD_MS  (50U)
#define KEY_DEBOUNCE_TICKS      (3U)
#define KEY_LONG_PRESS_TICKS    (80U)
#define KEY_ACTION_NONE         (0U)
#define KEY_ACTION_SHORT        (1U)
#define KEY_ACTION_LONG         (2U)
#define KEY_ACTION_DOUBLE       (3U)
#define KEY_DOUBLE_WINDOW_TICKS (25U)

#define MENU_LEVEL_TOP          (0U)
#define MENU_LEVEL_DETAIL       (1U)
#define MENU_PAGE_CAR           (0U)
#define MENU_PAGE_TRACK         (1U)
#define MENU_PAGE_PID           (2U)
#define MENU_PAGE_FOLLOW        (3U)
#define MENU_PAGE_K230          (4U)
#define MENU_PAGE_BALL_TUNE     (5U)
#define MENU_PAGE_REQ4          (6U)
#define MENU_PAGE_REQ4_TUNE     (7U)
#define MENU_PAGE_BALL_STOP     (8U)
#define MENU_PAGE_LAST          MENU_PAGE_BALL_STOP
/* Kept only for unreachable legacy detail handlers; no top-menu entry exists. */
#define MENU_PAGE_SERVO         (0xFEU)
#define MENU_PAGE_ANGLE         (0xFDU)
#define SERVO_SELECT_1          (0U)
#define SERVO_SELECT_2          (1U)
#define SERVO_ANGLE_STEP_DEG    (15U)

/*
 * PID 页面实际调节的是红外循迹外环的 PD 参数：
 *
 * Kp：黑线偏离中心后，小车向黑线方向修正的力度；
 * Kd：根据误差变化速度抑制左右来回摆动的力度；
 * Ki：本项目暂时固定为 0。数字循迹传感器的误差是离散值，积分项容易在弯道
 *     中累积，出弯后仍持续反向修正，因此当前阶段不开放 Ki。
 *
 * 参数使用放大 10 倍的整数保存，例如数值 15 在 OLED 上显示为 1.5。这样在
 * 3ms 循迹中断中不需要执行浮点运算。Kp/Kd 每次按键改变 0.1，并限制在安全
 * 范围内，防止误操作产生特别大的左右轮差速。
 */
#define PID_SELECT_KP                       (0U)
#define PID_SELECT_KD                       (1U)
#define PID_SELECT_BACK                     (2U)
#define TRACK_PD_SCALE                      (10)
#define TRACK_PD_KP_MIN_X10                 (0)
#define TRACK_PD_KP_MAX_X10                 (40)
#define TRACK_PD_KD_MIN_X10                 (0)
#define TRACK_PD_KD_MAX_X10                 (60)
#define TRACK_PD_STEP_X10                   (1)

/*
 * H 题基础要求：从 A 点顺时针循迹一圈，回到 A 点停车。
 * 赛道由两段 1.5 米直线和两个半径 0.5 米的半圆组成，中心线总长度约为
 * 6.1416 米。现有车轮直径为 65 毫米，编码器每个车轮转一圈产生约 780 个
 * 有效计数，由此换算得到每行驶 1 米约产生 3820 个平均编码器计数。
 *
 * 编码器里程不直接作为“一圈完成”的唯一依据，而只用于防止起点误判：
 * 小车累计行驶达到 5 米后，先降低循迹速度，再允许识别终点。最终是否到达
 * A 点，仍由 8 路红外传感器在一个短距离窗口内共同检测启停横线来确认。
 *
 * 小车从圆弧进入 A 点附近时，车身可能没有完全摆正，启停横线会从阵列一侧
 * 斜着扫到另一侧。程序因此不要求“某一帧同时出现很多路黑色”，而是把短距离
 * 内先后检测到黑色的探头合并起来判断，避免因各探头触发不同步而漏判终点。
 */
#define TRACK_TIMER_PERIOD_MS              (3U)
#define ENCODER_COUNTS_PER_METER           (3820U)
#define FINISH_ARM_DISTANCE_COUNTS         (ENCODER_COUNTS_PER_METER * 5U)
/*
 * 超时保护距离设为 6.25 米。
 * 使用 25/4 的整数比例计算，既能准确表示 6.25，又不会在定时中断中引入浮点运算。
 * 以当前每米 3820 个计数计算，触发阈值为 23875 个平均编码器计数。
 */
#define FINISH_FAILSAFE_DISTANCE_COUNTS    \
    ((ENCODER_COUNTS_PER_METER * 25U) / 4U)
/*
 * 终点横线多帧累计识别参数：
 *
 * 1. FINISH_WINDOW_ENCODER_COUNTS 为一次横线事件允许使用的最大距离窗口。
 *    220 个平均编码器计数约等于 57.6 毫米，略大于 A 点 5 厘米标志线长度，
 *    足以容纳小车斜着通过时左右探头之间的触发距离差。
 *
 * 2. FINISH_ACCUMULATED_MIN_BLACK 表示在该窗口内至少要有 4 个不同探头检测
 *    到过黑色。这里统计的是“不同探头的累计数量”，不要求 4 路同时为黑。
 *
 * 3. 累计窗口仍由任意一个外侧探头命中后启动，但最终确认不再要求左、右
 *    两侧分别命中；只要窗口内累计4个不同探头见黑，就确认识别到A点横线。
 */
#define FINISH_WINDOW_ENCODER_COUNTS       (220U)
#define FINISH_ACCUMULATED_MIN_BLACK       (4U)
#define FINISH_WINDOW_START_MASK            \
    (TRACK_MASK_L4 | TRACK_MASK_L3 | TRACK_MASK_L2 | \
     TRACK_MASK_R2 | TRACK_MASK_R3 | TRACK_MASK_R4)
/*
 * 本版按照实车调试要求采用“识别即停车”：累计探头数量和左右覆盖条件一旦
 * 同时成立，立即清零左右轮目标速度、PWM 输出和速度闭环历史，不再继续执行
 * 原先的 590 个编码器计数停车位置补偿。
 */
#define START_LEAVE_MIN_COUNTS             (100U)
#define START_LINE_MIN_BLACK               (5U)
#define START_LINE_CONFIRM_TICKS           (6U)
#define LINE_LOST_STOP_TICKS               (67U)

/*
 * 下列速度的单位是“每 10 毫秒内的编码器计数”，不是每分钟转速。
 * 例如目标值 14 表示每 10 毫秒希望车轮产生 14 个编码器计数，换算后的
 * 线速度约为 0.37 米/秒，可为赛题 20 秒限时保留一定余量。
 *
 * 起步速度较低，随后通过速度斜坡逐渐提高；误差越大，基础速度越低。
 * 这样既能减少起步冲击，也能避免小车高速进入半圆弯道时冲出黑线。
 */
#define MOTOR_TEST_SPEED                   (5)
#define TRACK_START_SPEED                  (5)
#define TRACK_STRAIGHT_SPEED               (16)
#define TRACK_CURVE_SPEED                  (14)
#define TRACK_SHARP_SPEED                  (12)
#define TRACK_RECOVERY_SPEED               (10)
#define TRACK_LOST_SPEED                   (8)
#define TRACK_FINISH_SEARCH_SPEED          (8)
#define TRACK_TARGET_MAX                   (22)
#define TRACK_RAMP_TICKS                   (34U)
#define PWM_LIMIT                          (7999)

/*
 * 一圈循迹状态机，OLED 页面中的 Q 数字就是这里的状态编号：
 * 0：空闲，电机停止，等待按键启动。
 * 1：正在离开 A 点起始横线，此时禁止判断终点。
 * 2：正常循迹，但累计里程尚未达到终点开放条件。
 * 3：累计里程已超过 5 米，低速循迹并用多帧累计方式搜索 A 点终点横线。
 * 4：已经确认 A 点终点横线，并在识别成功的这一帧立即停车。
 * 6：连续丢线超过约 0.2 秒，低速向右转动寻找黑线；找到后恢复循迹。
 * 7：行驶达到约 6.25 米仍未检测到终点横线，判定识别异常并停车。
 */
#define FOLLOW_STATE_IDLE                  (0U)
#define FOLLOW_STATE_LEAVE_START           (1U)
#define FOLLOW_STATE_RUNNING               (2U)
#define FOLLOW_STATE_FINISH_ARMED          (3U)
#define FOLLOW_STATE_STOPPED               (4U)
#define FOLLOW_STATE_LINE_LOST             (6U)
#define FOLLOW_STATE_FINISH_MISSED         (7U)

extern volatile int32_t encoderA_cnt;
extern volatile int32_t encoderB_cnt;
extern volatile int32_t PWMA;
extern volatile int32_t PWMB;
extern volatile int32_t speedA_rpm;
extern volatile int32_t speedB_rpm;
extern volatile uint32_t g_lap_distance_x2;
extern volatile uint32_t g_run_time_ms;
extern volatile uint8_t g_follow_state;
extern volatile uint8_t g_track_mask;
extern volatile int16_t g_track_error;
extern volatile uint8_t g_line_found;
extern volatile uint8_t g_follow_restart;
extern volatile int32_t g_track_targetA;
extern volatile int32_t g_track_targetB;
extern volatile int16_t g_track_kp_x10;
extern volatile int16_t g_track_kd_x10;
extern volatile uint8_t g_pid_select;
extern volatile uint8_t g_oled_update;
extern volatile uint8_t g_oled_page_redraw;
extern volatile uint8_t g_motor_test_mode;
extern volatile uint8_t g_menu_level;
extern volatile uint8_t g_menu_page;
extern volatile uint8_t g_servo_select;
extern volatile uint8_t g_servo_angle1;
extern volatile uint8_t g_servo_angle2;
extern volatile uint8_t g_ball_tune_select;
extern volatile uint8_t g_req4_tune_select;
extern K230_Status g_k230_test_status;
extern int Flag_Stop;

#endif /* APPLICATION_APP_STATE_H_ */
