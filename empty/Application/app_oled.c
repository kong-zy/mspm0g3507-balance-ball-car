#include "Hardware/board.h"
#include "Application/app_state.h"
#include "Application/app_oled.h"
#include "Application/app_ball.h"
#include "Application/app_ball_stop.h"
#include "Application/app_req4.h"

/*
 * 本模块集中管理所有 OLED 页面及数字格式化。
 * 显示层只读取共享状态，不参与电机闭环和循迹状态机。
 */
static void OLED_ShowFixedSigned(uint8_t x, uint8_t page, int32_t num)
{
    char text[8];
    uint8_t index = 0U;
    uint8_t i;
    uint32_t value;
    uint32_t div = 1U;

    /* 固定 7 个字符宽度覆盖显示，避免先清屏造成数字闪烁。 */
    for (i = 0U; i < sizeof(text); i++) {
        text[i] = ' ';
    }
    text[sizeof(text) - 1U] = '\0';

    if (num < 0) {
        text[index++] = '-';
        value = (uint32_t)(-num);
    } else {
        value = (uint32_t)num;
    }

    if (value == 0U) {
        text[index++] = '0';
    } else {
        while ((value / div) >= 10U) {
            div *= 10U;
        }
        while ((div > 0U) && (index < (sizeof(text) - 1U))) {
            text[index++] = (char)('0' + (value / div));
            value %= div;
            div /= 10U;
        }
    }

    OLED_ShowString(x, page, text);
}

static void OLED_ShowFixedUnsigned(uint8_t x, uint8_t page, uint32_t num)
{
    char text[4];

    text[0] = (char)('0' + (num % 10U));
    text[1] = ' ';
    text[2] = ' ';
    text[3] = '\0';
    OLED_ShowString(x, page, text);
}

static void OLED_ShowFixed3Unsigned(uint8_t x, uint8_t page, uint32_t num)
{
    char text[4];

    if (num > 999U) {
        num = 999U;
    }

    text[0] = (char)('0' + ((num / 100U) % 10U));
    text[1] = (char)('0' + ((num / 10U) % 10U));
    text[2] = (char)('0' + (num % 10U));
    text[3] = '\0';
    OLED_ShowString(x, page, text);
}

static void OLED_ShowGain10(uint8_t x, uint8_t page, int16_t gain_x10)
{
    char text[5];
    uint16_t value;
    uint16_t integer_part;

    if (gain_x10 < 0) {
        value = 0U;
    } else {
        value = (uint16_t)gain_x10;
    }

    integer_part = value / 10U;
    text[0] = (integer_part >= 10U) ?
        (char)('0' + ((integer_part / 10U) % 10U)) : ' ';
    text[1] = (char)('0' + (integer_part % 10U));
    text[2] = '.';
    text[3] = (char)('0' + (value % 10U));
    text[4] = '\0';
    OLED_ShowString(x, page, text);
}

static void OLED_ShowTime100ms(uint8_t x, uint8_t page, uint32_t time_ms)
{
    char text[6];
    uint32_t deciseconds = time_ms / 100U;

    if (deciseconds > 9999U) {
        deciseconds = 9999U;
    }

    text[0] = (deciseconds >= 1000U) ?
        (char)('0' + ((deciseconds / 1000U) % 10U)) : ' ';
    text[1] = (deciseconds >= 100U) ?
        (char)('0' + ((deciseconds / 100U) % 10U)) : ' ';
    text[2] = (char)('0' + ((deciseconds / 10U) % 10U));
    text[3] = '.';
    text[4] = (char)('0' + (deciseconds % 10U));
    text[5] = '\0';
    OLED_ShowString(x, page, text);
}

/* 把0.1mm单位的位置四舍五入成带符号的整数毫米，例如-500显示-050。 */
static void OLED_ShowBallPositionMm(
    uint8_t x, uint8_t page, int16_t position_0p1mm)
{
    char text[5];
    uint16_t magnitude;
    uint16_t millimeter;

    text[0] = (position_0p1mm < 0) ? '-' : '+';
    magnitude = (uint16_t)((position_0p1mm < 0) ?
        -position_0p1mm : position_0p1mm);
    millimeter = (uint16_t)((magnitude + 5U) / 10U);
    if (millimeter > 999U) {
        millimeter = 999U;
    }

    text[1] = (char)('0' + ((millimeter / 100U) % 10U));
    text[2] = (char)('0' + ((millimeter / 10U) % 10U));
    text[3] = (char)('0' + (millimeter % 10U));
    text[4] = '\0';
    OLED_ShowString(x, page, text);
}

static void OLED_ShowFixedAngle100(uint8_t x, uint8_t page, int16_t angle_cdeg)
{
    char text[8];
    uint16_t magnitude;
    uint16_t degree;
    uint16_t decimal;

    /*
     * 固定显示成“+180.00”或“- 12.34”一类的 7 字符格式。
     * 每次覆盖相同宽度，角度变化时不需要清屏，因此不会出现数字闪烁。
     */
    if (angle_cdeg > 18000) {
        angle_cdeg = 18000;
    } else if (angle_cdeg < -18000) {
        angle_cdeg = -18000;
    }

    text[0] = (angle_cdeg < 0) ? '-' : '+';
    magnitude = (uint16_t)((angle_cdeg < 0) ? -angle_cdeg : angle_cdeg);
    degree = magnitude / 100U;
    decimal = magnitude % 100U;

    text[1] = (degree >= 100U) ? (char)('0' + (degree / 100U)) : ' ';
    text[2] = (degree >= 10U) ? (char)('0' + ((degree / 10U) % 10U)) : ' ';
    text[3] = (char)('0' + (degree % 10U));
    text[4] = '.';
    text[5] = (char)('0' + (decimal / 10U));
    text[6] = (char)('0' + (decimal % 10U));
    text[7] = '\0';

    OLED_ShowString(x, page, text);
}

static void OLED_ShowRunState(uint8_t x, uint8_t page)
{
    if (Flag_Stop != 0) {
        OLED_ShowString(x, page, "STOP");
    } else {
        OLED_ShowString(x, page, "RUN ");
    }
}

static void OLED_ShowTopMenuFrame(void)
{
    /*
     * OLED 一次只能显示四项。一级菜单现在共有 CAR、SERVO、ANGLE、TRACK、
     * PID、FOLLOW、BALL3 七项；光标移动到后面的选项时，
     * OLED_UpdateTopMenu() 自动切换菜单窗口。
     * 本函数保留为一级菜单固定框架入口，目前没有额外固定文字。
     */
}

static __attribute__((unused)) void OLED_UpdateTopMenuLegacy(void)
{
    /*
     * 每行固定覆盖 7 个字符，菜单滚动时不会留下上一次较长选项的残余字符。
     * 前四项显示原来的测试程序；光标移到后面的选项时窗口向下滚动，
     * 同时显示在线调参、实际循迹和H题要求3滚球控制入口。
     */
    if (g_menu_page >= MENU_PAGE_REQ4) {
        OLED_ShowString(0, 0, " BALL3 ");
        OLED_ShowString(0, 2, " BSET  ");
        OLED_ShowString(0, 4,
            (g_menu_page == MENU_PAGE_REQ4) ? ">REQ4  " : " REQ4  ");
        OLED_ShowString(0, 6,
            (g_menu_page == MENU_PAGE_BALL_STOP) ? ">BSTOP " : " BSTOP ");
    } else if (g_menu_page >= MENU_PAGE_BALL_TUNE) {
        OLED_ShowString(0, 0, " FOLLOW");
        OLED_ShowString(0, 2, " BALL3 ");
        OLED_ShowString(0, 4,
            (g_menu_page == MENU_PAGE_BALL_TUNE) ? ">BSET  " : " BSET  ");
        OLED_ShowString(0, 6,
            (g_menu_page == MENU_PAGE_REQ4) ? ">REQ4  " : " REQ4  ");
    } else if (g_menu_page >= MENU_PAGE_K230) {
        OLED_ShowString(0, 0, " PID   ");
        OLED_ShowString(0, 2, " FOLLOW");
        OLED_ShowString(0, 4,
            (g_menu_page == MENU_PAGE_K230) ? ">BALL3 " : " BALL3 ");
        OLED_ShowString(0, 6,
            (g_menu_page == MENU_PAGE_BALL_TUNE) ? ">BSET  " : " BSET  ");
    } else if (g_menu_page >= MENU_PAGE_FOLLOW) {
        OLED_ShowString(0, 0, " TRACK ");
        OLED_ShowString(0, 2, " PID   ");
        OLED_ShowString(0, 4,
            (g_menu_page == MENU_PAGE_FOLLOW) ? ">FOLLOW" : " FOLLOW");
        OLED_ShowString(0, 6,
            (g_menu_page == MENU_PAGE_K230) ? ">BALL3 " : " BALL3 ");
    } else if (g_menu_page >= MENU_PAGE_PID) {
        OLED_ShowString(0, 0, " ANGLE ");
        OLED_ShowString(0, 2, " TRACK ");
        OLED_ShowString(0, 4,
            (g_menu_page == MENU_PAGE_PID) ? ">PID   " : " PID   ");
        OLED_ShowString(0, 6, " FOLLOW");
    } else {
        OLED_ShowString(0, 0, (g_menu_page == MENU_PAGE_CAR) ? ">CAR   " : " CAR   ");
        OLED_ShowString(0, 2, (g_menu_page == MENU_PAGE_SERVO) ? ">SERVO " : " SERVO ");
        OLED_ShowString(0, 4, (g_menu_page == MENU_PAGE_ANGLE) ? ">ANGLE " : " ANGLE ");
        OLED_ShowString(0, 6, (g_menu_page == MENU_PAGE_TRACK) ? ">TRACK " : " TRACK ");
    }
}

static void OLED_UpdateTopMenu(void)
{
    if (g_menu_page <= MENU_PAGE_FOLLOW) {
        OLED_ShowString(0, 0,
            (g_menu_page == MENU_PAGE_CAR) ? ">CAR   " : " CAR   ");
        OLED_ShowString(0, 2,
            (g_menu_page == MENU_PAGE_TRACK) ? ">TRACK " : " TRACK ");
        OLED_ShowString(0, 4,
            (g_menu_page == MENU_PAGE_PID) ? ">PID   " : " PID   ");
        OLED_ShowString(0, 6,
            (g_menu_page == MENU_PAGE_FOLLOW) ? ">FOLLOW" : " FOLLOW");
    } else if (g_menu_page <= MENU_PAGE_REQ4_TUNE) {
        OLED_ShowString(0, 0,
            (g_menu_page == MENU_PAGE_K230) ? ">BALL3 " : " BALL3 ");
        OLED_ShowString(0, 2,
            (g_menu_page == MENU_PAGE_BALL_TUNE) ? ">BSET  " : " BSET  ");
        OLED_ShowString(0, 4,
            (g_menu_page == MENU_PAGE_REQ4) ? ">REQ4  " : " REQ4  ");
        OLED_ShowString(0, 6,
            (g_menu_page == MENU_PAGE_REQ4_TUNE) ? ">R4PID " : " R4PID ");
    } else {
        OLED_ShowString(0, 0, " BSET  ");
        OLED_ShowString(0, 2, " REQ4  ");
        OLED_ShowString(0, 4, " R4PID ");
        OLED_ShowString(0, 6, ">BSTOP ");
    }
}

static void OLED_ShowCarLabels(void)
{
    /* 小车页面的标签固定不变，仅在进入小车页面时绘制。 */
    if (g_motor_test_mode != 0U) {
        OLED_ShowString(0, 0, "CAR MOTOR TEST");
        OLED_ShowString(0, 2, "S:");
        OLED_ShowString(54, 2, "T:");
    } else {
        OLED_ShowString(0, 0, "FOLLOW T:");
        OLED_ShowString(0, 2, "S:");
        OLED_ShowString(48, 2, "Q:");
    }
    OLED_ShowString(0, 4, "A:");
    OLED_ShowString(64, 4, "B:");
    OLED_ShowString(0, 6, (g_motor_test_mode != 0U) ? "PWM:" : "D:");
    if (g_motor_test_mode == 0U) {
        OLED_ShowString(72, 6, "E:");
    }
}

static void OLED_ShowServoFrame(void)
{
    /* 舵机页面的标题、单位和按键提示固定不变，只绘制一次。 */
    OLED_ShowString(0, 0, "SERVO TEST");
    OLED_ShowString(62, 2, "deg");
    OLED_ShowString(62, 4, "deg");
    OLED_ShowString(0, 6, "S:+ L:SEL D:BACK");
}

static void OLED_UpdateServoPage(void)
{
    /* 舵机动作后只更新选择箭头和两个角度值，不再整屏清空。 */
    OLED_ShowString(0, 2, (g_servo_select == SERVO_SELECT_1) ? ">S1:" : " S1:");
    OLED_ShowFixed3Unsigned(32, 2, g_servo_angle1);
    OLED_ShowString(0, 4, (g_servo_select == SERVO_SELECT_2) ? ">S2:" : " S2:");
    OLED_ShowFixed3Unsigned(32, 4, g_servo_angle2);
}

static void OLED_ShowAngleFrame(void)
{
    /* 固定标签只在进入角度测试页面时绘制一次。 */
    OLED_ShowString(0, 0, "JY901 ANGLE");
    OLED_ShowString(0, 2, "R:");
    OLED_ShowString(72, 2, "deg");
    OLED_ShowString(0, 4, "P:");
    OLED_ShowString(72, 4, "deg");
    OLED_ShowString(0, 6, "Y:");
    OLED_ShowString(72, 6, "deg");
}

static void OLED_UpdateAnglePage(void)
{
    JY901_AngleData angle;

    /*
     * JY901_GetAngle() 读取中断更新的稳定快照。
     * R/P/Y 分别表示滚转角、俯仰角和偏航角，保留两位小数。
     */
    if (JY901_GetAngle(&angle) != 0U) {
        OLED_ShowString(84, 0, " OK ");
    } else {
        OLED_ShowString(84, 0, "WAIT");
    }

    OLED_ShowFixedAngle100(18, 2, angle.roll_cdeg);
    OLED_ShowFixedAngle100(18, 4, angle.pitch_cdeg);
    OLED_ShowFixedAngle100(18, 6, angle.yaw_cdeg);
}

static void OLED_ShowTrackMask(uint8_t x, uint8_t page, uint8_t mask)
{
    char text[9];
    uint8_t i;

    /*
     * 按照传感器 1~8 的顺序紧凑显示八个电平，便于同时观察原始值和黑线判定值。
     */
    for (i = 0U; i < 8U; i++) {
        text[i] = ((mask & (1U << i)) != 0U) ? '1' : '0';
    }
    text[8] = '\0';
    OLED_ShowString(x, page, text);
}

static void OLED_ShowTrackFrame(void)
{
    /*
     * HI 是 GPIO 直接读取到的高电平状态，BLK 是换算后的黑线状态。
     * 正常情况下：白底 HI=1、BLK=0；黑线 HI=0、BLK=1。
     */
    OLED_ShowString(0, 0, "TRACK GPIO TEST");
    OLED_ShowString(0, 2, "CH:12345678");
    OLED_ShowString(0, 4, "HI:");
    OLED_ShowString(0, 6, "BLK:");
}

static void OLED_UpdateTrackPage(uint8_t raw_mask, uint8_t black_mask)
{
    /* 只覆盖八位数字区域，避免实时刷新时整屏闪烁。 */
    OLED_ShowTrackMask(24, 4, raw_mask);
    OLED_ShowTrackMask(32, 6, black_mask);
}

static const char *OLED_GetBallStateText(AppBall_State state)
{
    switch (state) {
        case BALL_TASK_LEVEL_PIPE:    return "LEVEL  ";
        case BALL_TASK_WAIT_CENTER:   return "WAIT O ";
        case BALL_TASK_MOVE_POSITIVE: return "GO +5  ";
        case BALL_TASK_MOVE_NEGATIVE: return "GO -5  ";
        case BALL_TASK_HOLD_NEGATIVE: return "HOLD-5 ";
        case BALL_TASK_COMPLETE:      return "DONE   ";
        case BALL_TASK_COMM_FAULT:    return "COMMERR";
        case BALL_TASK_BALL_LOST:     return "NO BALL";
        case BALL_TASK_TIMEOUT:       return "TIMEOUT";
        default:                      return "IDLE   ";
    }
}

static void OLED_ShowK230Frame(void)
{
    OLED_ShowString(0, 0, "H3 BALL CONTROL");
    OLED_ShowString(0, 2, "C:");
    OLED_ShowString(42, 2, "S:");
    OLED_ShowString(0, 4, "P:");
    OLED_ShowString(48, 4, "T:");
    OLED_ShowString(0, 6, "TM:");
    OLED_ShowString(54, 6, "V:");
}

static void OLED_UpdateK230Page(void)
{
    AppBall_Status ball;
    uint32_t display_time;

    AppBall_GetStatus(&ball);
    display_time = (ball.completed != 0U) ?
        ball.completion_ms : ball.elapsed_ms;

    OLED_ShowString(12, 2,
        (ball.communication_ok != 0U) ? "OK  " : "WAIT");
    OLED_ShowString(54, 2, OLED_GetBallStateText(ball.state));
    OLED_ShowBallPositionMm(12, 4, ball.position_0p1mm);
    OLED_ShowBallPositionMm(60, 4, ball.target_0p1mm);
    OLED_ShowTime100ms(18, 6, display_time);
    OLED_ShowFixedSigned(
        66,
        6,
        (ball.speed_0p1mm_per_s >= 0) ?
            ((ball.speed_0p1mm_per_s + 5) / 10) :
            ((ball.speed_0p1mm_per_s - 5) / 10));
}

static const char *OLED_GetBallStopStateText(AppBallStop_State state)
{
    switch (state) {
        case BALL_STOP_RUNNING:      return "RUN    ";
        case BALL_STOP_HOLDING:      return "HOLD   ";
        case BALL_STOP_COMPLETE:     return "DONE   ";
        case BALL_STOP_VISION_FAULT: return "VISION ";
        default:                     return "IDLE   ";
    }
}

static void OLED_ShowBallStopFrame(void)
{
    OLED_ShowString(0, 0, "ANY -> -5 STOP");
    OLED_ShowString(0, 2, "C:");
    OLED_ShowString(42, 2, "S:");
    OLED_ShowString(0, 4, "P:");
    OLED_ShowString(48, 4, "B:");
    OLED_ShowString(0, 6, "TM:");
    OLED_ShowString(54, 6, "U:");
}

static void OLED_UpdateBallStopPage(void)
{
    AppBallStop_Status stop;
    uint32_t display_time;

    AppBallStop_GetStatus(&stop);
    display_time = (stop.completed != 0U) ?
        stop.completion_ms : stop.elapsed_ms;

    OLED_ShowString(12, 2,
        (stop.measurement_valid != 0U) ? "OK  " : "WAIT");
    OLED_ShowString(54, 2, OLED_GetBallStopStateText(stop.state));
    OLED_ShowBallPositionMm(12, 4, stop.position_0p1mm);
    OLED_ShowFixedSigned(60, 4, stop.balance_pulse_us);
    OLED_ShowTime100ms(18, 6, display_time);
    OLED_ShowFixedSigned(66, 6, stop.control_offset_us);
}

static const char *OLED_GetReq4StateText(const AppReq4_Status *req4)
{
    if (req4->line_lost != 0U) {
        return "SEARCH ";
    }

    switch (req4->state) {
        case REQ4_STATE_WAIT_CENTER:  return "CENTER ";
        case REQ4_STATE_RUNNING:      return "RUN    ";
        case REQ4_STATE_COMPLETE:     return "PASS B ";
        case REQ4_STATE_VISION_FAULT: return "VISION ";
        case REQ4_STATE_BALL_LIMIT:   return "BALL>1 ";
        case REQ4_STATE_LINE_LOST:    return "NO LINE";
        case REQ4_STATE_TIMEOUT:      return "TIMEOUT";
        default:                      return "IDLE   ";
    }
}

static __attribute__((unused)) const char *OLED_GetReq4BallText(
    const AppReq4_Status *req4)
{
    if (req4->vision_fault != 0U) {
        return "VIS ";
    }
    if (req4->ball_limit_exceeded != 0U) {
        return ">10 ";
    }
    if (req4->measurement_valid != 0U) {
        return "OK  ";
    }
    return "NONE";
}

static void OLED_ShowReq4Frame(void)
{
    OLED_ShowString(0, 0, "H4 N:");
    OLED_ShowString(72, 0, "us");
    OLED_ShowString(0, 2, "C:");
    OLED_ShowString(58, 2, "T:");
    OLED_ShowString(0, 4, "V:");
    OLED_ShowString(60, 4, "P:");
    OLED_ShowString(0, 6, "D:");
    OLED_ShowString(66, 6, "U:");
}

static void OLED_UpdateReq4Page(void)
{
    AppReq4_Status req4;

    AppReq4_GetStatus(&req4);
    OLED_ShowFixedSigned(30, 0, (int32_t)req4.balance_pulse_us);
    OLED_ShowString(12, 2, OLED_GetReq4StateText(&req4));
    OLED_ShowTime100ms(
        72, 2,
        (req4.completed != 0U) ? req4.completion_ms : req4.elapsed_ms);
    OLED_ShowFixedSigned(12, 4, req4.speed_0p1mm_per_s / 10);
    OLED_ShowBallPositionMm(72, 4, req4.position_0p1mm);
    OLED_ShowFixedSigned(12, 6, (int32_t)req4.distance_count);
    OLED_ShowFixedSigned(78, 6, req4.servo_offset_us);
}

static const char *OLED_GetReq4TuneLabel(uint8_t parameter)
{
    switch ((AppReq4_TuneParameter)parameter) {
        case APP_REQ4_TUNE_POSITION_KP_X10: return ">P-KP: ";
        case APP_REQ4_TUNE_SPEED_KP_X100:   return ">V-KP: ";
        case APP_REQ4_TUNE_MAX_OFFSET_US:   return ">MAX:  ";
        case APP_REQ4_TUNE_DRIVE_STEP_US:   return ">DRIVE:";
        case APP_REQ4_TUNE_BRAKE_STEP_US:   return ">BRAKE:";
        case APP_REQ4_TUNE_BACK:            return ">BACK  ";
        default:                            return ">P-KP: ";
    }
}

static void OLED_ShowReq4TuneFrame(void)
{
    OLED_ShowString(0, 0, "REQ4 CASCADE SET");
}

static void OLED_UpdateReq4TunePage(void)
{
    AppReq4_Status req4;
    AppReq4_TuneParameter parameter =
        (AppReq4_TuneParameter)g_req4_tune_select;
    int32_t value = AppReq4_GetTuneParameter(parameter);
    const char *unit = "us";

    AppReq4_GetStatus(&req4);
    if (parameter == APP_REQ4_TUNE_POSITION_KP_X10) {
        unit = "x0.1";
    } else if (parameter == APP_REQ4_TUNE_SPEED_KP_X100) {
        unit = "x.01";
    }

    OLED_ShowString(0, 2, "                     ");
    OLED_ShowString(0, 4, "                     ");
    OLED_ShowString(0, 6, "                     ");
    OLED_ShowString(0, 2, OLED_GetReq4TuneLabel(g_req4_tune_select));
    if (parameter != APP_REQ4_TUNE_BACK) {
        OLED_ShowFixedSigned(48, 2, value);
        OLED_ShowString(96, 2, unit);
    }

    OLED_ShowString(0, 4, "P:");
    OLED_ShowBallPositionMm(12, 4, req4.position_0p1mm);
    OLED_ShowString(48, 4, "U:");
    OLED_ShowFixedSigned(60, 4, req4.servo_offset_us);
    OLED_ShowString(0, 6,
        (parameter == APP_REQ4_TUNE_BACK) ?
        "S/D:BACK L:NEXT" : "S:+ D:- L:NEXT");
}

static const char *OLED_GetBallTuneLabel(uint8_t parameter)
{
    switch ((AppBall_TuneParameter)parameter) {
        case APP_BALL_TUNE_CENTER_US:      return ">CENTER:";
        case APP_BALL_TUNE_MOVE_US:        return ">MOVE:  ";
        case APP_BALL_TUNE_BRAKE_US:       return ">BRAKE: ";
        case APP_BALL_TUNE_CAPTURE_US:     return ">CAP:   ";
        case APP_BALL_TUNE_HOLD_US:        return ">HOLD:  ";
        case APP_BALL_TUNE_SPEED_BRAKE_US: return ">V-BRK: ";
        case APP_BALL_TUNE_POS_SPEED:      return ">+SPD:  ";
        case APP_BALL_TUNE_NEG_SPEED:      return ">-SPD:  ";
        case APP_BALL_TUNE_PREDICT_MS:     return ">PRED:  ";
        case APP_BALL_TUNE_BACK:           return ">BACK   ";
        default:                           return ">CENTER:";
    }
}

static void OLED_ShowBallTuneFrame(void)
{
    OLED_ShowString(0, 0, "BALL SERVO SET");
}

static void OLED_UpdateBallTunePage(void)
{
    AppBall_Status ball;
    AppBall_TuneParameter parameter =
        (AppBall_TuneParameter)g_ball_tune_select;
    int32_t value = AppBall_GetTuneParameter(parameter);
    const char *unit = "us";

    AppBall_GetStatus(&ball);

    if ((parameter == APP_BALL_TUNE_POS_SPEED) ||
        (parameter == APP_BALL_TUNE_NEG_SPEED)) {
        value = (value + 5) / 10;
        unit = "mm/s";
    } else if (parameter == APP_BALL_TUNE_PREDICT_MS) {
        unit = "ms";
    }

    /* Clear complete rows so labels and units from the prior item disappear. */
    OLED_ShowString(0, 2, "                     ");
    OLED_ShowString(0, 4, "                     ");
    OLED_ShowString(0, 6, "                     ");

    OLED_ShowString(0, 2, OLED_GetBallTuneLabel(g_ball_tune_select));
    if (parameter != APP_BALL_TUNE_BACK) {
        OLED_ShowFixedSigned(48, 2, value);
        OLED_ShowString(96, 2, unit);
    }

    OLED_ShowString(0, 4, "P:");
    OLED_ShowBallPositionMm(12, 4, ball.position_0p1mm);
    OLED_ShowString(48, 4, "O:");
    OLED_ShowFixedSigned(60, 4, ball.control_offset_us);

    OLED_ShowString(0, 6,
        (parameter == APP_BALL_TUNE_BACK) ?
        "S/D:BACK L:NEXT" : "S:+ D:- L:NEXT");
}

static void OLED_ShowPidFrame(void)
{
    /*
     * OLED 只有 4 行可用，因此标题占第 1 行，Kp/Kd 各占一行，最后一行同时
     * 显示 BACK 和按键提示。S 表示短按，D 表示双击，L 表示长按。
     */
    OLED_ShowString(0, 0, "TRACK PD SET");
}

static void OLED_UpdatePidPage(void)
{
    /*
     * 箭头表示当前长按选择到的项目：
     *   短按：当前 Kp/Kd 增加 0.1；
     *   双击：当前 Kp/Kd 减少 0.1；
     *   长按：Kp -> Kd -> BACK 循环切换；
     *   BACK 被选中后短按或双击：返回一级菜单。
     */
    OLED_ShowString(0, 2,
        (g_pid_select == PID_SELECT_KP) ? ">KP:" : " KP:");
    OLED_ShowGain10(30, 2, g_track_kp_x10);

    OLED_ShowString(0, 4,
        (g_pid_select == PID_SELECT_KD) ? ">KD:" : " KD:");
    OLED_ShowGain10(30, 4, g_track_kd_x10);

    OLED_ShowString(0, 6,
        (g_pid_select == PID_SELECT_BACK) ?
        ">BACK S+ D- L:SEL" : " BACK S+ D- L:SEL");
}


void AppOled_Init(void)
{
    OLED_Init();
    g_oled_page_redraw = 1U;
    g_oled_update = 1U;
}

void AppOled_Task(void)
{
    static uint8_t first_refresh = 1U;
    static uint32_t last_refresh_tick_10ms = 0U;
    uint32_t now_tick_10ms = K230_GetTick10ms();
    int32_t displayA_rpm = 0;
    int32_t displayB_rpm = 0;
    uint8_t display_corner = 0U;
    uint8_t display_track_raw = 0U;
    uint8_t display_track = 0U;
    uint8_t display_line_found = 0U;
    int16_t display_error = 0;
    uint32_t display_distance_count = 0U;
    uint32_t display_run_time_ms = 0U;

        /*
         * OLED每50ms刷新一次，但这里不再调用delay_ms()忙等待。
         * 未到刷新时刻立即返回App_Run主循环，让K230解析和滚球PID持续运行。
         */
        if ((first_refresh == 0U) &&
            ((uint32_t)(now_tick_10ms - last_refresh_tick_10ms) <
             (OLED_REFRESH_PERIOD_MS / 10U))) {
            return;
        }
        first_refresh = 0U;
        last_refresh_tick_10ms = now_tick_10ms;

        displayA_rpm = speedA_rpm;
        displayB_rpm = speedB_rpm;
        display_corner = g_follow_state;
        display_track = g_track_mask;
        display_line_found = g_line_found;
        display_error = g_track_error;
        display_distance_count = g_lap_distance_x2 / 2U;
        display_run_time_ms = g_run_time_ms;

        if (g_menu_level == MENU_LEVEL_TOP) {
            if (g_oled_page_redraw != 0U) {
                /* 只有切换页面时才整屏清空并绘制固定内容。 */
                g_oled_page_redraw = 0U;
                OLED_Clear();
                OLED_ShowTopMenuFrame();
                g_oled_update = 1U;
            }
            if (g_oled_update != 0U) {
                g_oled_update = 0U;
                OLED_UpdateTopMenu();
            }
            return;
        }

        if (g_menu_page == MENU_PAGE_SERVO) {
            if (g_oled_page_redraw != 0U) {
                /* 固定标题和操作提示只写一次，角度变化时只覆盖数值区域。 */
                g_oled_page_redraw = 0U;
                OLED_Clear();
                OLED_ShowServoFrame();
                g_oled_update = 1U;
            }
            if (g_oled_update != 0U) {
                g_oled_update = 0U;
                OLED_UpdateServoPage();
            }
            return;
        }

        if (g_menu_page == MENU_PAGE_ANGLE) {
            if (g_oled_page_redraw != 0U) {
                /* 进入角度页面时清屏一次，之后只覆盖角度数字和在线状态。 */
                g_oled_page_redraw = 0U;
                g_oled_update = 0U;
                OLED_Clear();
                OLED_ShowAngleFrame();
            }
            OLED_UpdateAnglePage();
            return;
        }

        if (g_menu_page == MENU_PAGE_PID) {
            if (g_oled_page_redraw != 0U) {
                /*
                 * 进入 PID 页面时清屏并绘制固定标题。之后每次按键只覆盖选择
                 * 箭头和参数数字，减少软件 I2C 刷新导致的屏幕闪烁。
                 */
                g_oled_page_redraw = 0U;
                OLED_Clear();
                OLED_ShowPidFrame();
                g_oled_update = 1U;
            }
            if (g_oled_update != 0U) {
                g_oled_update = 0U;
                OLED_UpdatePidPage();
            }
            return;
        }

        if (g_menu_page == MENU_PAGE_TRACK) {
            uint8_t line_found;

            /*
             * 测试页面直接读取 GPIO，不依赖 10ms 中断里保存的旧值。
             * 这样即使定时器或共享变量出现问题，也能看到针脚当前的真实电平。
             */
            display_track_raw = Track_ReadRawMask();
            display_track = Track_ReadBlackMask();
            display_error = Track_GetError(display_track, &line_found);
            display_line_found = line_found;
            g_track_mask = display_track;
            g_track_error = display_error;
            g_line_found = display_line_found;

            if (g_oled_page_redraw != 0U) {
                /* 进入循迹测试页面时只清屏一次，实时数据随后局部覆盖。 */
                g_oled_page_redraw = 0U;
                g_oled_update = 0U;
                OLED_Clear();
                OLED_ShowTrackFrame();
            }
            OLED_UpdateTrackPage(display_track_raw, display_track);
            return;
        }

        if (g_menu_page == MENU_PAGE_K230) {
            if (g_oled_page_redraw != 0U) {
                /* 要求3页面只在进入时清屏，之后每50ms局部更新控制状态。 */
                g_oled_page_redraw = 0U;
                g_oled_update = 0U;
                OLED_Clear();
                OLED_ShowK230Frame();
            }
            OLED_UpdateK230Page();
            return;
        }

        if (g_menu_page == MENU_PAGE_BALL_TUNE) {
            if (g_oled_page_redraw != 0U) {
                g_oled_page_redraw = 0U;
                OLED_Clear();
                OLED_ShowBallTuneFrame();
                g_oled_update = 1U;
            }
            if (g_oled_update != 0U) {
                g_oled_update = 0U;
                OLED_UpdateBallTunePage();
            }
            return;
        }

        if (g_menu_page == MENU_PAGE_REQ4) {
            if (g_oled_page_redraw != 0U) {
                g_oled_page_redraw = 0U;
                g_oled_update = 0U;
                OLED_Clear();
                OLED_ShowReq4Frame();
            }
            OLED_UpdateReq4Page();
            return;
        }

        if (g_menu_page == MENU_PAGE_REQ4_TUNE) {
            if (g_oled_page_redraw != 0U) {
                g_oled_page_redraw = 0U;
                OLED_Clear();
                OLED_ShowReq4TuneFrame();
                g_oled_update = 1U;
            }
            if (g_oled_update != 0U) {
                g_oled_update = 0U;
                OLED_UpdateReq4TunePage();
            }
            return;
        }

        if (g_menu_page == MENU_PAGE_BALL_STOP) {
            if (g_oled_page_redraw != 0U) {
                g_oled_page_redraw = 0U;
                g_oled_update = 0U;
                OLED_Clear();
                OLED_ShowBallStopFrame();
            }
            OLED_UpdateBallStopPage();
            return;
        }

        if (g_oled_page_redraw != 0U) {
            /* 小车页面的标签不随数据变化，只在进入页面时绘制。 */
            g_oled_page_redraw = 0U;
            OLED_Clear();
            OLED_ShowCarLabels();
            g_oled_update = 1U;
        }

        if (g_oled_update != 0U) {
            g_oled_update = 0U;
            if (g_motor_test_mode != 0U) {
                OLED_ShowRunState(12, 2);
                OLED_ShowFixedSigned(66, 2, MOTOR_TEST_SPEED);
            } else {
                OLED_ShowRunState(12, 2);
                OLED_ShowFixedUnsigned(60, 2, display_corner);
            }
        }

        if (g_motor_test_mode == 0U) {
            OLED_ShowTime100ms(54, 0, display_run_time_ms);
            OLED_ShowFixedSigned(84, 6, display_error);
        }
        OLED_ShowFixedSigned(18, 4, displayA_rpm);
        OLED_ShowFixedSigned(82, 4, displayB_rpm);
        if (g_motor_test_mode != 0U) {
            OLED_ShowFixedSigned(30, 6, PWMA);
            OLED_ShowFixedSigned(78, 6, PWMB);
        } else {
            /* D 显示左右轮平均累计编码器计数，用于实车一圈里程校准。 */
            OLED_ShowFixedSigned(18, 6, (int32_t)display_distance_count);
        }

}
