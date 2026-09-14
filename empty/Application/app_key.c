#include "Hardware/board.h"
#include "Application/app_state.h"
#include "Application/app_key.h"
#include "Application/app_ball.h"
#include "Application/app_ball_stop.h"
#include "Application/app_req4.h"

/*
 * 本模块只处理按键输入和由按键直接触发的操作。
 * PA18 引脚、按下为高电平、30ms 消抖及原有操作方式均未改变。
 */
static uint8_t Read_KeyPressed(void)
{
    /*
     * 恢复原工程 PA18 外接按键的有效电平：松开时为低电平，按下时为高电平。
     * 本函数把高电平换算为统一的“1=已经按下”，后面的短按、长按和双击
     * 状态机保持不变。
     */
    return (DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) != 0U) ? 1U : 0U;
}

static uint8_t Key_ScanAction(void)
{
    static uint8_t last_raw = 0U;
    static uint8_t stable_state = 0U;
    static uint8_t debounce_ticks = 0U;
    static uint16_t press_ticks = 0U;
    static uint8_t long_action_sent = 0U;
    static uint8_t wait_double = 0U;
    static uint8_t double_ticks = 0U;
    static uint8_t suppress_release = 0U;
    uint8_t raw = Read_KeyPressed();
    uint8_t action = KEY_ACTION_NONE;

    /*
     * 每 10ms 调用一次。先做 30ms 消抖，再区分短按和长按。
     * 长按动作触发后，松手时不会再额外产生短按动作。
     */
    if (raw == last_raw) {
        if (debounce_ticks < KEY_DEBOUNCE_TICKS) {
            debounce_ticks++;
        }
    } else {
        last_raw = raw;
        debounce_ticks = 0U;
    }

    if ((debounce_ticks >= KEY_DEBOUNCE_TICKS) && (stable_state != raw)) {
        stable_state = raw;
        if (stable_state != 0U) {
            if (wait_double != 0U) {
                wait_double = 0U;
                double_ticks = 0U;
                suppress_release = 1U;
                long_action_sent = 1U;
                action = KEY_ACTION_DOUBLE;
            } else {
                long_action_sent = 0U;
            }
            press_ticks = 0U;
        } else {
            if (suppress_release != 0U) {
                suppress_release = 0U;
            } else if ((long_action_sent == 0U) && (press_ticks > 0U)) {
                wait_double = 1U;
                double_ticks = 0U;
            }
            press_ticks = 0U;
        }
    }

    if (stable_state != 0U) {
        if (press_ticks < 60000U) {
            press_ticks++;
        }
        if ((press_ticks >= KEY_LONG_PRESS_TICKS) && (long_action_sent == 0U)) {
            long_action_sent = 1U;
            wait_double = 0U;
            double_ticks = 0U;
            action = KEY_ACTION_LONG;
        }
    }

    if ((action == KEY_ACTION_NONE) && (wait_double != 0U)) {
        if (double_ticks < KEY_DOUBLE_WINDOW_TICKS) {
            double_ticks++;
        } else {
            wait_double = 0U;
            double_ticks = 0U;
            action = KEY_ACTION_SHORT;
        }
    }

    return action;
}

static void Servo_TestStep(void)
{
    if (g_servo_select == SERVO_SELECT_1) {
        g_servo_angle1 += SERVO_ANGLE_STEP_DEG;
        if (g_servo_angle1 > SERVO_MAX_ANGLE_DEG) {
            g_servo_angle1 = SERVO_MIN_ANGLE_DEG;
        }
    } else {
        g_servo_angle2 += SERVO_ANGLE_STEP_DEG;
        if (g_servo_angle2 > SERVO_MAX_ANGLE_DEG) {
            g_servo_angle2 = SERVO_MIN_ANGLE_DEG;
        }
    }

    Servo_SetAngle(g_servo_angle1, g_servo_angle2);
}

static uint8_t Servo_PulseToAngle(float pulse_us, uint8_t reversed)
{
    float angle;
    uint8_t logical_angle;

    if (pulse_us <= (float)SERVO_MIN_PULSE_US) {
        logical_angle = SERVO_MIN_ANGLE_DEG;
    } else if (pulse_us >= (float)SERVO_MAX_PULSE_US) {
        logical_angle = SERVO_MAX_ANGLE_DEG;
    } else {
        angle =
            (pulse_us - (float)SERVO_MIN_PULSE_US) *
            (float)SERVO_MAX_ANGLE_DEG /
            (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

        logical_angle = (uint8_t)(angle + 0.5f);
    }

    if (reversed != 0U) {
        logical_angle =
            (uint8_t)(SERVO_MAX_ANGLE_DEG - logical_angle);
    }

    return logical_angle;
}

static void Servo_TestCaptureCurrentPosition(void)
{
    float position1 = Servo_Position1;
    float position2 = Servo_Position2;

    /*
     * Freeze the current pulses before the SERVO page takes control.
     * This avoids jumping back to angles retained from an earlier test.
     */
    Servo_SetTarget(position1, position2);
    g_servo_angle1 = Servo_PulseToAngle(
        position1, SERVO_1_ANGLE_REVERSED);
    g_servo_angle2 = Servo_PulseToAngle(
        position2, SERVO_2_ANGLE_REVERSED);
}

static void PID_AdjustSelected(int8_t direction)
{
    /*
     * direction > 0 表示增加，direction < 0 表示减少。到达上下限后保持不变，
     * 不采用从最大值突然跳回最小值的循环方式，避免误按一次导致控制特性突变。
     */
    if (g_pid_select == PID_SELECT_KP) {
        if ((direction > 0) &&
            (g_track_kp_x10 <=
             (TRACK_PD_KP_MAX_X10 - TRACK_PD_STEP_X10))) {
            g_track_kp_x10 += TRACK_PD_STEP_X10;
        } else if ((direction < 0) &&
                   (g_track_kp_x10 >=
                    (TRACK_PD_KP_MIN_X10 + TRACK_PD_STEP_X10))) {
            g_track_kp_x10 -= TRACK_PD_STEP_X10;
        }
    } else if (g_pid_select == PID_SELECT_KD) {
        if ((direction > 0) &&
            (g_track_kd_x10 <=
             (TRACK_PD_KD_MAX_X10 - TRACK_PD_STEP_X10))) {
            g_track_kd_x10 += TRACK_PD_STEP_X10;
        } else if ((direction < 0) &&
                   (g_track_kd_x10 >=
                    (TRACK_PD_KD_MIN_X10 + TRACK_PD_STEP_X10))) {
            g_track_kd_x10 -= TRACK_PD_STEP_X10;
        }
    }
}

void AppKey_Update10ms(void)
{
    uint8_t key_value = Key_ScanAction();

    if (key_value == KEY_ACTION_NONE) {
        return;
    }

    if (g_menu_level == MENU_LEVEL_TOP) {
        /*
         * 一级菜单：短按只移动光标，使用局部刷新；
         * 长按进入二级页面，页面内容完全不同，因此请求整页重画。
         */
        if (key_value == KEY_ACTION_SHORT) {
            g_menu_page++;
            if (g_menu_page > MENU_PAGE_LAST) {
                g_menu_page = MENU_PAGE_CAR;
            }
        } else if (key_value == KEY_ACTION_LONG) {
            g_menu_level = MENU_LEVEL_DETAIL;
            Flag_Stop = 1;
            g_oled_page_redraw = 1U;

            /*
             * CAR 保持双电机等速测试；PID 页面只允许停车调参；FOLLOW 启用
             * 八路循迹和一圈启停状态机。各模式分开，避免查看或修改参数时
             * 意外启动电机。
             */
            if (g_menu_page == MENU_PAGE_CAR) {
                g_motor_test_mode = 1U;
            } else if (g_menu_page == MENU_PAGE_FOLLOW) {
                g_motor_test_mode = 0U;
                g_follow_state = FOLLOW_STATE_IDLE;
            }

            if (g_menu_page == MENU_PAGE_SERVO) {
                Set_PWM(0, 0);
                AppBallStop_RequestStop();
                Servo_TestCaptureCurrentPosition();
            } else if (g_menu_page == MENU_PAGE_K230) {
                /*
                 * 进入要求3页面时先保持摆杆水平，必须由用户短按明确启动。
                 * 浏览菜单本身不会让钢球突然运动。
                 */
                AppBallStop_RequestStop();
                AppBall_RequestStop();
            } else if (g_menu_page == MENU_PAGE_BALL_TUNE) {
                AppBallStop_RequestStop();
                AppBall_RequestStop();
                g_ball_tune_select = APP_BALL_TUNE_CENTER_US;
            } else if (g_menu_page == MENU_PAGE_REQ4) {
                /* 要求三和要求四共用滚球舵机，进入要求四时先释放要求三。 */
                AppBallStop_RequestStop();
                AppBall_RequestStop();
                AppReq4_RequestStop();
                g_motor_test_mode = 0U;
                g_follow_state = FOLLOW_STATE_IDLE;
            } else if (g_menu_page == MENU_PAGE_REQ4_TUNE) {
                AppBallStop_RequestStop();
                AppBall_RequestStop();
                AppReq4_RequestStop();
                g_req4_tune_select = APP_REQ4_TUNE_POSITION_KP_X10;
                g_motor_test_mode = 0U;
                g_follow_state = FOLLOW_STATE_IDLE;
            } else if (g_menu_page == MENU_PAGE_BALL_STOP) {
                /* 独立刹停页不检查起点，短按后直接控制到-5cm。 */
                AppBall_RequestStop();
                AppReq4_RequestStop();
                AppBallStop_RequestStop();
                g_motor_test_mode = 0U;
                g_follow_state = FOLLOW_STATE_IDLE;
            }
        }
        g_oled_update = 1U;
        return;
    }

    if (g_menu_page == MENU_PAGE_PID) {
        /*
         * PID 调参页面必须始终保持停车。参数修改只改变下一次 FOLLOW 运行时
         * 使用的循迹 PD 系数，不在调参过程中驱动电机。
         */
        Flag_Stop = 1;
        g_follow_state = FOLLOW_STATE_IDLE;
        g_track_targetA = 0;
        g_track_targetB = 0;
        PWMA = 0;
        PWMB = 0;
        Motor_PID_Reset();
        Set_PWM(0, 0);

        if (key_value == KEY_ACTION_LONG) {
            /* 长按依次选择 Kp、Kd 和 BACK。 */
            g_pid_select++;
            if (g_pid_select > PID_SELECT_BACK) {
                g_pid_select = PID_SELECT_KP;
            }
        } else if (g_pid_select == PID_SELECT_BACK) {
            /*
             * BACK 被选中后，短按或双击都返回一级菜单。PID 页面中的双击平时
             * 用于减小参数，因此不能再使用所有页面共用的“双击立即返回”逻辑。
             */
            if ((key_value == KEY_ACTION_SHORT) ||
                (key_value == KEY_ACTION_DOUBLE)) {
                g_menu_level = MENU_LEVEL_TOP;
                g_oled_page_redraw = 1U;
            }
        } else if (key_value == KEY_ACTION_SHORT) {
            PID_AdjustSelected(1);
        } else if (key_value == KEY_ACTION_DOUBLE) {
            PID_AdjustSelected(-1);
        }

        g_oled_update = 1U;
        return;
    }

    if (g_menu_page == MENU_PAGE_BALL_TUNE) {
        /*
         * Runtime ball-servo tuning is only allowed while the car and the
         * requirement-3 state machine are stopped.  Short press increments,
         * double press decrements, and long press selects the next item.
         */
        Flag_Stop = 1;
        g_follow_state = FOLLOW_STATE_IDLE;
        g_track_targetA = 0;
        g_track_targetB = 0;
        Set_PWM(0, 0);
        AppBall_RequestStop();

        if (key_value == KEY_ACTION_LONG) {
            g_ball_tune_select++;
            if (g_ball_tune_select >= APP_BALL_TUNE_COUNT) {
                g_ball_tune_select = APP_BALL_TUNE_CENTER_US;
            }
        } else if (g_ball_tune_select == APP_BALL_TUNE_BACK) {
            if ((key_value == KEY_ACTION_SHORT) ||
                (key_value == KEY_ACTION_DOUBLE)) {
                g_menu_level = MENU_LEVEL_TOP;
                g_oled_page_redraw = 1U;
            }
        } else if (key_value == KEY_ACTION_SHORT) {
            AppBall_AdjustTuneParameter(
                (AppBall_TuneParameter)g_ball_tune_select, 1);
        } else if (key_value == KEY_ACTION_DOUBLE) {
            AppBall_AdjustTuneParameter(
                (AppBall_TuneParameter)g_ball_tune_select, -1);
        }

        g_oled_update = 1U;
        return;
    }

    if (g_menu_page == MENU_PAGE_REQ4_TUNE) {
        Flag_Stop = 1;
        g_follow_state = FOLLOW_STATE_IDLE;
        g_track_targetA = 0;
        g_track_targetB = 0;
        Set_PWM(0, 0);
        AppReq4_RequestStop();

        if (key_value == KEY_ACTION_LONG) {
            g_req4_tune_select++;
            if (g_req4_tune_select >= APP_REQ4_TUNE_COUNT) {
                g_req4_tune_select = APP_REQ4_TUNE_POSITION_KP_X10;
            }
        } else if (g_req4_tune_select == APP_REQ4_TUNE_BACK) {
            if ((key_value == KEY_ACTION_SHORT) ||
                (key_value == KEY_ACTION_DOUBLE)) {
                g_menu_level = MENU_LEVEL_TOP;
                g_oled_page_redraw = 1U;
            }
        } else if (key_value == KEY_ACTION_SHORT) {
            AppReq4_AdjustTuneParameter(
                (AppReq4_TuneParameter)g_req4_tune_select, 1);
        } else if (key_value == KEY_ACTION_DOUBLE) {
            AppReq4_AdjustTuneParameter(
                (AppReq4_TuneParameter)g_req4_tune_select, -1);
        }

        g_oled_update = 1U;
        return;
    }

    if (key_value == KEY_ACTION_DOUBLE) {
        /* 从任意二级页面双击返回一级菜单，需要重新绘制一级菜单框架。 */
        if (g_menu_page == MENU_PAGE_K230) {
            AppBall_RequestStop();
        } else if (g_menu_page == MENU_PAGE_REQ4) {
            AppReq4_RequestStop();
        } else if (g_menu_page == MENU_PAGE_BALL_STOP) {
            AppBallStop_RequestStop();
        }
        Flag_Stop = 1;
        g_follow_state = FOLLOW_STATE_IDLE;
        g_track_targetA = 0;
        g_track_targetB = 0;
        Set_PWM(0, 0);
        g_menu_level = MENU_LEVEL_TOP;
        g_oled_page_redraw = 1U;
        g_oled_update = 1U;
        return;
    }

    if (g_menu_page == MENU_PAGE_K230) {
        /*
         * H题要求3页面：
         *   短按：从O点开始执行“+5cm -> -5cm并稳定”；
         *   长按：立即停止任务并把摆杆恢复中位；
         *   双击：已在上方统一处理，停止并返回一级菜单。
         *
         * 本页面始终禁止车轮运动，保证测试的是题目要求的“小车静止状态”。
         */
        Flag_Stop = 1;
        g_follow_state = FOLLOW_STATE_IDLE;
        g_track_targetA = 0;
        g_track_targetB = 0;
        Set_PWM(0, 0);

        if (key_value == KEY_ACTION_SHORT) {
            AppBall_RequestStart();
        } else if (key_value == KEY_ACTION_LONG) {
            AppBall_RequestStop();
        }

        g_oled_update = 1U;
        return;
    }

    if (g_menu_page == MENU_PAGE_BALL_STOP) {
        /*
         * 独立刹停测试：短按从当前可见位置直接前往-50mm；
         * 长按停止并回到BALL SET中的舵机中心；双击返回由上方处理。
         */
        Flag_Stop = 1;
        g_follow_state = FOLLOW_STATE_IDLE;
        g_track_targetA = 0;
        g_track_targetB = 0;
        Set_PWM(0, 0);

        if (key_value == KEY_ACTION_SHORT) {
            AppBall_RequestStop();
            AppReq4_RequestStop();
            AppBallStop_RequestStart();
        } else if (key_value == KEY_ACTION_LONG) {
            AppBallStop_RequestStop();
        }

        g_oled_update = 1U;
        return;
    }

    if (g_menu_page == MENU_PAGE_REQ4) {
        /*
         * 短按：检查K230与O点位置，连续3帧合格后自动启动A到B；
         * 长按：立即停车并恢复平衡脉宽；双击返回由上方统一处理。
         */
        g_motor_test_mode = 0U;
        if (key_value == KEY_ACTION_SHORT) {
            AppBall_RequestStop();
            AppReq4_RequestStart();
        } else if (key_value == KEY_ACTION_LONG) {
            AppReq4_RequestStop();
        }

        g_oled_update = 1U;
        return;
    }

    if (g_menu_page == MENU_PAGE_SERVO) {
        /*
         * 舵机二级菜单：短按增加当前舵机角度，长按切换舵机；
         * 两种操作都只改变少量字符，所以只请求局部刷新。
         */
        Flag_Stop = 1;
        Set_PWM(0, 0);
        if (key_value == KEY_ACTION_SHORT) {
            Servo_TestStep();
        } else if (key_value == KEY_ACTION_LONG) {
            g_servo_select = (g_servo_select == SERVO_SELECT_1) ? SERVO_SELECT_2 : SERVO_SELECT_1;
        }
        g_oled_update = 1U;
        return;
    }

    if ((g_menu_page == MENU_PAGE_ANGLE) ||
        (g_menu_page == MENU_PAGE_TRACK)) {
        /*
         * 角度和循迹测试页面只负责观察数据，不改变硬件配置。
         * 强制保持小车停止；双击返回操作已在上方统一处理。
         */
        Flag_Stop = 1;
        Set_PWM(0, 0);
        return;
    }

    if (Flag_Stop != 0) {
        /*
         * 当前只完成 H 题基础的一圈循迹，因此不再设置目标圈数。
         * 停车状态下短按或长按都可以启动；启动时统一清零里程、计时、
         * 终点记录、循迹目标和速度闭环历史，并从“离开起点横线”状态开始。
         */
        if ((key_value == KEY_ACTION_SHORT) ||
            (key_value == KEY_ACTION_LONG)) {
            Flag_Stop = 0;
            g_follow_restart = 1U;
            g_lap_distance_x2 = 0U;
            g_run_time_ms = 0U;
            g_follow_state = FOLLOW_STATE_LEAVE_START;
            g_track_targetA = TRACK_START_SPEED;
            g_track_targetB = TRACK_START_SPEED;
            Motor_PID_Reset();
        }
    } else {
        /* 小车运行时，短按或长按都立即停车。 */
        if ((key_value == KEY_ACTION_SHORT) || (key_value == KEY_ACTION_LONG)) {
            Flag_Stop = 1;
            g_follow_state = FOLLOW_STATE_IDLE;
            g_track_targetA = 0;
            g_track_targetB = 0;
        }
    }

    g_oled_update = 1U;
}
