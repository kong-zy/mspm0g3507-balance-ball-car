#include "Hardware/board.h"
#include "Application/app_state.h"
#include "Application/app_follow.h"

/*
 * 循迹模块包含三部分：
 * 1. 3ms 红外位置 PD，产生左右轮目标速度；
 * 2. 10ms 编码器速度闭环，产生 PWM；
 * 3. 一圈状态机，负责 5 米后减速、累计 4 路识别 A 线立即停车、
 *    Q6 右转找线和 6.25 米 Q7 安全停车。
 *
 * 控制参数全部在 app_state.h 中，算法与拆分前保持一致。
 */
void AppFollow_UpdateTarget3ms(void)
{
    /*
     * last_error 用于计算误差变化量；last_direction 用于短时丢线时沿最后一次
     * 看见黑线的方向搜索。离开起点仍使用连续采样计数确认；终点则改用短距离
     * 多帧累计窗口，允许启停横线从阵列一侧斜着扫到另一侧。
     */
    static int16_t last_error = 0;
    static int8_t last_direction = 0;
    static uint16_t line_lost_ticks = 0U;
    static uint8_t leave_line_ticks = 0U;
    static uint8_t finish_window_active = 0U;
    static uint8_t finish_hit_mask = 0U;
    static uint32_t finish_window_start_count = 0U;
    static uint8_t ramp_ticks = 0U;
    static int32_t ramp_speed = TRACK_START_SPEED;
    uint8_t line_found;
    uint8_t black_count;
    uint8_t new_finish_hit_mask;
    uint8_t accumulated_finish_count;
    int16_t error;
    int16_t abs_error;
    int16_t derivative;
    int32_t base_speed;
    int32_t turn;
    int32_t targetA;
    int32_t targetB;
    uint32_t distance_count;

    /*
     * 每次中断都重新读取 8 路传感器。g_track_mask 的每一位表示对应探头是否
     * 检测到黑线；black_count 表示当前检测到黑线的探头总数。普通轨迹通常
     * 只覆盖少数探头，而 A 点横线会覆盖阵列中的大部分探头。
     */
    g_track_mask = Track_ReadBlackMask();
    error = Track_GetError(g_track_mask, &line_found);
    black_count = Track_CountBlackSensors(g_track_mask);
    g_track_error = error;
    g_line_found = line_found;
    distance_count = g_lap_distance_x2 / 2U;

    if (g_motor_test_mode != 0U) {
        /* 电机测试页面不参与循迹，左右轮始终使用相同的测试目标速度。 */
        g_track_targetA = MOTOR_TEST_SPEED;
        g_track_targetB = MOTOR_TEST_SPEED;
        return;
    }

    if (g_follow_restart != 0U) {
        /*
         * 每次重新启动都必须清除上一圈留下的微分历史、丢线方向、终点累计
         * 掩码和速度斜坡。否则上一圈的横线命中记录可能让新一圈提前停车，
         * 上一圈结束前的较大误差也可能让新一圈起步时产生过大差速。
         */
        g_follow_restart = 0U;
        last_error = (line_found != 0U) ? error : 0;
        last_direction = 0;
        line_lost_ticks = 0U;
        leave_line_ticks = 0U;
        finish_window_active = 0U;
        finish_hit_mask = 0U;
        finish_window_start_count = 0U;
        ramp_ticks = 0U;
        ramp_speed = TRACK_START_SPEED;
    }

    /*
     * 起点与终点使用同一条横线，因此必须分阶段判断：
     *
     * 状态 1 中，至少 5 路探头同时为黑色视为仍压在起点横线上。只有黑色探头
     * 少于 5 路、平均里程超过 100 个计数，并连续保持 6 个采样周期后，才认为
     * 小车已经离开起点。这样启动瞬间不会把 A 点误认为完成一圈。
     *
     * 状态 3 中已经由编码器确认行驶超过 5 米，程序固定使用较低速度循迹。
     * 当左侧或右侧外部探头开始检测到黑色时，开启一个约 5.8 厘米的累计窗口。
     * 窗口中每一帧的黑色位掩码都执行按位或运算，因此同一探头重复触发只计算
     * 一次，而不同时间先后触发的探头会被合并。累计至少命中4个不同探头，
     * 就确认这是斜着扫过的A点横线，不再要求左右两侧分别命中。
     */
    if (g_follow_state == FOLLOW_STATE_LEAVE_START) {
        if ((black_count < START_LINE_MIN_BLACK) &&
            (distance_count >= START_LEAVE_MIN_COUNTS)) {
            if (leave_line_ticks < START_LINE_CONFIRM_TICKS) {
                leave_line_ticks++;
            }
            if (leave_line_ticks >= START_LINE_CONFIRM_TICKS) {
                g_follow_state = FOLLOW_STATE_RUNNING;
                g_oled_update = 1U;
            }
        } else {
            leave_line_ticks = 0U;
        }
    } else if (g_follow_state == FOLLOW_STATE_FINISH_ARMED) {
        if (finish_window_active == 0U) {
            /*
             * 普通循迹线通常位于阵列中间，因此只有 L2～L4 或 R2～R4 中至少
             * 一路变黑时才启动累计。启动帧本身也要保存，不能丢掉最先压到
             * 横线的那一侧探头。
             */
            if ((g_track_mask & FINISH_WINDOW_START_MASK) != 0U) {
                finish_window_active = 1U;
                finish_hit_mask = g_track_mask;
                finish_window_start_count = distance_count;
            }
        } else {
            /*
             * 找出本帧中新出现的黑色探头并合并到累计掩码。同一路探头在横线
             * 上持续为黑只占一个二进制位，不会被重复计数。
             */
            new_finish_hit_mask =
                (uint8_t)(g_track_mask & (uint8_t)(~finish_hit_mask));
            if (new_finish_hit_mask != 0U) {
                finish_hit_mask |= new_finish_hit_mask;
            }
        }

        if (finish_window_active != 0U) {
            accumulated_finish_count =
                Track_CountBlackSensors(finish_hit_mask);

            if (accumulated_finish_count >=
                FINISH_ACCUMULATED_MIN_BLACK) {
                /*
                 * A 点横线识别条件已经成立。本版不再进入低速停车接近阶段，
                 * 而是在当前 3 毫秒控制周期内直接完成以下动作：
                 *
                 * 1. 状态切换到 Q4，表示正常识别终点并完成停车；
                 * 2. 设置 Flag_Stop，使后续循迹中断和速度闭环停止驱动车轮；
                 * 3. 左右目标速度、PWM 软件变量和实际 PWM 寄存器全部清零；
                 * 4. 清除速度 PI 历史，防止下一次启动继承停车前的积分输出。
                 *
                 * 清零完成后立即 return，避免本函数后面的丢线处理或 PD 计算
                 * 再次覆盖已经置零的电机目标速度。
                 */
                g_follow_state = FOLLOW_STATE_STOPPED;
                Flag_Stop = 1;
                g_track_targetA = 0;
                g_track_targetB = 0;
                PWMA = 0;
                PWMB = 0;
                Motor_PID_Reset();
                Set_PWM(0, 0);
                finish_window_active = 0U;
                finish_hit_mask = 0U;
                g_oled_update = 1U;
                return;
            } else if ((distance_count - finish_window_start_count) >=
                       FINISH_WINDOW_ENCODER_COUNTS) {
                /*
                 * 在规定距离内没有累计命中4路，本次多半只是普通弯道偏差或
                 * 短暂噪声。丢弃本窗口，后续遇到新的外侧黑线时重新开始，
                 * 不让旧命中无限累积形成误判。
                 */
                finish_window_active = 0U;
                finish_hit_mask = 0U;
                finish_window_start_count = distance_count;
            }
        }
    }

    if (line_found == 0U) {
        /*
         * 所有探头都没有检测到黑线时，先根据 last_direction 进行低速找线：
         * 最后一次黑线在左侧，就降低左轮速度使车头向左寻找；最后一次黑线在
         * 右侧，则降低右轮速度。连续丢线达到 67 个采样周期（约 0.2 秒）后，
         * 进入状态 6，随后不再根据历史方向，而是固定低速向右转动寻找黑线。
         * 状态 6 不设置停车标志，因此 3 毫秒循迹中断和 10 毫秒电机闭环会
         * 继续运行。
         */
        if (line_lost_ticks < LINE_LOST_STOP_TICKS) {
            line_lost_ticks++;
        }
        if (line_lost_ticks >= LINE_LOST_STOP_TICKS) {
            g_follow_state = FOLLOW_STATE_LINE_LOST;
            g_track_targetA = TRACK_LOST_SPEED;
            g_track_targetB = TRACK_LOST_SPEED / 2;
            g_oled_update = 1U;
            return;
        }

        targetA = TRACK_LOST_SPEED;
        targetB = TRACK_LOST_SPEED;
        if (last_direction < 0) {
            targetA = TRACK_LOST_SPEED / 2;
        } else if (last_direction > 0) {
            targetB = TRACK_LOST_SPEED / 2;
        }
        g_track_targetA = targetA;
        g_track_targetB = targetB;
        return;
    }

    if (g_follow_state == FOLLOW_STATE_LINE_LOST) {
        /*
         * 状态 6 搜索过程中重新检测到黑线后立即恢复正常循迹。若累计里程已经
         * 达到 5 米，则直接回到状态 3，以终点搜索低速继续识别 A 点横线；
         * 否则回到状态 2。把 last_error 设为本次误差，可避免重新找到黑线的
         * 第一帧产生过大的微分冲击。
         */
        if (distance_count >= FINISH_ARM_DISTANCE_COUNTS) {
            g_follow_state = FOLLOW_STATE_FINISH_ARMED;
        } else {
            g_follow_state = FOLLOW_STATE_RUNNING;
        }
        /*
         * Q6 找线期间车身姿态变化较大，之前尚未完成的横线累计窗口不再可靠。
         * 恢复正常循迹时清空该窗口，避免把找线前后的两段无关黑线拼成终点。
         */
        finish_window_active = 0U;
        finish_hit_mask = 0U;
        finish_window_start_count = distance_count;
        last_error = error;
        g_oled_update = 1U;
    }

    line_lost_ticks = 0U;
    abs_error = (error < 0) ? (int16_t)(-error) : error;
    if (error < 0) {
        last_direction = -1;
    } else if (error > 0) {
        last_direction = 1;
    }

    /*
     * 根据误差绝对值选择基础速度：中心附近使用最高速度；偏差逐渐增大时依次
     * 降低速度，为转向留出余量。状态 3 等待终点横线时固定使用终点搜索低速，
     * 让传感器有更长时间覆盖横线；确认终点后会在识别函数中立即停车。
     */
    if (g_follow_state == FOLLOW_STATE_FINISH_ARMED) {
        base_speed = TRACK_FINISH_SEARCH_SPEED;
    } else if (abs_error <= 1) {
        base_speed = TRACK_STRAIGHT_SPEED;
    } else if (abs_error <= 3) {
        base_speed = TRACK_CURVE_SPEED;
    } else if (abs_error <= 5) {
        base_speed = TRACK_SHARP_SPEED;
    } else {
        base_speed = TRACK_RECOVERY_SPEED;
    }

    /*
     * 启动阶段使用速度斜坡限制：大约每 100 毫秒把允许的最高目标速度增加
     * 1 个编码器计数。即使当前误差很小，也不能立即从起步速度跳到直线速度，
     * 可减少轮胎打滑和车体前后冲击。终点识别成功后函数已经直接返回，
     * 因此不会再执行本段速度斜坡计算。
     */
    if (ramp_ticks < TRACK_RAMP_TICKS) {
        ramp_ticks++;
    } else {
        ramp_ticks = 0U;
        if (ramp_speed < TRACK_STRAIGHT_SPEED) {
            ramp_speed++;
        }
    }
    if (base_speed > ramp_speed) {
        base_speed = ramp_speed;
    }

    derivative = error - last_error;
    /*
     * 比例项根据当前偏差决定基本转向量，微分项根据偏差变化速度抑制蛇形摆动：
     *
     * 转向量 = (Kp放大10倍值 × 当前误差
     *          + Kd放大10倍值 × 本次与上次误差之差) / 10
     *
     * 参数初值仍等效为 Kp=1.0、Kd=1.5，但可以在 OLED 的 PID 页面修改。
     * 数字探头输出是离散的，因此不加入积分项，避免弯道中积累的转向量在
     * 出弯后仍然继续作用，造成反向摆动。
     */
    turn = ((int32_t)g_track_kp_x10 * error +
            (int32_t)g_track_kd_x10 * derivative) / TRACK_PD_SCALE;
    last_error = error;

    /*
     * 误差为正表示黑线位于车体右侧，此时提高 A 轮目标速度、降低 B 轮目标
     * 速度，使车头向右修正；误差为负时计算结果自动相反。最后把目标速度限制
     * 在 0 到 TRACK_TARGET_MAX 之间，避免强修正时出现反转或超过闭环能力。
     */
    targetA = base_speed + turn;
    targetB = base_speed - turn;
    if (targetA < 0) {
        targetA = 0;
    } else if (targetA > TRACK_TARGET_MAX) {
        targetA = TRACK_TARGET_MAX;
    }
    if (targetB < 0) {
        targetB = 0;
    } else if (targetB > TRACK_TARGET_MAX) {
        targetB = TRACK_TARGET_MAX;
    }

    g_track_targetA = targetA;
    g_track_targetB = targetB;
}

void AppFollow_MotorSpeedControl10ms(void)
{
    int32_t targetA;
    int32_t targetB;

    /*
     * 电机测试模式保持 A/B 等速；FOLLOW 模式读取 3ms 循迹中断发布的最新目标。
     * 编码器和速度 PID 仍严格每 10ms 执行，原来的速度单位和 PI 参数不变。
     */
    if (g_motor_test_mode != 0U) {
        targetA = MOTOR_TEST_SPEED;
        targetB = MOTOR_TEST_SPEED;
    } else {
        targetA = g_track_targetA;
        targetB = g_track_targetB;
    }

    PWMA = -Velocity_A((int)targetA, (int)encoderA_cnt);
    PWMB = -Velocity_B((int)targetB, (int)encoderB_cnt);
    PWMA = limit_PWM(PWMA, -PWM_LIMIT, PWM_LIMIT);
    PWMB = limit_PWM(PWMB, -PWM_LIMIT, PWM_LIMIT);
    Set_PWM(PWMA, PWMB);
}

void AppFollow_UpdateDistance10ms(void)
{
    uint32_t countA;
    uint32_t countB;
    uint32_t delta_x2;
    uint32_t distance_count;

    if ((g_menu_level != MENU_LEVEL_DETAIL) ||
        (g_menu_page != MENU_PAGE_FOLLOW) ||
        (g_motor_test_mode != 0U) ||
        (Flag_Stop != 0)) {
        return;
    }

    /*
     * 编码器方向受左右电机安装方向影响，但计算车体行驶距离时只需要增量大小，
     * 因此分别取绝对值。正常循迹时保留两轮计数之和存入 g_lap_distance_x2，
     * 可避免整数平均时丢失 0.5 个计数的精度。
     *
     * 如果当前处于 Q6，左右轮的运动主要用于把车头向右转回黑线，并不等于
     * 沿标准赛道继续前进，此时令本周期里程增量为零。编码器速度闭环仍照常
     * 工作，只有“一圈累计里程”暂停。
     */
    countA = (encoderA_cnt < 0) ?
        (uint32_t)(-encoderA_cnt) : (uint32_t)encoderA_cnt;
    countB = (encoderB_cnt < 0) ?
        (uint32_t)(-encoderB_cnt) : (uint32_t)encoderB_cnt;
    delta_x2 = countA + countB;
    if (g_follow_state == FOLLOW_STATE_LINE_LOST) {
        delta_x2 = 0U;
    }
    g_lap_distance_x2 += delta_x2;
    distance_count = g_lap_distance_x2 / 2U;

    if (g_run_time_ms <= (UINT32_MAX - CONTROL_PERIOD_MS)) {
        /* 仅在 FOLLOW 页面实际运行时累计时间，停车后保持最终成绩不再变化。 */
        g_run_time_ms += CONTROL_PERIOD_MS;
    }

    if ((g_follow_state == FOLLOW_STATE_RUNNING) &&
        (distance_count >= FINISH_ARM_DISTANCE_COUNTS)) {
        /*
         * 平均里程达到约 5 米后，从状态 2 转入状态 3。该阈值明显小于赛道
         * 6.14 米总长，既能在到达 A 点前及时开放识别，又能避开启动横线。
         */
        g_follow_state = FOLLOW_STATE_FINISH_ARMED;
        g_oled_update = 1U;
    }

    if ((g_follow_state == FOLLOW_STATE_FINISH_ARMED) &&
        (distance_count >= FINISH_FAILSAFE_DISTANCE_COUNTS)) {
        /*
         * 已开放终点识别且沿赛道累计里程达到约 6.25 米仍未完成一圈，通常意味着
         * 横线识别参数不合适或传感器极性错误。Q6 找线期间已经暂停累计圈长，
         * 因此不会因原地右转搜索而提前触发本保护。达到保护值后进入状态 7
         * 并停车，避免车辆在赛道上无限循环运行。
         */
        g_follow_state = FOLLOW_STATE_FINISH_MISSED;
        Flag_Stop = 1;
        g_track_targetA = 0;
        g_track_targetB = 0;
        PWMA = 0;
        PWMB = 0;
        Motor_PID_Reset();
        Set_PWM(0, 0);
        g_oled_update = 1U;
    }

}
