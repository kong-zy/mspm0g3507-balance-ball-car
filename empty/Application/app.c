#include "Hardware/board.h"
#include "Application/app.h"
#include "Application/app_state.h"
#include "Application/app_key.h"
#include "Application/app_oled.h"
#include "Application/app_follow.h"
#include "Application/app_ball.h"
#include "Application/app_ball_stop.h"
#include "Application/app_req4.h"

/*
 * 应用层总调度文件。
 *
 * App_Run() 只负责硬件初始化与后台显示任务；
 * App_Timer10ms() 调度按键、舵机、编码器速度环和里程状态机；
 * App_TrackTimer3ms() 调度高频红外位置环。
 *
 * 引脚、速度参数、PID 参数和终点识别条件均放在原有驱动或 app_state.h 中，
 * 本文件不再包含具体页面绘制和循迹算法。
 */
void App_Run(void)
{
    /*
     * SYSCFG_DL_init() 已由 empty.c 调用。下列顺序与拆分前一致，
     * 避免改变舵机、串口、OLED、编码器和定时器的启动时序。
     */
    Servo_Init();
    Servo_SetAngle(g_servo_angle1, g_servo_angle2);
    Track_Init();
    /* Contest build: gyro use is prohibited; do not initialize JY901. */
    K230_Init();
    AppBall_Init();
    AppReq4_Init();
    AppBallStop_Init();
    AppOled_Init();

    DL_TimerA_startCounter(PWM_0_INST);

    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    /*
     * 3ms 定时器只负责红外采样和位置外环；
     * 编码器读取及电机速度内环仍由 10ms 定时器完成。
     */
    NVIC_ClearPendingIRQ(TRACK_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(TRACK_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(TRACK_TIMER_INST);

    while (1)
    {
        /*
         * K230串口和滚球状态机都放在后台主循环：
         * UART中断只收字节，主循环做校验、解析和PID计算。
         * OLED任务按10ms系统节拍限频，不使用50ms忙等待堵住这里。
         */
        K230_Process();
        K230_GetStatus(&g_k230_test_status);
        AppBall_BackgroundTask(&g_k230_test_status);
        AppReq4_BackgroundTask(&g_k230_test_status);
        AppBallStop_BackgroundTask(&g_k230_test_status);
        AppOled_Task();
    }
}

void App_Timer10ms(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMERG_IIDX_ZERO)
    {
        LED_Flash(100);
        AppKey_Update10ms();
        Servo_Update();
        /* Contest build: no JY901 polling or gyro feedback. */
        K230_Tick10ms();

        /* 读取本次 10ms 内的编码器计数，B 电机方向与 A 相反，所以这里取反。 */
        encoderA_cnt = Get_Encoder_countA;
        encoderB_cnt = -Get_Encoder_countB;
        Get_Encoder_countA = 0;
        Get_Encoder_countB = 0;

        speedA_rpm = (int32_t)Calculate_Motor_RPM((int)encoderA_cnt, CONTROL_PERIOD_MS);
        speedB_rpm = (int32_t)Calculate_Motor_RPM((int)encoderB_cnt, CONTROL_PERIOD_MS);

        /* 使用本次 10ms 的左右编码器增量累计行程并推进一圈启停状态机。 */
        AppFollow_UpdateDistance10ms();
        AppReq4_UpdateDistance10ms();

        if ((g_menu_level == MENU_LEVEL_DETAIL) &&
            ((g_menu_page == MENU_PAGE_CAR) ||
             (g_menu_page == MENU_PAGE_FOLLOW) ||
             (g_menu_page == MENU_PAGE_REQ4)) &&
            (!Flag_Stop)) {
            AppFollow_MotorSpeedControl10ms();
        } else {
            uint8_t line_found;

            /*
             * 停车期间持续清除速度 PI 状态，防止下一次启动继承上一次转弯时
             * A/B 两侧不同的累计 PWM，确保 E=0 时两轮从相同状态开始加速。
             */
            Motor_PID_Reset();
            PWMA = 0;
            PWMB = 0;
            Set_PWM(0, 0);
            g_track_mask = Track_ReadBlackMask();
            g_track_error = Track_GetError(g_track_mask, &line_found);
            g_line_found = line_found;
        }
    }
}

void App_TrackTimer3ms(void)
{
    if (DL_TimerG_getPendingInterrupt(TRACK_TIMER_INST) == DL_TIMERG_IIDX_ZERO)
    {
        /*
         * 只在 FOLLOW 运行页面进行 3ms 循迹计算。
         * CAR 电机测试及其他页面不会被循迹目标干扰。
         */
        if ((g_menu_level == MENU_LEVEL_DETAIL) &&
            (g_motor_test_mode == 0U) && (!Flag_Stop)) {
            if (g_menu_page == MENU_PAGE_FOLLOW) {
                AppFollow_UpdateTarget3ms();
            } else if (g_menu_page == MENU_PAGE_REQ4) {
                AppReq4_UpdateTarget3ms();
            }
        }
    }
}
