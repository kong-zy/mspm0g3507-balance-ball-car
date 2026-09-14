/*
 * 主程序入口。
 *
 * 本文件只保留三项最顶层职责：
 * 1. 调用 SysConfig 生成的芯片基础初始化；
 * 2. 启动 Application 应用层；
 * 3. 把两个定时器中断转交给 Application 处理。
 *
 * 按键、OLED、PID 调参、循迹、终点识别、编码器和电机闭环的具体逻辑统一
 * 放在 Application/app.c 中，使主函数保持简洁。
 */

#include "Hardware/board.h"
#include "Application/app.h"

int main(void)
{
    SYSCFG_DL_init();
    App_Run();

    /*
     * App_Run() 内部是永久主循环，正常情况下不会返回。
     * 保留 return 语句是为了满足标准 C 语言 main() 返回类型要求。
     */
    return 0;
}

void TIMER_0_INST_IRQHandler(void)
{
    App_Timer10ms();
}

void TRACK_TIMER_INST_IRQHandler(void)
{
    App_TrackTimer3ms();
}
