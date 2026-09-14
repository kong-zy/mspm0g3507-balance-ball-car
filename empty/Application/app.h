#ifndef APPLICATION_APP_H_
#define APPLICATION_APP_H_

/*
 * 应用层统一接口。
 *
 * Hardware 文件夹负责底层硬件驱动；Application 文件夹负责把按键、OLED、
 * 循迹、终点识别、电机闭环、舵机和串口测试组织成完整的小车应用。
 *
 * empty.c 只调用下面三个接口，不再直接接触应用状态和页面内部实现。
 */

/*
 * 完成应用相关外设初始化、打开中断并进入永久主循环。
 * 调用前必须先在 empty.c 中执行 SYSCFG_DL_init()。
 */
void App_Run(void);

/*
 * 10ms 定时器中断处理入口。
 * 内部负责按键扫描、舵机刷新、编码器采样、里程状态机和电机速度闭环。
 */
void App_Timer10ms(void);

/*
 * 3ms 循迹定时器中断处理入口。
 * 内部负责读取红外阵列、计算循迹误差并更新左右轮目标速度。
 */
void App_TrackTimer3ms(void);

#endif
