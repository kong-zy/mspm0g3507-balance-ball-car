#ifndef APPLICATION_APP_OLED_H_
#define APPLICATION_APP_OLED_H_

/* 初始化 OLED，并请求第一次整页重绘。 */
void AppOled_Init(void);

/* 根据当前菜单状态执行一次页面绘制，并保持原有 50ms 刷新周期。 */
void AppOled_Task(void);

#endif /* APPLICATION_APP_OLED_H_ */
