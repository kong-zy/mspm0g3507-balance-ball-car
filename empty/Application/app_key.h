#ifndef APPLICATION_APP_KEY_H_
#define APPLICATION_APP_KEY_H_

/*
 * 每 10ms 调用一次：读取 PA18，完成消抖、短按、长按和双击识别，
 * 然后处理菜单切换、PID 调节、舵机测试以及循迹启停。
 */
void AppKey_Update10ms(void);

#endif /* APPLICATION_APP_KEY_H_ */
