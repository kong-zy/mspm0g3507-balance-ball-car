#ifndef APPLICATION_APP_FOLLOW_H_
#define APPLICATION_APP_FOLLOW_H_

/* 3ms 红外位置外环：读取八路传感器并更新左右轮目标速度。 */
void AppFollow_UpdateTarget3ms(void);

/* 10ms 编码器速度内环：把目标速度转换成左右电机 PWM。 */
void AppFollow_MotorSpeedControl10ms(void);

/* 10ms 里程及状态机：推进 Q1/Q2/Q3/Q4/Q6/Q7 状态。 */
void AppFollow_UpdateDistance10ms(void);

#endif /* APPLICATION_APP_FOLLOW_H_ */
