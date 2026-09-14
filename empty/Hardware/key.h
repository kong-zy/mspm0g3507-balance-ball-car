#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"
#define KEY  DL_GPIO_readPins(KEY_PORT,KEY_key_PIN)
/*
 * KEY 返回 PA18 的原始电平：非 0 表示外接按键已按下，0 表示按键松开。
 * 主程序使用 Read_KeyPressed() 统一转换为“1=按下”；本宏保留给旧按键接口。
 */
uint8_t click_N_Double (uint8_t time);  //单击按键扫描和双击按键扫描
uint8_t click(void);               //单击按键扫描
uint8_t Long_Press(void);           //长按扫描
void Key(void);
#endif
