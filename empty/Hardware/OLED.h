#ifndef _OLED_H_
#define _OLED_H_

#include "ti_msp_dl_config.h"

/*
 * 天猛星 MSPM0G3507 排母上的 0.96 寸 OLED 接线说明：
 *
 * OLED VCC      -> 3.3V
 * OLED GND      -> GND
 * OLED D0/SCL   -> PA1
 * OLED D1/SDA   -> PA0
 *
 * 常见 4 针 SSD1306 模块的 IIC 地址是 0x3C。本驱动使用软件 IIC，
 * 不需要在 SysConfig 里额外开启硬件 I2C 外设。请在 SYSCFG_DL_init()
 * 之后调用 OLED_Init()。
 */
#define OLED_WIDTH                 (128U)
#define OLED_HEIGHT                (64U)
#define OLED_PAGE_NUM              (8U)
#define OLED_I2C_ADDR              (0x3CU)

/*
 * OLED_PORT / OLED_SCL_PIN / OLED_SDA_PIN / OLED_SCL_IOMUX /
 * OLED_SDA_IOMUX 由 empty.syscfg 中名为 OLED 的 GPIO 族自动生成。
 */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(uint8_t data);
void OLED_Set_Pos(uint8_t x, uint8_t page);
void OLED_Write_Cmd(uint8_t cmd);
void OLED_Write_Data(uint8_t data);

void OLED_ShowChar(uint8_t x, uint8_t page, char chr);
void OLED_ShowString(uint8_t x, uint8_t page, const char *str);
void OLED_ShowSignedNum(uint8_t x, uint8_t page, int32_t num);
void OLED_ShowUnsignedNum(uint8_t x, uint8_t page, uint32_t num);

#endif
