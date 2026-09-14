#ifndef _TRACK_H_
#define _TRACK_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 感为八路数字循迹模块接线说明，从小车前方看由左到右排列：
 *
 *   OUT1 / D1 / S1 -> PB4   排母丝印 B04，最左侧 L4
 *   OUT2 / D2 / S2 -> PB5   排母丝印 B05，左侧   L3
 *   OUT3 / D3 / S3 -> PB8   排母丝印 B08，左侧   L2
 *   OUT4 / D4 / S4 -> PB9   排母丝印 B09，左内侧 L1
 *   OUT5 / D5 / S5 -> PA12  排母丝印 A12，右内侧 R1
 *   OUT6 / D6 / S6 -> PB23  排母丝印 B23，右侧   R2
 *   OUT7 / D7 / S7 -> PB26  排母丝印 B26，右侧   R3
 *   OUT8 / D8 / S8 -> PB27  排母丝印 B27，最右侧 R4
 *   VCC -> 5V（感为模块推荐供电电压）
 *   GND -> GND，必须和 MSPM0 控制板共地
 *
 * 感为模块使用 5V 供电并连接 3.3V MSPM0 时，应安装 PULL/开漏模式跳帽；
 * 驱动会在这些输入脚上开启 3.3V 内部上拉。模块检测到黑线时输出低电平。
 * 如果你的模块检测到黑线时输出高电平，把 TRACK_BLACK_LEVEL 改为 1U。
 */
#define TRACK_BLACK_LEVEL       (0U)

#define TRACK_MASK_L4           (0x01U)
#define TRACK_MASK_L3           (0x02U)
#define TRACK_MASK_L2           (0x04U)
#define TRACK_MASK_L1           (0x08U)
#define TRACK_MASK_R1           (0x10U)
#define TRACK_MASK_R2           (0x20U)
#define TRACK_MASK_R3           (0x40U)
#define TRACK_MASK_R4           (0x80U)

void Track_Init(void);
uint8_t Track_ReadRawMask(void);
uint8_t Track_ReadBlackMask(void);
int16_t Track_GetError(uint8_t black_mask, uint8_t *line_found);
uint8_t Track_CountBlackSensors(uint8_t black_mask);

#endif
