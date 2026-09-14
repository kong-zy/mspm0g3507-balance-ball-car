#ifndef _JY901_H_
#define _JY901_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * JY901 与天猛星 MSPM0G3507 开发板接线说明：
 *
 *   JY901 VCC  -> 开发板 3V3
 *   JY901 GND  -> 开发板 GND
 *   JY901 TX   -> 开发板 PB7（JY901_UART RX，传感器数据进入单片机）
 *   JY901 RX   -> 开发板 PB6（JY901_UART TX，单片机命令进入传感器）
 *
 * 注意：
 * 1. TX 和 RX 必须交叉连接，GND 必须共地。
 * 2. 说明书标明模块支持 3.3V~5V；这里推荐使用 3.3V 供电，使串口电平
 *    与 MSPM0 的 3.3V IO 完全一致。
 * 3. JY901 使用 TTL 串口电平，不能直接连接 RS232 电平接口。
 * 4. 本工程在 empty.syscfg 中使用 UART1、9600 波特率、8N1，无校验。
 */

/* JY901 串口数据帧固定为 11 字节。 */
#define JY901_FRAME_LENGTH             (11U)
#define JY901_FRAME_HEADER             (0x55U)
#define JY901_FRAME_TYPE_ANGLE         (0x53U)

/* 超过 1 秒没有收到有效角度帧时，OLED 将显示传感器离线。 */
#define JY901_OFFLINE_TICKS_10MS       (100U)

/*
 * 对外提供的角度快照。
 * cdeg 表示 0.01 度，例如 -1234 表示 -12.34 度。
 * 使用整数可以避免在中断和 OLED 刷新中引入不必要的浮点运算。
 */
typedef struct
{
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;
    int16_t temperature_centi_c;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
    uint32_t angle_frame_count;
    uint32_t checksum_error_count;
    uint16_t no_data_ticks;
    uint8_t online;
} JY901_AngleData;

/* 初始化接收状态并开启 JY901_UART 的 NVIC 中断。 */
void JY901_Init(void);

/*
 * 每 10ms 调用一次，用于判断传感器是否掉线。
 * 本工程在 TIMER_0_INST_IRQHandler 中调用。
 */
void JY901_Tick10ms(void);

/*
 * 读取一份不会被串口中断同时修改的角度快照。
 * 返回 1 表示最近 1 秒内收到过有效角度帧，返回 0 表示仍在等待或已掉线。
 */
uint8_t JY901_GetAngle(JY901_AngleData *data);

/*
 * 接收并解析单个串口字节，供中断函数调用。
 * 函数会检查 0x55 帧头、11 字节长度和累加校验和。
 */
void JY901_ParseByte(uint8_t byte);

#endif
