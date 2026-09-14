#ifndef HARDWARE_K230_H_
#define HARDWARE_K230_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * MSPM0G3507 与 K230 接线：
 *
 *   K230 UART_TX -> PA22（MSPM0 UART2_RX）
 *   K230 UART_RX -> PB15（MSPM0 UART2_TX）
 *   K230 GND     -> MSPM0 GND
 *
 * TX/RX 必须交叉连接，两块板必须共地。
 * empty.syscfg 已配置为 115200 波特率、8 数据位、无校验、1 停止位。
 * K230 功耗较大，应使用自己的可靠电源，不要由 MSPM0 的 3.3V 引脚供电。
 *
 * 本驱动使用可直接通过串口助手观察的 ASCII 行协议。
 * 要求3的正式钢球位置帧为：
 *
 *   $BALL,SEQ,POS10,PIXEL_X,VALID,QUALITY,FRAME_AGE_MS*CS\r\n
 *
 * For compatibility, the receiver also accepts the original packet without
 * FRAME_AGE_MS.  FRAME_AGE_MS is the elapsed time from camera exposure to
 * packet construction and lets the controller compensate real vision delay.
 *
 * 字段说明：
 *   $BALL   ：固定帧头；
 *   SEQ     ：0~9999循环帧序号；
 *   POS10   ：钢球相对中心O的位置，单位0.1mm，正方向指向+5cm；
 *   PIXEL_X ：钢球圆心的原始横向像素，便于标定；
 *   VALID   ：1表示本帧检测到钢球，0表示未检测到；
 *   QUALITY ：识别质量0~100；
 *   CS      ：从字符B开始，到星号前最后一个字符为止逐字节异或，
 *             用两位大写十六进制表示。
 *
 * 示例：
 *   $BALL,0042,-500,224,1,86*3A
 * 表示第42帧，钢球位于O点负方向50.0mm处，圆心x=224，识别有效。
 *
 * 原有 @K230/@ACK、PING/PONG 和 TARGET 帧仍然保留，便于独立测试通信。
 */

#define K230_OFFLINE_TICKS_10MS    (200U)
/* 钢球闭环要求低延迟，超过150ms没有新BALL帧即判定视觉数据过期。 */
#define K230_BALL_OFFLINE_TICKS_10MS (15U)

typedef struct
{
    int16_t target_x;
    int16_t target_y;
    uint8_t target_valid;
    uint8_t online;
    uint32_t last_pong_sequence;
    uint32_t valid_frame_count;
    uint32_t format_error_count;
    uint32_t rx_overflow_count;
    uint32_t tx_timeout_count;
    uint32_t last_test_sequence;
    int16_t last_test_value;
    uint32_t test_frame_count;
    uint32_t ball_sequence;
    int16_t ball_position_0p1mm;
    uint16_t ball_pixel_x;
    uint8_t ball_valid;
    uint8_t ball_quality;
    uint16_t ball_frame_age_ms;
    uint8_t ball_online;
    uint32_t ball_frame_count;
    uint32_t checksum_error_count;
    uint32_t ball_last_rx_tick_10ms;
    uint32_t last_rx_tick_10ms;
} K230_Status;

/* 必须在 SYSCFG_DL_init() 之后调用。 */
void K230_Init(void);

/* 每 10ms 调用一次，为在线超时和心跳提供时间基准。 */
void K230_Tick10ms(void);

/* 返回从 K230_Init() 开始累计的 10ms 节拍。 */
uint32_t K230_GetTick10ms(void);

/*
 * 非阻塞处理接收数据，主循环应持续调用。
 * 每次最多处理固定数量的字节，即使 K230 连续发送也会按时返回。
 */
void K230_Process(void);

/* 发送一行文本，自动添加 \r\n。 */
void K230_SendLine(const char *text);

/* 发送 PING,序号 心跳帧。 */
void K230_SendPing(uint32_t sequence);

/* 读取 K230 的最新状态快照。 */
void K230_GetStatus(K230_Status *status);

#endif
