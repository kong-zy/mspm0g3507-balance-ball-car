#include "JY901.h"

/* 串口中断逐字节填充该缓冲区，凑齐 11 字节后再校验和解析。 */
static uint8_t s_rx_buffer[JY901_FRAME_LENGTH];
static uint8_t s_rx_index = 0U;

/*
 * 下列变量在 UART 中断中写入、在主循环中读取，因此声明为 volatile。
 * 主循环通过临时关闭 JY901 UART 中断来取得同一时刻的一组完整数据。
 */
static volatile int16_t s_roll_raw = 0;
static volatile int16_t s_pitch_raw = 0;
static volatile int16_t s_yaw_raw = 0;
static volatile int16_t s_temperature_centi_c = 0;
static volatile uint32_t s_angle_frame_count = 0U;
static volatile uint32_t s_checksum_error_count = 0U;
static volatile uint16_t s_no_data_ticks = JY901_OFFLINE_TICKS_10MS + 1U;
static volatile uint8_t s_has_angle = 0U;

/* 将连续的低字节、高字节组合成带符号的 16 位数。 */
static int16_t JY901_MakeInt16(uint8_t low_byte, uint8_t high_byte)
{
    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte);
}

/*
 * 说明书给出的换算关系为：角度 = 原始值 / 32768 * 180 度。
 * 这里将结果扩大 100 倍，直接得到 0.01 度单位，便于 OLED 显示两位小数。
 */
static int16_t JY901_RawToCentiDegree(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 18000L) / 32768L);
}

void JY901_Init(void)
{
    uint8_t i;

    /* SYSCFG_DL_init() 已完成 UART1 外设配置，这里只清除软件接收状态。 */
    s_rx_index = 0U;
    for (i = 0U; i < JY901_FRAME_LENGTH; i++) {
        s_rx_buffer[i] = 0U;
    }

    s_roll_raw = 0;
    s_pitch_raw = 0;
    s_yaw_raw = 0;
    s_temperature_centi_c = 0;
    s_angle_frame_count = 0U;
    s_checksum_error_count = 0U;
    s_no_data_ticks = JY901_OFFLINE_TICKS_10MS + 1U;
    s_has_angle = 0U;

    /* 开启 UART1 的 CPU 中断；RX 中断源已在 empty.syscfg 中打开。 */
    NVIC_ClearPendingIRQ(JY901_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(JY901_UART_INST_INT_IRQN);
}

void JY901_Tick10ms(void)
{
    /* 饱和计数，避免传感器长时间未连接时发生无符号数回绕。 */
    if (s_no_data_ticks < 0xFFFFU) {
        s_no_data_ticks++;
    }
}

void JY901_ParseByte(uint8_t byte)
{
    uint8_t checksum = 0U;
    uint8_t i;

    /* 未找到帧头时丢弃其他字节，直到收到 0x55。 */
    if (s_rx_index == 0U) {
        if (byte == JY901_FRAME_HEADER) {
            s_rx_buffer[0] = byte;
            s_rx_index = 1U;
        }
        return;
    }

    s_rx_buffer[s_rx_index] = byte;
    s_rx_index++;

    /* 一帧未收满 11 字节时直接返回，等待下一次 RX 中断。 */
    if (s_rx_index < JY901_FRAME_LENGTH) {
        return;
    }

    /* 前 10 字节逐字节累加，低 8 位必须等于第 11 字节。 */
    for (i = 0U; i < (JY901_FRAME_LENGTH - 1U); i++) {
        checksum = (uint8_t)(checksum + s_rx_buffer[i]);
    }

    if (checksum == s_rx_buffer[JY901_FRAME_LENGTH - 1U]) {
        if (s_rx_buffer[1] == JY901_FRAME_TYPE_ANGLE) {
            /* 0x53 角度帧：Roll、Pitch、Yaw、温度均为低字节在前。 */
            s_roll_raw = JY901_MakeInt16(s_rx_buffer[2], s_rx_buffer[3]);
            s_pitch_raw = JY901_MakeInt16(s_rx_buffer[4], s_rx_buffer[5]);
            s_yaw_raw = JY901_MakeInt16(s_rx_buffer[6], s_rx_buffer[7]);
            s_temperature_centi_c = JY901_MakeInt16(s_rx_buffer[8], s_rx_buffer[9]);
            s_angle_frame_count++;
            s_no_data_ticks = 0U;
            s_has_angle = 1U;
        }
    } else {
        /* 校验失败通常由波特率不一致、接线松动或干扰造成。 */
        s_checksum_error_count++;
    }

    /* 当前帧处理完毕，重新等待下一帧的 0x55 帧头。 */
    s_rx_index = 0U;
}

uint8_t JY901_GetAngle(JY901_AngleData *data)
{
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;

    if (data == 0) {
        return 0U;
    }

    /*
     * 复制期间只关闭 JY901 的 UART1 中断，不影响电机的 10ms 定时中断、
     * 编码器中断或其他外设，因此读取快照不会破坏原有控制功能。
     */
    NVIC_DisableIRQ(JY901_UART_INST_INT_IRQN);
    roll_raw = s_roll_raw;
    pitch_raw = s_pitch_raw;
    yaw_raw = s_yaw_raw;
    data->temperature_centi_c = s_temperature_centi_c;
    data->angle_frame_count = s_angle_frame_count;
    data->checksum_error_count = s_checksum_error_count;
    data->no_data_ticks = s_no_data_ticks;
    data->online = ((s_has_angle != 0U) &&
        (s_no_data_ticks <= JY901_OFFLINE_TICKS_10MS)) ? 1U : 0U;
    NVIC_EnableIRQ(JY901_UART_INST_INT_IRQN);

    data->roll_raw = roll_raw;
    data->pitch_raw = pitch_raw;
    data->yaw_raw = yaw_raw;
    data->roll_cdeg = JY901_RawToCentiDegree(roll_raw);
    data->pitch_cdeg = JY901_RawToCentiDegree(pitch_raw);
    data->yaw_cdeg = JY901_RawToCentiDegree(yaw_raw);

    return data->online;
}

/*
 * SysConfig 将该宏展开为 UART1_IRQHandler。
 * 每次 RX 中断只取出一个字节并交给协议解析器，保持中断处理尽量简短。
 */
void JY901_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(JY901_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            JY901_ParseByte(DL_UART_Main_receiveData(JY901_UART_INST));
            break;

        default:
            break;
    }
}
