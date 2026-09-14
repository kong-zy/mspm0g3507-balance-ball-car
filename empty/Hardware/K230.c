#include "K230.h"

/*
 * UART2 接收中断只负责把字节放入环形缓冲区。
 * 字符串拼接和协议解析全部由主循环中的 K230_Process() 完成，
 * 避免串口中断占用过长时间而干扰循迹、电机和编码器中断。
 */
#define K230_RX_RING_SIZE          (128U)
#define K230_RX_RING_MASK          (K230_RX_RING_SIZE - 1U)
#define K230_LINE_BUFFER_SIZE      (64U)
#define K230_PROCESS_BYTE_BUDGET   (64U)
#define K230_TX_WAIT_SPIN_LIMIT    (20000UL)

static volatile uint8_t s_rx_ring[K230_RX_RING_SIZE];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;
static volatile uint32_t s_rx_overflow_count = 0U;
static volatile uint32_t s_tick_10ms = 0U;

static char s_line_buffer[K230_LINE_BUFFER_SIZE];
static uint8_t s_line_length = 0U;
static uint8_t s_discard_line = 0U;

static volatile int16_t s_target_x = 0;
static volatile int16_t s_target_y = 0;
static volatile uint8_t s_target_valid = 0U;
static volatile uint32_t s_last_pong_sequence = 0U;
static volatile uint32_t s_valid_frame_count = 0U;
static volatile uint32_t s_format_error_count = 0U;
static volatile uint32_t s_tx_timeout_count = 0U;
static volatile uint32_t s_last_test_sequence = 0U;
static volatile int16_t s_last_test_value = 0;
static volatile uint32_t s_test_frame_count = 0U;
static volatile uint32_t s_ball_sequence = 0U;
static volatile int16_t s_ball_position_0p1mm = 0;
static volatile uint16_t s_ball_pixel_x = 0U;
static volatile uint8_t s_ball_valid = 0U;
static volatile uint8_t s_ball_quality = 0U;
static volatile uint16_t s_ball_frame_age_ms = 0U;
static volatile uint32_t s_ball_frame_count = 0U;
static volatile uint32_t s_checksum_error_count = 0U;
static volatile uint32_t s_ball_last_rx_tick_10ms = 0U;
static volatile uint32_t s_last_rx_tick_10ms = 0U;

/*
 * 尝试发送一个字节，并设置有限的等待次数。
 *
 * 不能使用 DL_UART_isBusy() 等待，因为该标志在 UART 正在接收字符时也会置位。
 * 如果 K230 连续发送数据，等待 Busy 清零可能让主循环长时间卡住。
 * DL_UART_Main_transmitDataCheck() 只检查 TX 缓冲区是否有空间，适合这里使用。
 */
static uint8_t K230_SendByte(uint8_t byte)
{
    uint32_t spin;

    for (spin = 0U; spin < K230_TX_WAIT_SPIN_LIMIT; spin++) {
        if (DL_UART_Main_transmitDataCheck(K230_UART_INST, byte) == true) {
            return 1U;
        }
    }

    /*
     * 超时后立即返回，不允许通信故障拖死 OLED 和按键主循环。
     * 当前帧可能不完整，对端会在换行或下一帧处重新同步。
     */
    s_tx_timeout_count++;
    return 0U;
}

/* 不调用 printf，直接把无符号整数转换为十进制字符发送。 */
static uint8_t K230_SendUint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        return K230_SendByte((uint8_t)'0');
    }

    while ((value != 0U) && (count < (uint8_t)sizeof(digits))) {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        count++;
    }

    while (count != 0U) {
        count--;
        if (K230_SendByte((uint8_t)digits[count]) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static void K230_SendPong(uint32_t sequence)
{
    const char *prefix = "PONG,";

    while (*prefix != '\0') {
        if (K230_SendByte((uint8_t)*prefix) == 0U) {
            return;
        }
        prefix++;
    }
    if (K230_SendUint32(sequence) == 0U) {
        return;
    }
    if (K230_SendByte((uint8_t)'\r') == 0U) {
        return;
    }
    (void)K230_SendByte((uint8_t)'\n');
}

/* 测试协议要求序号固定占四位，便于 K230 逐帧比对 ACK。 */
static uint8_t K230_SendFixed4(uint32_t value)
{
    uint32_t divisor = 1000U;

    if (value > 9999U) {
        return 0U;
    }

    while (divisor != 0U) {
        uint8_t digit = (uint8_t)((value / divisor) % 10U);

        if (K230_SendByte((uint8_t)('0' + digit)) == 0U) {
            return 0U;
        }
        divisor /= 10U;
    }

    return 1U;
}

/* 收到 @K230 测试帧后，立即回送携带相同四位序号的 ACK。 */
static void K230_SendTestAck(uint32_t sequence)
{
    const char *prefix = "@ACK,";

    while (*prefix != '\0') {
        if (K230_SendByte((uint8_t)*prefix) == 0U) {
            return;
        }
        prefix++;
    }
    if (K230_SendFixed4(sequence) == 0U) {
        return;
    }
    if (K230_SendByte((uint8_t)'\r') == 0U) {
        return;
    }
    (void)K230_SendByte((uint8_t)'\n');
}

/* 解析一个十进制无符号整数，并把 cursor 移动到数字末尾。 */
static uint8_t K230_ParseUint32(const char **cursor, uint32_t *value)
{
    const char *p = *cursor;
    uint32_t result = 0U;
    uint8_t digit_count = 0U;

    while ((*p >= '0') && (*p <= '9')) {
        uint32_t digit = (uint32_t)(*p - '0');

        if (result > ((0xFFFFFFFFUL - digit) / 10U)) {
            return 0U;
        }
        result = result * 10U + digit;
        digit_count++;
        p++;
    }

    if (digit_count == 0U) {
        return 0U;
    }

    *cursor = p;
    *value = result;
    return 1U;
}

/* 解析 -32768 至 32767 范围内的十进制有符号整数。 */
static uint8_t K230_ParseInt16(const char **cursor, int16_t *value)
{
    const char *p = *cursor;
    uint8_t negative = 0U;
    uint32_t magnitude;
    uint32_t limit;

    if (*p == '-') {
        negative = 1U;
        p++;
    } else if (*p == '+') {
        p++;
    }

    if (K230_ParseUint32(&p, &magnitude) == 0U) {
        return 0U;
    }

    limit = (negative != 0U) ? 32768U : 32767U;
    if (magnitude > limit) {
        return 0U;
    }

    if (negative != 0U) {
        *value = (int16_t)(-(int32_t)magnitude);
    } else {
        *value = (int16_t)magnitude;
    }
    *cursor = p;
    return 1U;
}

/* 匹配固定字符串前缀，匹配成功后移动 cursor。 */
static uint8_t K230_ConsumePrefix(const char **cursor, const char *prefix)
{
    const char *p = *cursor;

    while (*prefix != '\0') {
        if (*p != *prefix) {
            return 0U;
        }
        p++;
        prefix++;
    }

    *cursor = p;
    return 1U;
}

/* 把一个十六进制字符转换成0~15，非法字符返回0。 */
static uint8_t K230_ParseHexNibble(char c, uint8_t *value)
{
    if ((c >= '0') && (c <= '9')) {
        *value = (uint8_t)(c - '0');
        return 1U;
    }
    if ((c >= 'A') && (c <= 'F')) {
        *value = (uint8_t)(c - 'A' + 10);
        return 1U;
    }
    if ((c >= 'a') && (c <= 'f')) {
        *value = (uint8_t)(c - 'a' + 10);
        return 1U;
    }
    return 0U;
}

/*
 * 校验 $BALL 帧末尾的两位异或校验值。
 *
 * 计算范围不包含开头的'$'，也不包含'*'和后面的两个十六进制字符。
 * 例如 $BALL,0001,0,160,1,90*AB 中，参与异或的是：
 *      BALL,0001,0,160,1,90
 */
static uint8_t K230_VerifyBallChecksum(const char *line)
{
    const char *p = line;
    uint8_t calculated = 0U;
    uint8_t high;
    uint8_t low;
    uint8_t received;

    if (*p != '$') {
        return 0U;
    }
    p++;

    while ((*p != '\0') && (*p != '*')) {
        calculated ^= (uint8_t)(*p);
        p++;
    }

    if (*p != '*') {
        return 0U;
    }
    p++;

    if ((K230_ParseHexNibble(p[0], &high) == 0U) ||
        (K230_ParseHexNibble(p[1], &low) == 0U) ||
        (p[2] != '\0')) {
        return 0U;
    }

    received = (uint8_t)((high << 4U) | low);
    return (received == calculated) ? 1U : 0U;
}

static void K230_RecordValidFrame(void)
{
    s_valid_frame_count++;
    s_last_rx_tick_10ms = s_tick_10ms;
}

/*
 * 支持五种接收帧：
 *   $BALL,...*CS       要求3正式钢球位置帧
 *   @K230,nnnn,value   压力测试帧，自动回复 @ACK,nnnn
 *   PING,n             自动回复 PONG,n
 *   PONG,n             保存 K230 确认的心跳序号
 *   TARGET,x,y,valid   保存目标坐标和有效标志
 */
static void K230_ParseLine(const char *line)
{
    const char *p = line;
    uint32_t sequence;
    uint32_t valid;
    uint32_t pixel_x;
    uint32_t quality;
    uint32_t frame_age_ms;
    int16_t x;
    int16_t y;
    int16_t test_value;
    int16_t ball_position;

    if (K230_ConsumePrefix(&p, "$BALL,") != 0U) {
        /*
         * 先做整帧校验，再解析数值。即使乱码恰好仍是数字，
         * 校验失败也不会更新钢球位置，更不会进入摆杆闭环。
         */
        if (K230_VerifyBallChecksum(line) == 0U) {
            s_checksum_error_count++;
            return;
        }

        if ((K230_ParseUint32(&p, &sequence) != 0U) &&
            (sequence <= 9999U) && (*p == ',')) {
            p++;
            if ((K230_ParseInt16(&p, &ball_position) != 0U) &&
                (ball_position >= -1250) && (ball_position <= 1250) &&
                (*p == ',')) {
                p++;
                if ((K230_ParseUint32(&p, &pixel_x) != 0U) &&
                    (pixel_x <= 4095U) && (*p == ',')) {
                    p++;
                    if ((K230_ParseUint32(&p, &valid) != 0U) &&
                        (valid <= 1U) && (*p == ',')) {
                        p++;
                        if ((K230_ParseUint32(&p, &quality) != 0U) &&
                            (quality <= 100U)) {
                            /*
                             * New frames include camera-to-UART age.  Keep
                             * accepting the old format so an older K230
                             * script cannot break requirement 3.
                             */
                            frame_age_ms = 0U;
                            if (*p == ',') {
                                p++;
                                if ((K230_ParseUint32(&p, &frame_age_ms) == 0U) ||
                                    (frame_age_ms > 500U)) {
                                    return;
                                }
                            }
                            if (*p == '*') {
                            s_ball_sequence = sequence;
                            s_ball_position_0p1mm = ball_position;
                            s_ball_pixel_x = (uint16_t)pixel_x;
                            s_ball_valid = (uint8_t)valid;
                            s_ball_quality = (uint8_t)quality;
                            s_ball_frame_age_ms = (uint16_t)frame_age_ms;
                            s_ball_frame_count++;
                            s_ball_last_rx_tick_10ms = s_tick_10ms;
                            K230_RecordValidFrame();
                            return;
                            }
                        }
                    }
                }
            }
        }
    } else if (K230_ConsumePrefix(&p, "@K230,") != 0U) {
        if ((K230_ParseUint32(&p, &sequence) != 0U) &&
            (sequence <= 9999U) && (*p == ',')) {
            p++;
            if ((K230_ParseInt16(&p, &test_value) != 0U) && (*p == '\0')) {
                s_last_test_sequence = sequence;
                s_last_test_value = test_value;
                s_test_frame_count++;
                K230_RecordValidFrame();
                K230_SendTestAck(sequence);
                return;
            }
        }
    } else if (K230_ConsumePrefix(&p, "PING,") != 0U) {
        if ((K230_ParseUint32(&p, &sequence) != 0U) && (*p == '\0')) {
            K230_RecordValidFrame();
            K230_SendPong(sequence);
            return;
        }
    } else {
        p = line;
        if (K230_ConsumePrefix(&p, "PONG,") != 0U) {
            if ((K230_ParseUint32(&p, &sequence) != 0U) && (*p == '\0')) {
                s_last_pong_sequence = sequence;
                K230_RecordValidFrame();
                return;
            }
        } else {
            p = line;
            if (K230_ConsumePrefix(&p, "TARGET,") != 0U) {
                if ((K230_ParseInt16(&p, &x) != 0U) && (*p == ',')) {
                    p++;
                    if ((K230_ParseInt16(&p, &y) != 0U) && (*p == ',')) {
                        p++;
                        if ((K230_ParseUint32(&p, &valid) != 0U) &&
                            (*p == '\0') && (valid <= 1U)) {
                            s_target_x = x;
                            s_target_y = y;
                            s_target_valid = (uint8_t)valid;
                            K230_RecordValidFrame();
                            return;
                        }
                    }
                }
            }
        }
    }

    s_format_error_count++;
}

/* 从中断环形缓冲区非阻塞取出一个字节。 */
static uint8_t K230_PopRxByte(uint8_t *byte)
{
    uint8_t tail = s_rx_tail;

    if (tail == s_rx_head) {
        return 0U;
    }

    *byte = s_rx_ring[tail];
    s_rx_tail = (uint8_t)((tail + 1U) & K230_RX_RING_MASK);
    return 1U;
}

void K230_Init(void)
{
    uint16_t i;

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow_count = 0U;
    s_tick_10ms = 0U;
    s_line_length = 0U;
    s_discard_line = 0U;

    for (i = 0U; i < K230_RX_RING_SIZE; i++) {
        s_rx_ring[i] = 0U;
    }
    for (i = 0U; i < K230_LINE_BUFFER_SIZE; i++) {
        s_line_buffer[i] = '\0';
    }

    s_target_x = 0;
    s_target_y = 0;
    s_target_valid = 0U;
    s_last_pong_sequence = 0U;
    s_valid_frame_count = 0U;
    s_format_error_count = 0U;
    s_tx_timeout_count = 0U;
    s_last_test_sequence = 0U;
    s_last_test_value = 0;
    s_test_frame_count = 0U;
    s_ball_sequence = 0U;
    s_ball_position_0p1mm = 0;
    s_ball_pixel_x = 0U;
    s_ball_valid = 0U;
    s_ball_quality = 0U;
    s_ball_frame_age_ms = 0U;
    s_ball_frame_count = 0U;
    s_checksum_error_count = 0U;
    s_ball_last_rx_tick_10ms = 0U;
    s_last_rx_tick_10ms = 0U;

    /*
     * UART2 的波特率、PA22/PB15 复用和 RX 中断源已由
     * SYSCFG_DL_init() 根据 empty.syscfg 完成。
     */
    /*
     * K230 串口设为最低优先级。电机、按键和循迹的定时器中断可以抢占它，
     * 即使视觉端发送频率较高，也不会挤占控制周期。
     */
    NVIC_SetPriority(K230_UART_INST_INT_IRQN, 3U);
    NVIC_ClearPendingIRQ(K230_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_UART_INST_INT_IRQN);
}

void K230_Tick10ms(void)
{
    s_tick_10ms++;
}

uint32_t K230_GetTick10ms(void)
{
    return s_tick_10ms;
}

void K230_Process(void)
{
    uint8_t byte;
    uint8_t processed = 0U;

    /*
     * 单次最多处理 64 字节，保证函数执行时间有上限。
     * 剩余数据保留在环形缓冲区，主循环下一次再继续处理。
     */
    while ((processed < K230_PROCESS_BYTE_BUDGET) &&
        (K230_PopRxByte(&byte) != 0U)) {
        processed++;

        /*
         * 使用 '\n' 作为一帧结束符，忽略前面的 '\r'，
         * 因此兼容 \n 和 \r\n 两种换行格式。
         */
        if (byte == (uint8_t)'\n') {
            if (s_discard_line != 0U) {
                s_discard_line = 0U;
                s_line_length = 0U;
                s_format_error_count++;
                continue;
            }

            if (s_line_length != 0U) {
                s_line_buffer[s_line_length] = '\0';
                K230_ParseLine(s_line_buffer);
                s_line_length = 0U;
            }
            continue;
        }

        if (byte == (uint8_t)'\r') {
            continue;
        }

        if (s_discard_line != 0U) {
            continue;
        }

        if (s_line_length < (K230_LINE_BUFFER_SIZE - 1U)) {
            s_line_buffer[s_line_length] = (char)byte;
            s_line_length++;
        } else {
            /* 超长行丢弃到下一换行符，随后自动重新同步。 */
            s_discard_line = 1U;
            s_line_length = 0U;
        }
    }
}

void K230_SendLine(const char *text)
{
    if (text == 0) {
        return;
    }

    while (*text != '\0') {
        if (K230_SendByte((uint8_t)*text) == 0U) {
            return;
        }
        text++;
    }
    if (K230_SendByte((uint8_t)'\r') == 0U) {
        return;
    }
    (void)K230_SendByte((uint8_t)'\n');
}

void K230_SendPing(uint32_t sequence)
{
    const char *prefix = "PING,";

    while (*prefix != '\0') {
        if (K230_SendByte((uint8_t)*prefix) == 0U) {
            return;
        }
        prefix++;
    }
    if (K230_SendUint32(sequence) == 0U) {
        return;
    }
    if (K230_SendByte((uint8_t)'\r') == 0U) {
        return;
    }
    (void)K230_SendByte((uint8_t)'\n');
}

void K230_GetStatus(K230_Status *status)
{
    uint32_t now;
    uint32_t last_rx;
    uint32_t ball_last_rx;
    uint32_t valid_count;
    uint32_t ball_count;

    if (status == 0) {
        return;
    }

    /* 复制期间只关闭 K230 UART 中断，避免状态字段更新到一半。 */
    NVIC_DisableIRQ(K230_UART_INST_INT_IRQN);
    status->target_x = s_target_x;
    status->target_y = s_target_y;
    status->target_valid = s_target_valid;
    status->last_pong_sequence = s_last_pong_sequence;
    valid_count = s_valid_frame_count;
    status->valid_frame_count = valid_count;
    status->format_error_count = s_format_error_count;
    status->rx_overflow_count = s_rx_overflow_count;
    status->tx_timeout_count = s_tx_timeout_count;
    status->last_test_sequence = s_last_test_sequence;
    status->last_test_value = s_last_test_value;
    status->test_frame_count = s_test_frame_count;
    status->ball_sequence = s_ball_sequence;
    status->ball_position_0p1mm = s_ball_position_0p1mm;
    status->ball_pixel_x = s_ball_pixel_x;
    status->ball_valid = s_ball_valid;
    status->ball_quality = s_ball_quality;
    status->ball_frame_age_ms = s_ball_frame_age_ms;
    ball_count = s_ball_frame_count;
    status->ball_frame_count = ball_count;
    status->checksum_error_count = s_checksum_error_count;
    ball_last_rx = s_ball_last_rx_tick_10ms;
    status->ball_last_rx_tick_10ms = ball_last_rx;
    last_rx = s_last_rx_tick_10ms;
    status->last_rx_tick_10ms = last_rx;
    NVIC_EnableIRQ(K230_UART_INST_INT_IRQN);

    now = s_tick_10ms;
    status->online = ((valid_count != 0U) &&
        ((uint32_t)(now - last_rx) <= K230_OFFLINE_TICKS_10MS)) ? 1U : 0U;
    status->ball_online = ((ball_count != 0U) &&
        ((uint32_t)(now - ball_last_rx) <=
         K230_BALL_OFFLINE_TICKS_10MS)) ? 1U : 0U;
}

/*
 * SysConfig 将该函数名宏展开为 UART2_IRQHandler。
 * 中断只保存字节，不解析字符串。
 */
void K230_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(K230_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            uint8_t head = s_rx_head;
            uint8_t next = (uint8_t)((head + 1U) & K230_RX_RING_MASK);
            uint8_t byte = DL_UART_Main_receiveData(K230_UART_INST);

            if (next == s_rx_tail) {
                s_rx_overflow_count++;
            } else {
                s_rx_ring[head] = byte;
                s_rx_head = next;
            }
            break;
        }

        default:
            break;
    }
}
