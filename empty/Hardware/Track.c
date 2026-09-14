#include "Track.h"

static uint8_t Track_ReadOne(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

static uint8_t Track_LevelToBlack(uint8_t level)
{
    return (level == TRACK_BLACK_LEVEL) ? 1U : 0U;
}

void Track_Init(void)
{
    /*
     * SysConfig 已经把转接板右侧排母上的 8 个循迹引脚生成为 GPIO 输入。
     * 这里再打开内部上拉和施密特触发，提高数字输出的抗干扰能力。
     */
    DL_GPIO_initDigitalInputFeatures(TRACK_L4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_L3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_L2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_L1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_R1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_R2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_R3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_R4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

uint8_t Track_ReadRawMask(void)
{
    uint8_t mask = 0U;

    if (Track_ReadOne(TRACK_L4_PORT, TRACK_L4_PIN) != 0U) {
        mask |= TRACK_MASK_L4;
    }
    if (Track_ReadOne(TRACK_L3_PORT, TRACK_L3_PIN) != 0U) {
        mask |= TRACK_MASK_L3;
    }
    if (Track_ReadOne(TRACK_L2_PORT, TRACK_L2_PIN) != 0U) {
        mask |= TRACK_MASK_L2;
    }
    if (Track_ReadOne(TRACK_L1_PORT, TRACK_L1_PIN) != 0U) {
        mask |= TRACK_MASK_L1;
    }
    if (Track_ReadOne(TRACK_R1_PORT, TRACK_R1_PIN) != 0U) {
        mask |= TRACK_MASK_R1;
    }
    if (Track_ReadOne(TRACK_R2_PORT, TRACK_R2_PIN) != 0U) {
        mask |= TRACK_MASK_R2;
    }
    if (Track_ReadOne(TRACK_R3_PORT, TRACK_R3_PIN) != 0U) {
        mask |= TRACK_MASK_R3;
    }
    if (Track_ReadOne(TRACK_R4_PORT, TRACK_R4_PIN) != 0U) {
        mask |= TRACK_MASK_R4;
    }

    return mask;
}

uint8_t Track_ReadBlackMask(void)
{
    uint8_t mask = 0U;

    if (Track_LevelToBlack(Track_ReadOne(TRACK_L4_PORT, TRACK_L4_PIN)) != 0U) {
        mask |= TRACK_MASK_L4;
    }
    if (Track_LevelToBlack(Track_ReadOne(TRACK_L3_PORT, TRACK_L3_PIN)) != 0U) {
        mask |= TRACK_MASK_L3;
    }
    if (Track_LevelToBlack(Track_ReadOne(TRACK_L2_PORT, TRACK_L2_PIN)) != 0U) {
        mask |= TRACK_MASK_L2;
    }
    if (Track_LevelToBlack(Track_ReadOne(TRACK_L1_PORT, TRACK_L1_PIN)) != 0U) {
        mask |= TRACK_MASK_L1;
    }
    if (Track_LevelToBlack(Track_ReadOne(TRACK_R1_PORT, TRACK_R1_PIN)) != 0U) {
        mask |= TRACK_MASK_R1;
    }
    if (Track_LevelToBlack(Track_ReadOne(TRACK_R2_PORT, TRACK_R2_PIN)) != 0U) {
        mask |= TRACK_MASK_R2;
    }
    if (Track_LevelToBlack(Track_ReadOne(TRACK_R3_PORT, TRACK_R3_PIN)) != 0U) {
        mask |= TRACK_MASK_R3;
    }
    if (Track_LevelToBlack(Track_ReadOne(TRACK_R4_PORT, TRACK_R4_PIN)) != 0U) {
        mask |= TRACK_MASK_R4;
    }

    return mask;
}

uint8_t Track_CountBlackSensors(uint8_t black_mask)
{
    uint8_t count = 0U;
    uint8_t i;

    /*
     * 统计传入位掩码中共有多少路探头被置位。本函数既可以统计某一帧同时
     * 检测到黑线的探头数，也可以统计主控制程序在短距离窗口内按位或得到的
     * “累计命中过黑线的不同探头数”。后一种用法允许 A 点横线斜着扫过阵列，
     * 即使各探头不在同一时刻触发，也能得到完整的横线覆盖数量。
     */
    for (i = 0U; i < 8U; i++) {
        if ((black_mask & (1U << i)) != 0U) {
            count++;
        }
    }

    return count;
}

int16_t Track_GetError(uint8_t black_mask, uint8_t *line_found)
{
    int16_t sum = 0;
    int16_t count = 0;

    /*
     * 八路传感器从左到右的权值为 -7,-5,-3,-1,+1,+3,+5,+7。
     * 返回值为负：黑线偏左；返回值为正：黑线偏右。
     */
    if ((black_mask & TRACK_MASK_L4) != 0U) {
        sum -= 7;
        count++;
    }
    if ((black_mask & TRACK_MASK_L3) != 0U) {
        sum -= 5;
        count++;
    }
    if ((black_mask & TRACK_MASK_L2) != 0U) {
        sum -= 3;
        count++;
    }
    if ((black_mask & TRACK_MASK_L1) != 0U) {
        sum -= 1;
        count++;
    }
    if ((black_mask & TRACK_MASK_R1) != 0U) {
        sum += 1;
        count++;
    }
    if ((black_mask & TRACK_MASK_R2) != 0U) {
        sum += 3;
        count++;
    }
    if ((black_mask & TRACK_MASK_R3) != 0U) {
        sum += 5;
        count++;
    }
    if ((black_mask & TRACK_MASK_R4) != 0U) {
        sum += 7;
        count++;
    }

    if (count == 0) {
        *line_found = 0U;
        return 0;
    }

    *line_found = 1U;
    return (int16_t)(sum / count);
}
