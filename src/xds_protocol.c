/*
 * XDS 功率测量解析实现
 *
 * 完整复制 xds_forwarding 中 xds_protocol.c 的偏移与符号约定，
 * 保证与已实测数据（踏频、角度）一致。
 */

#include "xds_protocol.h"

static uint16_t read_le_u16(uint8_t const * p, size_t off)
{
    return (uint16_t)p[off] | (uint16_t)((uint16_t)p[off + 1] << 8);
}

static int16_t read_le_s16(uint8_t const * p, size_t off)
{
    return (int16_t)read_le_u16(p, off);
}

bool xds_power_measurement_parse(uint8_t const * p_payload,
                                 size_t          length,
                                 xds_power_measurement_t * p_output)
{
    if ((p_payload == NULL) || (p_output == NULL) || (length < 2U))
    {
        return false;
    }

    *p_output = (xds_power_measurement_t){0};

    p_output->total_power_w = read_le_u16(p_payload, 0U);

    if (length >= 4U)
    {
        p_output->left_power_w = read_le_s16(p_payload, 2U);
    }
    if (length >= 6U)
    {
        p_output->right_power_w = read_le_s16(p_payload, 4U);
    }
    if (length >= 8U)
    {
        /* 偏移 6 是有符号的：反踩时会出现负值，不能当成大踏频 */
        int16_t const cadence = read_le_s16(p_payload, 6U);
        p_output->cadence_rpm = (cadence > 0) ? (uint16_t)cadence : 0U;
    }
    if (length >= 10U)
    {
        p_output->angle_deg = (int16_t)read_le_u16(p_payload, 8U);
    }
    if (length >= 11U)
    {
        p_output->error_code = p_payload[10];
    }

    return true;
}
