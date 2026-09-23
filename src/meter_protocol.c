/*
 * 功率计私有帧解析实现 —— 游标式
 *
 * 用一个读游标按线序依次取字段。关键在于游标是「粘性失败」的：
 * 一旦某次读取越界，游标即被毒化，后续读取一律失败。
 *
 * 这样帧被截短时，尾部那些根本没出现的字段自然保持为 0，
 * 调用处不必在每个字段上都重复一遍长度判断 —— 长度只在游标里处理一次。
 */

#include "meter_protocol.h"

/** @brief 帧内读游标；left 表示还剩多少字节可读，为 0 表示已毒化 */
typedef struct
{
    uint8_t const *at;
    size_t         left;
} cursor_t;

static bool take_u8(cursor_t *c, uint8_t *out)
{
    if (c->left < 1U)
    {
        c->left = 0U;   /* 毒化：之后的读取不会再成功 */
        return false;
    }

    *out = c->at[0];

    c->at   += 1;
    c->left -= 1U;

    return true;
}

static bool take_u16(cursor_t *c, uint16_t *out)
{
    if (c->left < 2U)
    {
        c->left = 0U;   /* 毒化 */
        return false;
    }

    *out = (uint16_t)((uint16_t)c->at[0] | ((uint16_t)c->at[1] << 8));

    c->at   += 2;
    c->left -= 2U;

    return true;
}

static bool take_s16(cursor_t *c, int16_t *out)
{
    uint16_t raw;

    if (!take_u16(c, &raw))
    {
        return false;
    }

    *out = (int16_t)raw;

    return true;
}

bool meter_frame_parse(uint8_t const *payload, size_t length, meter_frame_t *out)
{
    if ((payload == NULL) || (out == NULL) || (length < 2U))
    {
        return false;
    }

    *out = (meter_frame_t){0};

    cursor_t c = {.at = payload, .left = length};

    uint16_t u16 = 0U;
    int16_t  s16 = 0;
    uint8_t  u8  = 0U;

    /* 上面的长度检查已保证至少 2 字节，这一步必定成功 */
    (void)take_u16(&c, &u16);
    out->total_w = u16;

    if (take_s16(&c, &s16))
    {
        out->left_w = s16;
    }

    if (take_s16(&c, &s16))
    {
        out->right_w = s16;
    }

    if (take_s16(&c, &s16))
    {
        /* 反踩时该字段为负。负值并不代表「很高的踏频」，一律归零。 */
        out->rpm = (s16 > 0) ? (uint16_t)s16 : 0U;
    }

    if (take_u16(&c, &u16))
    {
        out->crank_angle = (int16_t)u16;
    }

    if (take_u8(&c, &u8))
    {
        out->fault = u8;
    }

    return true;
}
