/*
 * 功率计私有帧解析
 * =================
 *
 * 本文件按协议事实独立实现，不含任何第三方源码。字段偏移属于实现互操作
 * 所必需的功能性信息。
 *
 * 帧布局（实测通知固定 11 字节，小端序）：
 *
 *   偏移  宽度  类型   含义
 *   ----  ----  -----  ------------------------------
 *     0     2   u16    总功率，瓦特
 *     2     2   s16    左侧功率，瓦特
 *     4     2   s16    右侧功率，瓦特
 *     6     2   s16    踏频，rpm（反踩时为负）
 *     8     2   u16    曲柄角度，度（0~359 循环）
 *    10     1   u8     错误码
 *
 * 实测佐证：
 *   - 偏移 6-7 取 0x28 / 0x34 / 0x48 时分别对应 40 / 52 / 72 rpm
 *   - 偏移 8-9 在 0~339 之间循环，与曲柄旋转吻合
 *   - 每帧都满足「总功率 = 左 + 右」
 *
 * BLE 通知的长度并不保证恰好 11 字节，所以解析按顺序推进；帧被截短时，
 * 尾部读不到的字段保持 0。
 */

#ifndef METER_PROTOCOL_H__
#define METER_PROTOCOL_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief 一帧功率计私有数据的解析结果 */
typedef struct
{
    uint16_t total_w;      /**< 总功率（W） */
    int16_t  left_w;       /**< 左侧功率（W） */
    int16_t  right_w;      /**< 右侧功率（W） */
    uint16_t rpm;          /**< 踏频（rpm）；反踩的负值按 0 处理 */
    int16_t  crank_angle;  /**< 曲柄角度（度） */
    uint8_t  fault;        /**< 错误码 */
} meter_frame_t;

/**
 * @brief 解析一帧私有功率数据
 *
 * @param payload 原始通知负载
 * @param length  负载字节数
 * @param out     解析结果；仅在返回 true 时被写入
 *
 * @retval true  至少解析出总功率（长度 >= 2）
 * @retval false 指针为空或长度不足
 */
bool meter_frame_parse(uint8_t const *payload, size_t length, meter_frame_t *out);

#endif /* METER_PROTOCOL_H__ */
