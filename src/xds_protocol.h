/*
 * XDS 功率计私有数据解析 —— 字段定义与 xds_forwarding（DuoDuoJuZi）保持一致
 *
 * 已用实测数据验证的部分：
 *   - 通知负载长度 = 11 字节（实测 29 条通知全部为 11 字节）
 *   - 偏移 6-7 = 踏频（s16 LE）：实测 0x28→40、0x34→52、0x48→72 rpm，精确吻合
 *   - 偏移 8-9 = 曲柄角度（u16 LE）：实测在 0..339 之间循环，符合曲柄旋转
 *
 * 尚未在真实负载下验证的部分：
 *   - 偏移 0-1（总功率）：实测空转时恒为 0，符合"无扭矩即无功率"的物理预期，
 *     但还没有拿到真实骑行数据来确认量纲（是否为瓦特）
 *   - 偏移 2-3 / 4-5（左右功率）：同上
 *
 * 若日后拿到真实负载数据发现字段不符，只需修改本文件的偏移即可。
 */

#ifndef XDS_PROTOCOL_H__
#define XDS_PROTOCOL_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief 一帧 XDS 功率测量的解析结果 */
typedef struct
{
    uint16_t total_power_w;   /**< 总功率（W），来自偏移 0 */
    int16_t  left_power_w;    /**< 左功率（W），来自偏移 2 */
    int16_t  right_power_w;   /**< 右功率（W），来自偏移 4 */
    uint16_t cadence_rpm;     /**< 踏频（rpm），来自偏移 6；负值（反踩）归零 */
    int16_t  angle_deg;       /**< 曲柄角度（度），来自偏移 8 */
    uint8_t  error_code;      /**< 错误码，来自偏移 10 */
} xds_power_measurement_t;

/**
 * @brief 解析一帧 XDS 功率测量负载
 *
 * @param p_payload 来自 0x2A63 特征的原始通知负载
 * @param length    负载字节数
 * @param p_output  解析结果
 *
 * @retval true  解析成功（长度至少 2 字节）
 * @retval false 指针为空或长度不足
 */
bool xds_power_measurement_parse(uint8_t const * p_payload,
                                 size_t          length,
                                 xds_power_measurement_t * p_output);

#endif /* XDS_PROTOCOL_H__ */
