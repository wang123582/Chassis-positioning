#include "odom_protocol.h"
#include <string.h>

/* ---------- 全局变量 ---------- */
volatile uint64_t odom_isr_tick = 0;
YawUnwrap_t g_yaw_unwrap = {0.0f, 0.0f, 0};

/* ---------- CRC-16/CCITT ---------- */
/* 多项式 0x1021, 初始值 0xFFFF, 无反转 */
uint16_t odom_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ---------- YAW unwrap ---------- */
float odom_yaw_unwrap(YawUnwrap_t *state, float yaw_deg)
{
    if (!state->initialized) {
        state->prev_deg = yaw_deg;
        state->continuous_rad = yaw_deg * 0.0174532925f;
        state->initialized = 1;
        return state->continuous_rad;
    }

    float delta = yaw_deg - state->prev_deg;
    /* 处理 ±180° 翻转 */
    if (delta > 180.0f)
        delta -= 360.0f;
    else if (delta < -180.0f)
        delta += 360.0f;

    state->continuous_rad += delta * 0.0174532925f;
    state->prev_deg = yaw_deg;
    return state->continuous_rad;
}

/* ---------- 内部：写帧头 + payload + CRC ---------- */
static uint16_t odom_pack_frame(uint8_t *buf, uint8_t msg_type, uint8_t seq,
                                const void *payload, uint16_t payload_len)
{
    uint16_t idx = 0;

    /* Header */
    buf[idx++] = ODOM_FRAME_HEADER_0;
    buf[idx++] = ODOM_FRAME_HEADER_1;
    /* Version */
    buf[idx++] = ODOM_FRAME_VERSION;
    /* Type */
    buf[idx++] = msg_type;
    /* Sequence */
    buf[idx++] = seq;
    /* Length (little-endian) */
    buf[idx++] = (uint8_t)(payload_len & 0xFF);
    buf[idx++] = (uint8_t)((payload_len >> 8) & 0xFF);
    /* Payload */
    memcpy(&buf[idx], payload, payload_len);
    idx += payload_len;
    /* CRC16 over [Version .. Payload] */
    uint16_t crc = odom_crc16(&buf[2], idx - 2);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    return idx;
}

/* ---------- 公开打包接口 ---------- */
uint16_t odom_pack_pose(uint8_t *buf, uint8_t seq, const OdomPosePayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_POSE, seq, payload, ODOM_POSE_PAYLOAD_LEN);
}

uint16_t odom_pack_state(uint8_t *buf, uint8_t seq, const OdomStatePayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_STATE, seq, payload, ODOM_STATE_PAYLOAD_LEN);
}

uint16_t odom_pack_status(uint8_t *buf, uint8_t seq, const OdomStatusPayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_STATUS, seq, payload, ODOM_STATUS_PAYLOAD_LEN);
}
