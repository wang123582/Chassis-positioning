#ifndef ODOM_PROTOCOL_H
#define ODOM_PROTOCOL_H

#include <stdint.h>

/* ---------- 编译宏：协议模式切换 ---------- */
/* 1 = 二进制 ODOM 协议, 0 = VOFA CSV 调试模式 */
#ifndef ODOM_BINARY_MODE
#define ODOM_BINARY_MODE 1
#endif

/* ---------- 帧头 & 版本 ---------- */
#define ODOM_FRAME_HEADER_0  0xAA
#define ODOM_FRAME_HEADER_1  0x55
#define ODOM_FRAME_VERSION   0x01

/* ---------- 消息类型 ---------- */
#define ODOM_MSG_POSE    0x01   /* 24 bytes payload */
#define ODOM_MSG_STATE   0x02   /* 36 bytes payload */
#define ODOM_MSG_STATUS  0x10   /* 12 bytes payload */

/* ---------- 帧结构常量 ---------- */
#define ODOM_FRAME_OVERHEAD  9  /* header(2)+ver(1)+type(1)+seq(1)+len(2)+crc(2) */
#define ODOM_POSE_PAYLOAD_LEN   24
#define ODOM_STATE_PAYLOAD_LEN  36
#define ODOM_STATUS_PAYLOAD_LEN 12

#define ODOM_POSE_FRAME_LEN   (ODOM_FRAME_OVERHEAD + ODOM_POSE_PAYLOAD_LEN)   /* 33 */
#define ODOM_STATE_FRAME_LEN  (ODOM_FRAME_OVERHEAD + ODOM_STATE_PAYLOAD_LEN)  /* 45 */
#define ODOM_STATUS_FRAME_LEN (ODOM_FRAME_OVERHEAD + ODOM_STATUS_PAYLOAD_LEN) /* 21 */

/* ---------- status_bits 定义 ---------- */
#define ODOM_STATUS_ENC_VALID    (1u << 0)
#define ODOM_STATUS_IMU_VALID    (1u << 1)
#define ODOM_STATUS_YAW_VALID    (1u << 2)
#define ODOM_STATUS_POS_VALID    (1u << 3)
#define ODOM_STATUS_VEL_VALID    (1u << 4)
#define ODOM_STATUS_RELOCATE     (1u << 5)
#define ODOM_STATUS_TIME_SYNC    (1u << 6)
#define ODOM_STATUS_DEGRADED     (1u << 7)

/* ---------- quality 等级 ---------- */
#define ODOM_QUALITY_UNAVAIL  0
#define ODOM_QUALITY_DEGRADED 1
#define ODOM_QUALITY_NORMAL   2
#define ODOM_QUALITY_HIGH     3

/* ---------- Payload 结构体 (packed) ---------- */
#pragma pack(push, 1)

typedef struct {
    uint64_t t_sample_us;
    float    x;
    float    y;
    float    yaw;
    uint16_t status_bits;
    uint8_t  quality;
    uint8_t  reserved;
} OdomPosePayload_t;

typedef struct {
    uint64_t t_sample_us;
    float    x;
    float    y;
    float    yaw;
    float    vx;
    float    vy;
    float    wz;
    uint16_t status_bits;
    uint8_t  quality;
    uint8_t  reserved;
} OdomStatePayload_t;

typedef struct {
    uint64_t t_sample_us;
    uint16_t status_bits;
    uint8_t  quality;
    uint8_t  link_state;
} OdomStatusPayload_t;

#pragma pack(pop)

/* ---------- YAW unwrap 状态 ---------- */
typedef struct {
    float prev_deg;       /* 上一次 YAW_ANGLE (度, [-180,+180]) */
    float continuous_rad; /* 连续弧度输出 */
    uint8_t initialized;
} YawUnwrap_t;

/* ---------- 函数声明 ---------- */

/* CRC-16/CCITT (初始值 0xFFFF, 多项式 0x1021) */
uint16_t odom_crc16(const uint8_t *data, uint16_t len);

/* YAW unwrap：输入度 [-180,+180]，输出连续弧度 */
float odom_yaw_unwrap(YawUnwrap_t *state, float yaw_deg);

/* 组帧：返回帧总长度，buf 需 >= ODOM_STATE_FRAME_LEN */
uint16_t odom_pack_pose(uint8_t *buf, uint8_t seq, const OdomPosePayload_t *payload);
uint16_t odom_pack_state(uint8_t *buf, uint8_t seq, const OdomStatePayload_t *payload);
uint16_t odom_pack_status(uint8_t *buf, uint8_t seq, const OdomStatusPayload_t *payload);

/* 全局 ISR tick 计数器 (每 TIM11 中断 +1, 50us/tick) */
extern volatile uint64_t odom_isr_tick;

/* 全局 unwrap 状态 */
extern YawUnwrap_t g_yaw_unwrap;

#endif /* ODOM_PROTOCOL_H */
