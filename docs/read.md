✅ MCU 端固件修改完成
修改已直接落到原仓库 type_04_407VE。

修改清单
文件	改动
MDK-ARM/odom_protocol.h	新增 0x20/0x21/0x30/0x31 类型常量、OdomTimeSyncReq/Resp/SetLocalOrigin/Ack payload 结构、OdomUpstreamFrame_t、odom_pack_time_sync_resp/odom_pack_set_origin_ack/odom_parse_upstream 声明
MDK-ARM/odom_protocol.c	实现两个 pack 函数 + odom_parse_upstream 扫描器（找 AA 55 头、校验 ver/payload_len/CRC16-CCITT）
Core/Src/main.c	RX DMA 缓冲 8 → 64 字节；Rcv_IdleCallback 用 NDTR 记录实际长度；Rcv_DealData 三路分发（旧 8 字节协议 / 0xBB 0xCC 复位 / 新 0xAA 0x55 帧）；新增 handle_upstream_frame + send_upstream_response（轮询等待 TX 空闲再 DMA 发送）
0x30 SET_LOCAL_ORIGIN 处理逻辑

1. 解 payload: `float x, y, yaw + uint8 flags + 3 reserved`
2. `flags bit0=reset_xy, bit1=reset_yaw, bit2=reset_encoder`
3. 为兼容旧版上位机，`flags=0` 时 MCU 按 `reset_xy + reset_yaw + reset_encoder` 全部执行
4. `reset_xy`：覆盖 `mpu_data[0].X_tt / Y_tt / REAL_X / REAL_Y`
5. `reset_yaw`：重置 `g_yaw_unwrap.continuous_rad = yaw`，并用当前 `YAW_ANGLE` 锚定 `prev_deg` 避免 unwrap 跳变
6. `reset_encoder`：清零 `AS5048s[].total_angle / delta_dis / cirle`，并把 `last_angle` 锚到当前 `angle`
7. 无论哪种归零，都会清空 odom 运行时积分缓存 `odom_dx_world_acc / odom_dy_world_acc / odom_vel_ticks`
8. 回 `0x31 ACK (acked_seq, result_code=0, event_counter)`
0x20 TIME_SYNC_REQ 处理

1. 取 echoed_host_time_us2. 用 odom_isr_tick * 50us 作为 mcu_time_us3. 回 0x21 TIME_SYNC_RESP
你需要做的
在 MDK-ARM Keil 中重新编译整个工程（type_04_407VE.uvprojx）
烧录到 STM32F407
烧录后重启上位机节点 → 预期看到：
time_sync_locked: true
启动 3 秒内自动归零生效，/odom 发布 x=0, y=0, yaw=0
桥接节点收到 SET_LOCAL_ORIGIN_ACK
风险点
send_upstream_response 在 TIM11 ISR(20kHz) 里轮询等待 TX 空闲；最坏情况 ODOM_STATE 发送中（~3.9ms），轮询 20 万次保护，超时直接放弃这次响应。极端情况下可能漏一次 ACK，但桥接节点重试机制可处理。
ODOM_STATE 发送频率 100Hz，每次约 4ms 占用 TX，留给响应的窗口约 6ms/帧——足够应付 0.5Hz 的 TIME_SYNC + 仅启动一次的