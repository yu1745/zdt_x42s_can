/*
 * zdt_can.h - ZDT_X42S 第二代闭环步进电机 CAN(TWAI) 传输封装
 *
 * 依赖 libzdt（https://github.com/yu1745/libzdt）：libzdt 只负责构建
 * 主机命令的字节流（含地址字节、含 0x6B 校验码），本组件负责把它按
 * ZDT 手册 4.2 章的 CAN 协议规则拆分到 TWAI 扩展帧上发送，并提供
 * 接收重组。
 *
 * =========================================================================
 *  CAN 分帧规则（手册 4.2.1 原文，已真机验证）
 * =========================================================================
 *  对一条 N 字节 UART 帧 frame[0..N-1]（frame[0]=地址, frame[N-1]=0x6B）：
 *
 *    1. CAN 帧类型固定为扩展帧（29 位 ID）。
 *    2. CAN ID = (addr << 8) | packet，packet 从 0 开始递增。
 *    3. **地址字节 frame[0] 不进 payload**，只编码进 CAN ID。
 *    4. payload 从 frame[1]（FuncCode）开始，每包 8 字节，按 packet 拆分。
 *    5. 末包 DLC 可能 < 8。
 *
 *  例 1：UART 帧 "01 36 6B"
 *        → 1 包：CAN ID=0x0100, payload=36 6B (DLC=2)
 *
 *  例 2：UART 帧 "01 FD 01 0F A0 00 00 01 FA 00 00 00 6B"（13B）
 *        → 包0: CAN ID=0x0100, payload=FD 01 0F A0 00 00 01 FA (DLC=8)
 *          包1: CAN ID=0x0101, payload=00 00 00 6B (DLC=4)
 *
 *  真机验证证据（esp32c3 + SN65HVD230 + X42S 电机，500kbps）：
 *    - 发 payload=36 6B → 电机回复 36 01 00 00 00 03 6B（正常位置数据）
 *    - 发 payload=01 36 6B（错误加了地址前缀）→ 电机回复 00 EE 6B（格式错误）
 *    - 发 payload=1F 6B → 电机回复 1F 00 67 23 0A 6B（版本号数据）
 *    - 发 payload=F3 AB 01 00 6B（enable）→ 电机执行，无返回（正常异步命令）
 *
 *  历史：本组件早期版本曾按"每包首字节为地址"实现（误以为是 PDF 勘误），
 *        真机测试证明错误，已回退到 PDF 原文规则。详见 docs/CAN_FRAMING.md。
 *
 * =========================================================================
 *  线程安全：本组件内部不持锁。多任务调用 zdt_can_send / zdt_can_receive
 *  需调用方自行串行化（TWAI 驱动自身的 tx/rx 队列各自线程安全，但同时
 *  并发发送同一逻辑命令的多个包不保证顺序，建议外层加互斥）。
 */
#ifndef ZDT_CAN_H
#define ZDT_CAN_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   /* TickType_t */

#ifdef __cplusplus
extern "C" {
#endif

/* 默认参数 */
#define ZDT_CAN_DEFAULT_BAUDRATE   500000U   /* 手册 4.2 默认 CAN 速率 */
#define ZDT_CAN_DEFAULT_TX_GPIO    0
#define ZDT_CAN_DEFAULT_RX_GPIO    2

typedef struct {
    int      tx_gpio;     /* CAN TX 引脚（GPIO 矩阵任意可用脚） */
    int      rx_gpio;     /* CAN RX 引脚 */
    uint32_t baudrate;    /* bit/s，默认 500000 */
} zdt_can_config_t;

/* 单包拆分结果（host 测试与发送逻辑共用） */
typedef struct {
    uint32_t id;          /* 扩展帧 ID = (addr << 8) | packet */
    uint8_t  dlc;         /* 1..8 */
    uint8_t  data[8];     /* data[0] = addr（勘误：每包都带） */
} zdt_can_packet_t;

/* 初始化 TWAI 控制器并启动。重复调用需先 zdt_can_deinit()。
 * cfg 为 NULL 时使用默认（TX=0/RX=2/500k）。 */
esp_err_t zdt_can_init(const zdt_can_config_t *cfg);

/* 停止并卸载 TWAI 控制器。 */
esp_err_t zdt_can_deinit(void);

/* ---------------------------------------------------------------
 * 纯函数：把一条 libzdt 字节流拆成若干 CAN 包（不调用 TWAI）。
 * host 测试与 zdt_can_send 共用，保证分帧规则可独立验证。
 *
 * addr       = 电机地址（通常 = frame[0]）
 * frame      = libzdt zdtBuildXxxCmd 写出的完整缓冲（含地址字节、含 0x6B）
 * frame_len  = 字节数
 * out_pkts   = 调用者提供的数组，容量 >= ZDT_CAN_MAX_PACKETS
 * out_count  = 输出实际写入的包数
 * 返回 ESP_OK / ESP_ERR_INVALID_ARG。 */
#define ZDT_CAN_MAX_PACKETS   8     /* 37 字节最长命令 → ceil(36/8) = 5 包，留余量 */
esp_err_t zdt_can_split(uint8_t addr, const uint8_t *frame, int frame_len,
                        zdt_can_packet_t *out_pkts, int *out_count);

/* 核心：把 libzdt 字节流按手册 4.2.1 规则拆分发送。
 * frame = zdtBuildXxxCmd 写出的完整缓冲（含地址字节、含 0x6B）
 * frame_len = 返回字节数
 * addr 通常传 frame[0]，单独给便于广播场景校验。
 * 每包都 prepend 地址字节，CAN ID = (addr<<8)|packet，扩展帧。 */
esp_err_t zdt_can_send(uint8_t addr, const uint8_t *frame, int frame_len,
                       TickType_t timeout);

/* 可选：阻塞接收一帧电机返回（按 (Addr<<8)|Packet 重组）。
 * 电机返回格式见手册 4.2.2：第 2 字节是状态码 02/E2/EE/9F。
 * 当前实现：等同一 ID 的 0 号包到达，把 data[0..dlc-1] 写入 out_buf
 * （data[0]=addr 也包含在内），out_len 为写入字节数。
 * 多包返回重组暂未实现（绝大多数返回是单包短帧）。
 *
 * 注意：如果注册了 RX 回调（zdt_can_register_rx_callback），帧会被
 * 直接交给回调，不再进内部队列，此时 zdt_can_receive 读不到帧。 */
esp_err_t zdt_can_receive(uint8_t *out_addr, uint8_t *out_buf, int buf_size,
                          int *out_len, TickType_t timeout);

/* 可选：注册 RX 回调（回调优先，队列兜底）。
 * 注册后，每收到一帧扩展帧，ISR 直接调用 cb（在 ISR 上下文！cb 必须 IRAM_SAFE、
 * 不能阻塞、不能做重活）。帧不再进内部队列，zdt_can_receive 读不到。
 * 传 cb=NULL 取消注册，帧恢复进内部队列。
 *
 * 用法：
 *   - FluidNC 等高频场景：注册回调，回调里 xQueueSendFromISR 到自己的队列，
 *     update() 里 drain 解析。零轮询延迟。
 *   - command_test 等简单场景：不注册，用 zdt_can_receive 从内部队列读。
 *
 * cb 参数：addr=电机地址(从 CAN ID 高字节解出)，data/len=payload(DLC<=8)，
 *          user=注册时传入的 user_data。 */
typedef void (*zdt_can_rx_cb_t)(uint8_t addr, const uint8_t *data, int len, void *user);
esp_err_t zdt_can_register_rx_callback(zdt_can_rx_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif

#endif /* ZDT_CAN_H */
