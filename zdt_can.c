/*
 * zdt_can.c - ZDT_X42S CAN(TWAI) 传输封装实现
 *
 * 见 zdt_can.h 顶部的 PDF 勘误说明：每包 payload 第 1 字节都是地址字节。
 */
#include "zdt_can.h"
#include "driver/twai.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "zdt_can";

/* 单例：组件假设一颗芯片一个 TWAI 控制器（esp32c3 即如此） */
static bool s_installed = false;

/* ------------------------------------------------------------------- */
/*  波特率 → TWAI 时序配置映射（手册 4.2 支持的 10 档）                 */
/*                                                                     */
/*  TWAI_TIMING_CONFIG_*KBITS() 是裸花括号初始化器（{...}），不能取地址  */
/*  （&{...} 非法），所以这里用 static const twai_timing_config_t 表，    */
/*  用对应宏做初始化，再传表项地址。                                     */
/* ------------------------------------------------------------------- */
static const twai_timing_config_t t_10k   = TWAI_TIMING_CONFIG_10KBITS();
static const twai_timing_config_t t_20k   = TWAI_TIMING_CONFIG_20KBITS();
static const twai_timing_config_t t_50k   = TWAI_TIMING_CONFIG_50KBITS();
static const twai_timing_config_t t_100k  = TWAI_TIMING_CONFIG_100KBITS();
static const twai_timing_config_t t_125k  = TWAI_TIMING_CONFIG_125KBITS();
static const twai_timing_config_t t_250k  = TWAI_TIMING_CONFIG_250KBITS();
static const twai_timing_config_t t_500k  = TWAI_TIMING_CONFIG_500KBITS();
static const twai_timing_config_t t_800k  = TWAI_TIMING_CONFIG_800KBITS();
static const twai_timing_config_t t_1000k = TWAI_TIMING_CONFIG_1MBITS();

static const twai_timing_config_t *baud_to_timing(uint32_t baud)
{
    switch (baud) {
        case 10000:   return &t_10k;
        case 20000:   return &t_20k;
        case 50000:   return &t_50k;
        case 100000:  return &t_100k;
        case 125000:  return &t_125k;
        case 250000:  return &t_250k;
        case 500000:  return &t_500k;
        case 800000:  return &t_800k;
        case 1000000: return &t_1000k;
        /* 83333（83.333K）TWAI 没有预置宏，按需自行加；这里报参数错 */
        default:      return NULL;
    }
}

/* ------------------------------------------------------------------- */
/*  init / deinit                                                       */
/* ------------------------------------------------------------------- */
esp_err_t zdt_can_init(const zdt_can_config_t *cfg)
{
    if (s_installed) {
        ESP_LOGW(TAG, "already installed, call zdt_can_deinit first");
        return ESP_ERR_INVALID_STATE;
    }

    zdt_can_config_t defaults = {
        .tx_gpio  = ZDT_CAN_DEFAULT_TX_GPIO,
        .rx_gpio  = ZDT_CAN_DEFAULT_RX_GPIO,
        .baudrate = ZDT_CAN_DEFAULT_BAUDRATE,
    };
    if (!cfg) cfg = &defaults;

    const twai_timing_config_t *t = baud_to_timing(cfg->baudrate);
    if (!t) {
        ESP_LOGE(TAG, "unsupported baudrate=%lu", (unsigned long)cfg->baudrate);
        return ESP_ERR_INVALID_ARG;
    }

    /* 扩展帧、accept-all：电机返回也是 (Addr<<8)|Packet 扩展帧 */
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)cfg->tx_gpio, (gpio_num_t)cfg->rx_gpio, TWAI_MODE_NORMAL);
    g.tx_queue_len = 10;
    g.rx_queue_len = 10;
    /* 启用 bus-off / 错误告警，recover_if_busoff 需要 TWAI_ALERT_BUS_RECOVERED */
    g.alerts_enabled = TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_BUS_OFF |
                       TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS;
    const twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, t, &f);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return err;
    }

    s_installed = true;
    ESP_LOGI(TAG, "TWAI up: tx=%d rx=%d baud=%lu",
             cfg->tx_gpio, cfg->rx_gpio, (unsigned long)cfg->baudrate);
    return ESP_OK;
}

esp_err_t zdt_can_deinit(void)
{
    if (!s_installed) return ESP_ERR_INVALID_STATE;
    twai_stop();
    twai_driver_uninstall();
    s_installed = false;
    ESP_LOGI(TAG, "TWAI down");
    return ESP_OK;
}

/* ------------------------------------------------------------------- */
/*  纯函数：拆分（勘误：每包首字节为地址）                              */
/* ------------------------------------------------------------------- */
esp_err_t zdt_can_split(uint8_t addr, const uint8_t *frame, int frame_len,
                        zdt_can_packet_t *out_pkts, int *out_count)
{
    if (!frame || !out_pkts || !out_count) return ESP_ERR_INVALID_ARG;
    if (frame_len < 3) return ESP_ERR_INVALID_ARG;     /* 至少 Addr+Func+0x6B */

    /* 地址字节直接用 frame[0]（libzdt 构造时已写入）。addr 参数仅用于兼容
     * 调用习惯；如果调用方传了不同的 addr，以 frame[0] 为准（CAN ID 用它）。
     * 这允许广播命令（frame[0]=0x00）也能正确发送。 */
    const uint8_t eff_addr = frame[0];
    (void)addr;   /* 不再强制 frame[0]==addr，避免广播场景误报 */

    /*
     * 正确分帧规则（手册 4.2.1 原文 + 真机验证修正）：
     *   - 地址字节 frame[0] **不进 payload**，只编码进 CAN ID = (addr<<8)|pkt
     *   - **每一包** payload 首字节都是 FuncCode (frame[1])——
     *     包括中间包、末包，甚至末包只剩 0x6B 时也要带 func。
     *   - 每包后续字节为参数段连续切片
     *   - 末包最后一个字节强制为 0x6B (固定校验/结束符)
     *   - 中间包 DLC=7 (func + 6 字节参数)
     *   - 末包 DLC = 1(func) + ≤6 字节参数 + 1(0x6B)，保证 ≤8
     *
     * 为什么每包只取 6 字节参数（而不是 7）：
     *   若末包剩余 7 字节参数，func+7+0x6B=9 会超过 TWAI 的 8 字节上限。
     *   真机实测 WritePidEmm(17B, params=14)、StoreAutoRunEmm 等命令
     *   按旧"中间包取7"规则在末包 DLC=9 时 twai_transmit 返回
     *   ESP_ERR_INVALID_ARG。统一每包取 6 即可避免此边界。
     *
     * 真机证据：参数为 0/3/6/14 的命令（短帧、PosMode、WritePidEmm、
     * ReadAllConfigEmm 回读）按本规则全部正常收发。
     */
    const uint8_t func = frame[1];
    const int params_count = frame_len - 3;   /* 减 addr, func, chk(0x6B) */
    int taken = 0;                            /* 已发送的参数字节数 */
    int pkt = 0;
    const int PER_PKT = 6;                    /* 每包参数字节数（func+6=7B 中间包）*/

    while (pkt < ZDT_CAN_MAX_PACKETS) {
        zdt_can_packet_t *p = &out_pkts[pkt];
        p->id = ((uint32_t)eff_addr << 8) | (uint32_t)pkt;
        p->data[0] = func;                    /* 每包都带 func（含末包/纯0x6B） */

        int remaining_params = params_count - taken;
        if (remaining_params <= 0) {
            /* 无参数短命令，或所有参数已发完：末包 = func + 0x6B */
            p->data[1] = 0x6B;
            p->dlc = 2;
            pkt++;
            break;
        } else if (remaining_params <= PER_PKT) {
            /* 末包: func + 剩余参数 + 0x6B (≤1+6+1=8) */
            memcpy(&p->data[1], &frame[2 + taken], (size_t)remaining_params);
            p->data[1 + remaining_params] = 0x6B;
            p->dlc = (uint8_t)(1 + remaining_params + 1);
            taken += remaining_params;
            pkt++;
            break;
        } else {
            /* 中间包: func + PER_PKT 字节参数 (DLC=7) */
            memcpy(&p->data[1], &frame[2 + taken], PER_PKT);
            p->dlc = 1 + PER_PKT;
            taken += PER_PKT;
            pkt++;
        }
    }

    /* 溢出检查:while 循环在 pkt 达到 ZDT_CAN_MAX_PACKETS 时退出,
     * 如果还有参数没塞完,说明命令太长,返回错误而不是静默截断。 */
    if (taken < params_count) {
        ESP_LOGE(TAG, "split overflow: %d params, only %d sent in %d packets",
                 params_count, taken, pkt);
        return ESP_ERR_INVALID_SIZE;
    }
    *out_count = pkt;
    return ESP_OK;
}

/* ------------------------------------------------------------------- */
/*  send / receive                                                      */
/* ------------------------------------------------------------------- */

/* 检查并自愈 Bus-Off（CAN 总线无 ACK 时控制器会进入此态，所有 tx 都会失败）。
 * 真机调试常见触发：收发器没接好、TX/RX 接反、总线单节点无 ACK。
 * 流程：发现 Bus-Off → 发起恢复（等 128×11 个连续隐性位）→ 重新 start。
 * 返回 ESP_OK 表示已恢复可重试，ESP_FAIL 表示恢复失败。 */
static esp_err_t recover_if_busoff(void)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return ESP_FAIL;
    if (st.state != TWAI_STATE_BUS_OFF) return ESP_OK;   /* 没事 */

    ESP_LOGW(TAG, "TWAI Bus-Off detected, initiating recovery");
    /* twai_initiate_recovery 会让控制器进入 RECOVERING 态，等总线空闲后
     * 通过 TWAI_ALERT_BUS_RECOVERED 唤醒；这里同步等待。 */
    esp_err_t err = twai_initiate_recovery();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "initiate_recovery: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    uint32_t alerts = 0;
    err = twai_read_alerts(&alerts, pdMS_TO_TICKS(1000));
    if (err == ESP_OK && (alerts & TWAI_ALERT_BUS_RECOVERED)) {
        err = twai_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "restart after recovery: %s", esp_err_to_name(err));
            return ESP_FAIL;
        }
        ESP_LOGW(TAG, "Bus-Off recovered, TWAI restarted");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "recovery timed out (alerts=0x%lX err=%s)",
             (unsigned long)alerts, esp_err_to_name(err));
    return ESP_FAIL;
}

esp_err_t zdt_can_send(uint8_t addr, const uint8_t *frame, int frame_len,
                       TickType_t timeout)
{
    if (!s_installed) return ESP_ERR_INVALID_STATE;

    zdt_can_packet_t pkts[ZDT_CAN_MAX_PACKETS];
    int count = 0;
    esp_err_t err = zdt_can_split(addr, frame, frame_len, pkts, &count);
    if (err != ESP_OK) return err;

    for (int i = 0; i < count; ++i) {
        twai_message_t msg = {0};
        msg.extd              = 1;                          /* 扩展帧（手册强制） */
        msg.rtr               = 0;
        msg.ss                = 0;
        msg.self              = 0;
        msg.identifier        = pkts[i].id;
        msg.data_length_code  = pkts[i].dlc;
        memcpy(msg.data, pkts[i].data, pkts[i].dlc);

        err = twai_transmit(&msg, timeout);
        if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL || err == ESP_ERR_INVALID_STATE) {
            /* 无 ACK / Bus-Off 的典型征兆：尝试一次自愈后重发本包。
             * ESP_ERR_INVALID_STATE 在 v5.x 上是 Bus-Off 后 twai_transmit 的返回码。 */
            ESP_LOGW(TAG, "tx pkt%d (id=0x%05lX) err=%s, trying bus-off recovery",
                     i, (unsigned long)pkts[i].id, esp_err_to_name(err));
            if (recover_if_busoff() == ESP_OK) {
                err = twai_transmit(&msg, timeout);
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "tx pkt%d (id=0x%05lX dlc=%d) failed: %s",
                     i, (unsigned long)pkts[i].id, pkts[i].dlc,
                     esp_err_to_name(err));
            return err;
        }
        ESP_LOGD(TAG, "tx pkt%d id=0x%05lX dlc=%d", i,
                 (unsigned long)pkts[i].id, pkts[i].dlc);
    }
    return ESP_OK;
}

esp_err_t zdt_can_receive(uint8_t *out_addr, uint8_t *out_buf, int buf_size,
                          int *out_len, TickType_t timeout)
{
    if (!s_installed) return ESP_ERR_INVALID_STATE;
    if (!out_buf || !out_len || buf_size < 4) return ESP_ERR_INVALID_ARG;

    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, timeout);
    if (err != ESP_OK) return err;

    /* 只认扩展帧（电机返回都是扩展帧） */
    if (!msg.extd) return ESP_ERR_NOT_SUPPORTED;

    uint8_t addr = (uint8_t)((msg.identifier >> 8) & 0xFF);
    if (out_addr) *out_addr = addr;

    int n = msg.data_length_code;
    if (n > buf_size) n = buf_size;
    memcpy(out_buf, msg.data, (size_t)n);
    *out_len = n;
    return ESP_OK;
}
