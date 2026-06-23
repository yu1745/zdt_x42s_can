/*
 * zdt_can.c - ZDT_X42S CAN(TWAI) 传输封装实现
 *
 * IDF v5.5 新 TWAI API (esp_driver_twai 组件):
 *   - twai_new_node_onchip / twai_node_enable / twai_node_disable / twai_node_delete
 *   - twai_node_transmit (发送)
 *   - 接收: on_rx_done 回调里 twai_node_receive_from_isr → FreeRTOS 队列
 *
 * 分帧规则见 zdt_can.h / docs/CAN_FRAMING.md：
 *   每包重复 FuncCode，每包取 6 字节参数（末包 func+<=6参数+0x6B，DLC<=8）。
 */
#include "zdt_can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "zdt_can";

/* 单例：组件假设一颗芯片一个 TWAI 控制器（esp32c3 即如此） */
static bool s_installed = false;
static twai_node_handle_t s_node = NULL;

/* RX 队列：回调把收到的帧推到这里，上层 zdt_can_receive 从队列读 */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define RX_QUEUE_LEN   16
static QueueHandle_t s_rx_queue = NULL;

/* 队列元素：CAN ID + DLC + data[8]（zdt_can_receive 对外格式不变） */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} rx_item_t;

/* 用户回调（回调优先）：注册后 ISR 直接调它，帧不进队列 */
static zdt_can_rx_cb_t s_user_cb = NULL;
static void *s_user_ctx = NULL;

/* on_rx_done 回调（ISR 上下文）：取帧 → 注册了回调就调回调，否则推队列。 */
static IRAM_ATTR bool on_rx_done_cb(twai_node_handle_t handle,
                                    const twai_rx_done_event_data_t *edata,
                                    void *user_ctx)
{
    (void)user_ctx;
    uint8_t buf[8];
    twai_frame_t rx_frame = {
        .buffer = buf,
        .buffer_len = sizeof(buf),
    };
    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
        return false;
    }
    /* 只收扩展帧（电机返回都是扩展帧） */
    if (!rx_frame.header.ide) {
        return false;
    }
    int n = rx_frame.header.dlc;
    if (n > 8) n = 8;
    uint8_t addr = (uint8_t)((rx_frame.header.id >> 8) & 0xFF);

    /* 回调优先：注册了就直接交给用户，不进队列 */
    if (s_user_cb) {
        s_user_cb(addr, buf, n, s_user_ctx);
        return false;
    }

    /* 队列兜底：没注册回调才推队列 */
    rx_item_t item = {0};
    item.id  = rx_frame.header.id;
    item.dlc = (uint8_t)n;
    memcpy(item.data, buf, (size_t)n);
    if (xQueueSendFromISR(s_rx_queue, &item, NULL) != pdTRUE) {
        rx_item_t dummy;
        xQueueReceiveFromISR(s_rx_queue, &dummy, NULL);
        xQueueSendFromISR(s_rx_queue, &item, NULL);
    }
    return false;
}

/* 注册/取消 RX 回调（线程安全：调用方应在 init 后、通信前注册） */
esp_err_t zdt_can_register_rx_callback(zdt_can_rx_cb_t cb, void *user)
{
    if (!s_installed) return ESP_ERR_INVALID_STATE;
    /* 注意：这里没加锁。s_user_cb 是指针写入，原子性够；
     * 但调用方应避免在通信进行中切换，以免 ISR 读到中间态。
     * 建议在 init 后立即注册，通信过程中不改。 */
    s_user_cb = cb;
    s_user_ctx = user;
    return ESP_OK;
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

    /* RX 队列 */
    s_rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(rx_item_t));
    if (!s_rx_queue) {
        ESP_LOGE(TAG, "rx queue create failed");
        return ESP_ERR_NO_MEM;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx = (gpio_num_t)cfg->tx_gpio,
        .io_cfg.rx = (gpio_num_t)cfg->rx_gpio,
        .io_cfg.quanta_clk_out = -1,
        .io_cfg.bus_off_indicator = -1,
        .bit_timing.bitrate = cfg->baudrate,   /* 采样点用默认 */
        .tx_queue_depth = 10,
        .intr_priority = 0,
        .flags.enable_self_test = 0,
        .flags.enable_loopback = 0,
        .flags.enable_listen_only = 0,
        .flags.no_receive_rtr = 1,
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(err));
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
        return err;
    }

    /* 注册 RX 回调 */
    twai_event_callbacks_t cbs = {
        .on_rx_done = on_rx_done_cb,
        .on_tx_done = NULL,
        .on_state_change = NULL,
        .on_error = NULL,
    };
    err = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register callbacks failed: %s", esp_err_to_name(err));
        twai_node_delete(s_node);
        s_node = NULL;
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
        return err;
    }

    err = twai_node_enable(s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(err));
        twai_node_delete(s_node);
        s_node = NULL;
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
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
    twai_node_disable(s_node);
    twai_node_delete(s_node);
    s_node = NULL;
    vQueueDelete(s_rx_queue);
    s_rx_queue = NULL;
    s_installed = false;
    ESP_LOGI(TAG, "TWAI down");
    return ESP_OK;
}

/* ------------------------------------------------------------------- */
/*  纯函数：拆分（勘误：每包首字节为 func）                             */
/* ------------------------------------------------------------------- */
esp_err_t zdt_can_split(uint8_t addr, const uint8_t *frame, int frame_len,
                        zdt_can_packet_t *out_pkts, int *out_count)
{
    if (!frame || !out_pkts || !out_count) return ESP_ERR_INVALID_ARG;
    if (frame_len < 3) return ESP_ERR_INVALID_ARG;     /* 至少 Addr+Func+0x6B */

    const uint8_t eff_addr = frame[0];
    (void)addr;

    const uint8_t func = frame[1];
    const int params_count = frame_len - 3;   /* 减 addr, func, chk(0x6B) */
    int taken = 0;
    int pkt = 0;
    const int PER_PKT = 6;                    /* 每包参数字节数（func+6=7B 中间包）*/

    while (pkt < ZDT_CAN_MAX_PACKETS) {
        zdt_can_packet_t *p = &out_pkts[pkt];
        p->id = ((uint32_t)eff_addr << 8) | (uint32_t)pkt;
        p->data[0] = func;                    /* 每包都带 func（含末包/纯0x6B） */

        int remaining_params = params_count - taken;
        if (remaining_params <= 0) {
            p->data[1] = 0x6B;
            p->dlc = 2;
            pkt++;
            break;
        } else if (remaining_params <= PER_PKT) {
            memcpy(&p->data[1], &frame[2 + taken], (size_t)remaining_params);
            p->data[1 + remaining_params] = 0x6B;
            p->dlc = (uint8_t)(1 + remaining_params + 1);
            taken += remaining_params;
            pkt++;
            break;
        } else {
            memcpy(&p->data[1], &frame[2 + taken], PER_PKT);
            p->dlc = 1 + PER_PKT;
            taken += PER_PKT;
            pkt++;
        }
    }

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

/* Bus-Off 检测 + 恢复（新 API：twai_node_recover + twai_node_get_info）。
 * 返回 ESP_OK 表示已恢复可重试，ESP_FAIL 表示恢复失败。 */
static esp_err_t recover_if_busoff(void)
{
    twai_node_status_t st;
    twai_node_record_t rec;
    if (twai_node_get_info(s_node, &st, &rec) != ESP_OK) return ESP_FAIL;
    /* 旧 API 有 TWAI_STATE_BUS_OFF 枚举；新 API 用 twai_error_state_t。
     * bus-off 对应 TWAI_ERROR_BUS_OFF。靠 on_state_change 回调更准，
     * 这里做简化：检查 error_state 是否为 bus-off。 */
    if (st.state != TWAI_ERROR_BUS_OFF) return ESP_OK;

    ESP_LOGW(TAG, "TWAI Bus-Off detected, initiating recovery");
    esp_err_t err = twai_node_recover(s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_recover: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    /* 等恢复完成：轮询 state 变回正常（最多 2s） */
    for (int i = 0; i < 20; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (twai_node_get_info(s_node, &st, &rec) != ESP_OK) return ESP_FAIL;
        if (st.state != TWAI_ERROR_BUS_OFF) {
            ESP_LOGW(TAG, "Bus-Off recovered");
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Bus-Off recovery timed out");
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

    int timeout_ms = (timeout == portMAX_DELAY) ? -1 : (int)(timeout * portTICK_PERIOD_MS);

    /* 新 TWAI API 的 twai_node_transmit 内部存 frame 指针（不是拷贝），
     * 多包命令时后续包会被入 tx_mount_queue，ISR 稍后从队列取出异步发送。
     * 如果 frame/buffer 在栈上，zdt_can_send 返回后栈帧销毁，ISR 访问到野指针 → 崩溃。
     * 所以必须用堆分配，buffer 在 deinit 前一直有效。
     * 用静态池避免每次 malloc/free（TWAI 帧短，池大小 = MAX_PACKETS）。 */
    static uint8_t  s_tx_buf[ZDT_CAN_MAX_PACKETS][8];     /* 持久 data buffer 池 */
    static twai_frame_t s_tx_frames[ZDT_CAN_MAX_PACKETS]; /* 持久 frame 池 */
    for (int i = 0; i < count; ++i) {
        memcpy(s_tx_buf[i], pkts[i].data, pkts[i].dlc);
        s_tx_frames[i].header.id   = pkts[i].id;
        s_tx_frames[i].header.dlc  = pkts[i].dlc;
        s_tx_frames[i].header.ide  = 1;                    /* 扩展帧 */
        s_tx_frames[i].header.rtr  = 0;
        s_tx_frames[i].header.fdf  = 0;
        s_tx_frames[i].header.brs  = 0;
        s_tx_frames[i].header.esi  = 0;
        s_tx_frames[i].buffer      = s_tx_buf[i];
        s_tx_frames[i].buffer_len  = pkts[i].dlc;
    }

    for (int i = 0; i < count; ++i) {
        err = twai_node_transmit(s_node, &s_tx_frames[i], timeout_ms);
        if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL || err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "tx pkt%d (id=0x%05lX) err=%s, trying bus-off recovery",
                     i, (unsigned long)pkts[i].id, esp_err_to_name(err));
            if (recover_if_busoff() == ESP_OK) {
                err = twai_node_transmit(s_node, &s_tx_frames[i], timeout_ms);
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

    rx_item_t item;
    if (xQueueReceive(s_rx_queue, &item, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 只认扩展帧 */
    uint8_t addr = (uint8_t)((item.id >> 8) & 0xFF);
    if (out_addr) *out_addr = addr;

    int n = item.dlc;
    if (n > buf_size) n = buf_size;
    memcpy(out_buf, item.data, (size_t)n);
    *out_len = n;
    return ESP_OK;
}
