/*
 * zdt_can.c - ZDT_X42S CAN(TWAI) 传输封装实现
 *
 * IDF v5.5 新 TWAI API (esp_driver_twai 组件):
 *   - twai_new_node_onchip / twai_node_enable / twai_node_disable / twai_node_delete
 *   - twai_node_transmit (发送)
 *   - 接收: on_rx_done 回调里 twai_node_receive_from_isr → FreeRTOS 队列
 *
 * 发送可靠性（关键设计）:
 *   新 TWAI API 的 twai_node_transmit 存 frame 指针（不拷贝），多包命令的后续包
 *   由 ISR 从 tx 队列异步发送。若 frame/buffer 在栈上，send 返回后栈帧销毁，
 *   ISR 访问野指针 → 崩溃。
 *
 *   解决：用 s_tx_sem 二值信号量同步——每包 transmit 后等 on_tx_done 回调确认
 *   硬件发完，才发下一包。这样 buffer 可以用栈（整个 zdt_can_send 期间有效），
 *   且 s_tx_mutex 串行化整个发送，多任务并发安全。
 *
 * 接收（回调优先 + 队列兜底）:
 *   注册了用户回调 → ISR 直接调它，帧不进队列。
 *   未注册 → 帧进内部 RX 队列，zdt_can_receive 从队列读。
 *
 * 分帧规则见 zdt_can.h / docs/CAN_FRAMING.md：
 *   每包重复 FuncCode，每包取 6 字节参数（末包 func+<=6参数+0x6B，DLC<=8）。
 */
#include "zdt_can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "zdt_can";

/* 单例：组件假设一颗芯片一个 TWAI 控制器（esp32c3 即如此） */
static bool s_installed = false;
static twai_node_handle_t s_node = NULL;

/* ---------- 发送同步 ---------- */
/* s_tx_mutex: 串行化整个 zdt_can_send，多任务并发安全 */
static SemaphoreHandle_t s_tx_mutex = NULL;
/* s_tx_sem: 二值信号量，on_tx_done 回调 give，每包 transmit 后 take 等发完 */
static SemaphoreHandle_t s_tx_sem = NULL;

/* ---------- 接收 ---------- */
#define RX_QUEUE_LEN   16
static QueueHandle_t s_rx_queue = NULL;

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} rx_item_t;

/* 用户回调（回调优先）：注册后 ISR 直接调它，帧不进队列 */
static zdt_can_rx_cb_t s_user_cb = NULL;
static void *s_user_ctx = NULL;

/* ---------- ISR 回调 ---------- */

/* on_tx_done: 硬件发完一帧（成功或失败）时触发。give 信号量通知 send 线程。 */
static IRAM_ATTR bool on_tx_done_cb(twai_node_handle_t handle,
                                    const twai_tx_done_event_data_t *edata,
                                    void *user_ctx)
{
    (void)handle; (void)edata; (void)user_ctx;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_tx_sem, &hpw);
    return hpw == pdTRUE;
}

/* on_rx_done: 取帧 → 注册了回调就调回调，否则推队列。 */
static IRAM_ATTR bool on_rx_done_cb(twai_node_handle_t handle,
                                    const twai_rx_done_event_data_t *edata,
                                    void *user_ctx)
{
    (void)edata; (void)user_ctx;
    uint8_t buf[8];
    twai_frame_t rx_frame = {
        .buffer = buf,
        .buffer_len = sizeof(buf),
    };
    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
        return false;
    }
    if (!rx_frame.header.ide) {       /* 只收扩展帧 */
        return false;
    }
    int n = rx_frame.header.dlc;
    if (n > 8) n = 8;
    uint8_t addr = (uint8_t)((rx_frame.header.id >> 8) & 0xFF);

    if (s_user_cb) {
        s_user_cb(addr, buf, n, s_user_ctx);
        return false;
    }

    rx_item_t item = {0};
    item.id  = rx_frame.header.id;
    item.dlc = (uint8_t)n;
    memcpy(item.data, buf, (size_t)n);
    BaseType_t hpw = pdFALSE;
    if (xQueueSendFromISR(s_rx_queue, &item, &hpw) != pdTRUE) {
        rx_item_t dummy;
        xQueueReceiveFromISR(s_rx_queue, &dummy, &hpw);
        xQueueSendFromISR(s_rx_queue, &item, &hpw);
    }
    return hpw == pdTRUE;
}

/* ---------- 公共 API: 注册回调 ---------- */
esp_err_t zdt_can_register_rx_callback(zdt_can_rx_cb_t cb, void *user)
{
    if (!s_installed) return ESP_ERR_INVALID_STATE;
    /* 调用方应在 init 后、通信前注册，通信过程中不改（ISR 可能在读 s_user_cb）。
     * 指针写入在 RISC-V 上原子，但读改写不保证，故不支持并发注册。 */
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

    s_tx_mutex = xSemaphoreCreateMutex();
    s_tx_sem   = xSemaphoreCreateBinary();
    s_rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(rx_item_t));
    if (!s_tx_mutex || !s_tx_sem || !s_rx_queue) {
        ESP_LOGE(TAG, "sync primitive create failed");
        goto fail;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx = (gpio_num_t)cfg->tx_gpio,
        .io_cfg.rx = (gpio_num_t)cfg->rx_gpio,
        .io_cfg.quanta_clk_out = -1,
        .io_cfg.bus_off_indicator = -1,
        .bit_timing.bitrate = cfg->baudrate,
        .tx_queue_depth = 5,
        .intr_priority = 0,
        .flags.enable_self_test = 0,
        .flags.enable_loopback = 0,
        .flags.enable_listen_only = 0,
        .flags.no_receive_rtr = 1,
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(err));
        goto fail;
    }

    twai_event_callbacks_t cbs = {
        .on_rx_done    = on_rx_done_cb,
        .on_tx_done    = on_tx_done_cb,
        .on_state_change = NULL,
        .on_error      = NULL,
    };
    err = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register callbacks failed: %s", esp_err_to_name(err));
        twai_node_delete(s_node);
        goto fail;
    }

    err = twai_node_enable(s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(err));
        twai_node_delete(s_node);
        goto fail;
    }

    s_installed = true;
    ESP_LOGI(TAG, "TWAI up: tx=%d rx=%d baud=%lu",
             cfg->tx_gpio, cfg->rx_gpio, (unsigned long)cfg->baudrate);
    return ESP_OK;

fail:
    if (s_tx_mutex) { vSemaphoreDelete(s_tx_mutex); s_tx_mutex = NULL; }
    if (s_tx_sem)   { vSemaphoreDelete(s_tx_sem);   s_tx_sem = NULL; }
    if (s_rx_queue) { vQueueDelete(s_rx_queue);     s_rx_queue = NULL; }
    s_node = NULL;
    return ESP_FAIL;
}

esp_err_t zdt_can_deinit(void)
{
    if (!s_installed) return ESP_ERR_INVALID_STATE;
    twai_node_disable(s_node);
    twai_node_delete(s_node);
    s_node = NULL;
    vSemaphoreDelete(s_tx_mutex); s_tx_mutex = NULL;
    vSemaphoreDelete(s_tx_sem);   s_tx_sem = NULL;
    vQueueDelete(s_rx_queue);     s_rx_queue = NULL;
    s_user_cb = NULL;
    s_user_ctx = NULL;
    s_installed = false;
    ESP_LOGI(TAG, "TWAI down");
    return ESP_OK;
}

/* ------------------------------------------------------------------- */
/*  纯函数：拆分（每包首字节为 func）                                    */
/* ------------------------------------------------------------------- */
esp_err_t zdt_can_split(uint8_t addr, const uint8_t *frame, int frame_len,
                        zdt_can_packet_t *out_pkts, int *out_count)
{
    if (!frame || !out_pkts || !out_count) return ESP_ERR_INVALID_ARG;
    if (frame_len < 3) return ESP_ERR_INVALID_ARG;

    const uint8_t eff_addr = frame[0];
    (void)addr;

    const uint8_t func = frame[1];
    const int params_count = frame_len - 3;
    int taken = 0;
    int pkt = 0;
    const int PER_PKT = 6;

    while (pkt < ZDT_CAN_MAX_PACKETS) {
        zdt_can_packet_t *p = &out_pkts[pkt];
        p->id = ((uint32_t)eff_addr << 8) | (uint32_t)pkt;
        p->data[0] = func;

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

/* Bus-Off 检测 + 恢复。调用方持有 s_tx_mutex。 */
static esp_err_t recover_if_busoff(void)
{
    twai_node_status_t st;
    twai_node_record_t rec;
    if (twai_node_get_info(s_node, &st, &rec) != ESP_OK) return ESP_FAIL;
    if (st.state != TWAI_ERROR_BUS_OFF) return ESP_OK;

    ESP_LOGW(TAG, "TWAI Bus-Off detected, initiating recovery");
    esp_err_t err = twai_node_recover(s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_recover: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
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

    /* 串行化发送：多任务并发时一次只允许一个 zdt_can_send */
    if (xSemaphoreTake(s_tx_mutex, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int timeout_ms = (timeout == portMAX_DELAY) ? -1 : (int)(timeout * portTICK_PERIOD_MS);

    for (int i = 0; i < count; ++i) {
        /* buffer 在栈上：因为我们等每包 on_tx_done 确认硬件发完才发下一包，
         * 且整个循环在 mutex 保护下，zdt_can_send 返回前所有包都已发完。
         * 所以栈帧在整个发送期间有效，ISR 不会访问到失效指针。 */
        twai_frame_t tx_frame = {
            .header.id   = pkts[i].id,
            .header.dlc  = pkts[i].dlc,
            .header.ide  = 1,                  /* 扩展帧 */
            .header.rtr  = 0,
            .header.fdf  = 0,
            .header.brs  = 0,
            .header.esi  = 0,
            .buffer      = pkts[i].data,       /* 指向栈上 pkts，发送期间有效 */
            .buffer_len  = pkts[i].dlc,
        };

        err = twai_node_transmit(s_node, &tx_frame, timeout_ms);
        if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL || err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "tx pkt%d (id=0x%05lX) err=%s, trying bus-off recovery",
                     i, (unsigned long)pkts[i].id, esp_err_to_name(err));
            if (recover_if_busoff() == ESP_OK) {
                err = twai_node_transmit(s_node, &tx_frame, timeout_ms);
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "tx pkt%d (id=0x%05lX dlc=%d) failed: %s",
                     i, (unsigned long)pkts[i].id, pkts[i].dlc,
                     esp_err_to_name(err));
            xSemaphoreGive(s_tx_mutex);
            return err;
        }
        /* 等这一帧硬件发完（on_tx_done 回调 give s_tx_sem）。
         * 不等的话连续 transmit 会让后续包进 tx 队列，buffer 在栈上不安全。 */
        if (xSemaphoreTake(s_tx_sem, timeout) != pdTRUE) {
            ESP_LOGE(TAG, "tx pkt%d wait tx_done timeout", i);
            xSemaphoreGive(s_tx_mutex);
            return ESP_ERR_TIMEOUT;
        }
        ESP_LOGD(TAG, "tx pkt%d id=0x%05lX dlc=%d done",
                 i, (unsigned long)pkts[i].id, pkts[i].dlc);
    }

    xSemaphoreGive(s_tx_mutex);
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

    uint8_t addr = (uint8_t)((item.id >> 8) & 0xFF);
    if (out_addr) *out_addr = addr;

    int n = item.dlc;
    if (n > buf_size) n = buf_size;
    memcpy(out_buf, item.data, (size_t)n);
    *out_len = n;
    return ESP_OK;
}
