/*
 * esp32c3_motor_test main.c — 真机测试流程
 *
 * 硬件接线（详见仓库 README）：
 *   esp32c3 GPIO[TX] ── TXD ── CAN 收发器（SN65HVD230 / TJA1050）── CAN_H/L ── 电机 R/A/H & T/B/L
 *   esp32c3 GPIO[RX] ── RXD ── CAN 收发器
 *   共地，总线两端各 120Ω 终端电阻。
 *
 * 本板实测正确接线（GPIO6/7）：GPIO7=TX, GPIO6=RX
 *
 * 编译烧录：
 *   . C:\Users\wangyu\esp\v5.5\esp-idf\fast_export.ps1
 *   cd examples\esp32c3_motor_test
 *   idf.py set-target esp32c3
 *   idf.py -DZDT_CAN_TX_GPIO=7 -DZDT_CAN_RX_GPIO=6 build flash monitor
 *   （或 menuconfig 改 ZDT_CAN_TX_GPIO=7 / ZDT_CAN_RX_GPIO=6）
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "libzdt.h"
#include "zdt_can.h"

static const char *TAG = "main";

#define MOTOR_ADDR    CONFIG_ZDT_MOTOR_ADDR
#define CAN_TX        CONFIG_ZDT_CAN_TX_GPIO
#define CAN_RX        CONFIG_ZDT_CAN_RX_GPIO
#define CAN_BAUD      CONFIG_ZDT_CAN_BAUDRATE

static void log_frame(const char *tag, const uint8_t *frame, int n)
{
    char hex[160];
    int p = 0;
    for (int i = 0; i < n && p < (int)sizeof(hex) - 4; ++i)
        p += snprintf(hex + p, sizeof(hex) - p, "%02X ", frame[i]);
    ESP_LOGI(TAG, "  TX %s (%dB): %s", tag, n, hex);
}

static void try_rx(uint32_t wait_ms)
{
    uint8_t raddr, rbuf[8];
    int rlen = 0;
    esp_err_t rerr = zdt_can_receive(&raddr, rbuf, sizeof rbuf, &rlen,
                                     pdMS_TO_TICKS(wait_ms));
    if (rerr == ESP_OK) {
        char hex[32]; int p = 0;
        for (int i = 0; i < rlen && p < (int)sizeof(hex) - 4; ++i)
            p += snprintf(hex + p, sizeof(hex) - p, "%02X ", rbuf[i]);
        ESP_LOGI(TAG, "  RX addr=0x%02X (%dB): %s", raddr, rlen, hex);
    } else if (rerr == ESP_ERR_TIMEOUT) {
        ESP_LOGI(TAG, "  RX: timeout (异步命令或电机没回)");
    } else {
        ESP_LOGW(TAG, "  RX err: %s", esp_err_to_name(rerr));
    }
}

static void send_and_rx(const char *name, uint8_t addr,
                        const uint8_t *frame, int n, uint32_t rx_wait_ms)
{
    log_frame(name, frame, n);
    esp_err_t err = zdt_can_send(addr, frame, n, pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  TX err: %s", esp_err_to_name(err));
        return;
    }
    try_rx(rx_wait_ms);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== zdt_x42s_can esp32c3 motor test ===");
    ESP_LOGI(TAG, "motor_addr=%d  CAN tx=%d rx=%d baud=%d",
             MOTOR_ADDR, CAN_TX, CAN_RX, CAN_BAUD);

    zdt_can_config_t cfg = {
        .tx_gpio  = CAN_TX,
        .rx_gpio  = CAN_RX,
        .baudrate = CAN_BAUD,
    };
    ESP_ERROR_CHECK(zdt_can_init(&cfg));

    uint8_t buf[40];
    int n;
    const uint8_t addr = (uint8_t)MOTOR_ADDR;

    /* ====== 1. enable motor（异步命令，电机不回）====== */
    ESP_LOGI(TAG, "step 1: enable motor (5.3.2)");
    n = zdtBuildMotorEnableCmd(addr, 0x01, ZDT_SYNC_NOW, buf, sizeof buf);
    send_and_rx("enable", addr, buf, n, 100);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* ====== 2. read version（验证电机应答链路）====== */
    ESP_LOGI(TAG, "step 2: read version (5.5.2)");
    n = zdtBuildReadVersionCmd(addr, buf, sizeof buf);
    send_and_rx("read_ver", addr, buf, n, 200);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* ====== 3. trapezoid move ~1 rev（长命令，多包发送）====== */
    ESP_LOGI(TAG, "step 3: trapezoid move ~1 revolution (5.3.10)");
    n = zdtBuildTrapezoidPosModeCmd(addr, ZDT_DIR_CW,
                                    500, 500, 6000, 3600,
                                    ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW,
                                    buf, sizeof buf);
    send_and_rx("trap_move", addr, buf, n, 100);
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* ====== 4. read position 5 次（每次应带位置数据回）====== */
    ESP_LOGI(TAG, "step 4: read realtime pos x5 (5.5.13)");
    for (int i = 0; i < 5; ++i) {
        n = zdtBuildReadRealtimePosCmd(addr, buf, sizeof buf);
        send_and_rx("read_pos", addr, buf, n, 200);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* ====== 5. stop ====== */
    ESP_LOGI(TAG, "step 5: stop (5.3.13)");
    n = zdtBuildImmediateStopCmd(addr, ZDT_SYNC_NOW, buf, sizeof buf);
    send_and_rx("stop", addr, buf, n, 100);

    ESP_LOGI(TAG, "done, deinit");
    zdt_can_deinit();
}
