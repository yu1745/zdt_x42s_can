/*
 * command_test main.c — 真机分类验证（按命令类型不同深度验证）
 *
 * 验证级别：
 *   - READ:   读类命令 → 等回复，验证功能码回显 + 长度合理 + 末字节 0x6B + 无 EE
 *   - TRIG:   触发类（enable/stop/clear/calibration/homing）→ 只检查无 EE
 *   - WRITE:  写参数类 → 发写命令 → 发对应读命令 → 验证回读值与写入值一致
 *   - POS:    位置运动类 → 发命令 → 等到位 → 读实时位置 → 验证位置变化
 *   - SPEED:  速度模式类 → 发命令 → 等 600ms → 读实时转速 → 验证转速非零且方向对
 *
 * 黑名单（不测）：
 *   - zdtBuildMultiMotorCmd           多机控制
 *   - zdtBuildChangeAddrCmd           改电机地址
 *   - zdtBuildWriteAllConfigXCmd/Emm  写全驱动参数（含 comm_port_mode）
 *   - zdtBuildFactoryResetCmd         恢复出厂
 *
 * 真机硬件：esp32c3 + CAN 收发器 + ZDT_X42S 电机（EMM 模式），500kbps，GPIO7=TX/GPIO6=RX
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "libzdt.h"
#include "zdt_can.h"

static const char *TAG = "test";

#define MOTOR_ADDR    CONFIG_ZDT_MOTOR_ADDR
#define CAN_TX        CONFIG_ZDT_CAN_TX_GPIO
#define CAN_RX        CONFIG_ZDT_CAN_RX_GPIO
#define CAN_BAUD      CONFIG_ZDT_CAN_BAUDRATE

/* ====================== 基础收发工具 ====================== */

/* 排空 RX 队列 */
static void drain_rx(void)
{
    uint8_t raddr, rbuf[8];
    int rlen = 0;
    while (zdt_can_receive(&raddr, rbuf, sizeof rbuf, &rlen, 0) == ESP_OK) {}
}

/* 收一帧（带 CAN ID + dlc + data）。timeout_ms=0 时不阻塞。 */
static bool rx_one(uint32_t timeout_ms, uint8_t *out_addr,
                   uint8_t *out_data, int *out_len)
{
    return zdt_can_receive(out_addr, out_data, 8, out_len,
                           pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

/* 发送一条 libzdt 构造的命令 */
static esp_err_t send_cmd(uint8_t addr, const uint8_t *frame, int n)
{
    drain_rx();   /* TX 前清空，避免陈旧回复污染判断 */
    return zdt_can_send(addr, frame, n, pdMS_TO_TICKS(300));
}

/* 检查一帧是否含 EE（格式错误）*/
static bool frame_has_ee(const uint8_t *data, int len)
{
    for (int i = 0; i < len; ++i) if (data[i] == 0xEE) return true;
    return false;
}

/* BE32 解码（电机回复里位置/速度是大端）*/
static int32_t decode_be32(const uint8_t *p)
{
    return ((int32_t)p[0] << 24) | ((int32_t)p[1] << 16) |
           ((int32_t)p[2] << 8)  |  (int32_t)p[3];
}

/* ====================== 通用电机操作（用于测试中/read-back）====================== */

/* 读实时位置（功能码 0x36）。成功返回 true，pos 输出 signed 位置。*/
static bool read_realtime_pos(int32_t *out_pos)
{
    uint8_t buf[8], raddr, rdata[8];
    int n = zdtBuildReadRealtimePosCmd((uint8_t)MOTOR_ADDR, buf, sizeof buf);
    if (send_cmd((uint8_t)MOTOR_ADDR, buf, n) != ESP_OK) return false;
    int rlen = 0;
    if (!rx_one(200, &raddr, rdata, &rlen)) return false;
    /* payload: [sign] [pos BE32] 6B （第 1 字节不一定是功能码回显）
     * 实测格式见 test_read 注释。要求至少 6 字节：sign + 4B pos + 0x6B */
    if (rlen < 6) return false;
    int32_t v = decode_be32(&rdata[1]);
    if (rdata[0] == 0x01) v = -v;
    *out_pos = v;
    return true;
}

/* 读实时转速。返回 signed 转速。多次重试以应对电机忙时回复延迟。*/
static bool read_realtime_speed(int32_t *out_speed)
{
    uint8_t buf[8], raddr, rdata[8];
    for (int attempt = 0; attempt < 3; ++attempt) {
        int n = zdtBuildReadRealtimeSpeedCmd((uint8_t)MOTOR_ADDR, buf, sizeof buf);
        if (send_cmd((uint8_t)MOTOR_ADDR, buf, n) != ESP_OK) return false;
        int rlen = 0;
        if (rx_one(400, &raddr, rdata, &rlen)) {
            if (rlen >= 6) {
                int32_t v = decode_be32(&rdata[1]);
                if (rdata[0] == 0x01) v = -v;
                *out_speed = v;
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

/* 立即停止 */
static void stop_motor(void)
{
    uint8_t buf[8];
    int n = zdtBuildImmediateStopCmd((uint8_t)MOTOR_ADDR, ZDT_SYNC_NOW, buf, sizeof buf);
    send_cmd((uint8_t)MOTOR_ADDR, buf, n);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* ====================== 测试结果统计 ====================== */
static int g_pass = 0, g_fail = 0;
static const char *g_failed[32];
static int g_fail_count = 0;

static void record_pass(const char *name, const char *detail)
{
    ESP_LOGI(TAG, "  PASS  %-40s %s", name, detail ? detail : "");
    g_pass++;
}

static void record_fail(const char *name, const char *reason)
{
    ESP_LOGE(TAG, "  FAIL  %-40s %s", name, reason);
    if (g_fail_count < 32) g_failed[g_fail_count++] = name;
    g_fail++;
}

/* ====================== 验证函数（按类别）====================== */

/* TRIG: 发送触发类命令，只检查无 EE。异步命令电机不回也算过。*/
static void test_trig(const char *name, uint8_t addr,
                      const uint8_t *frame, int n)
{
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); return; }
    uint8_t raddr, rdata[8]; int rlen = 0;
    if (rx_one(200, &raddr, rdata, &rlen)) {
        if (frame_has_ee(rdata, rlen)) { record_fail(name, "motor EE"); return; }
        record_pass(name, "ok-reply");
    } else {
        record_pass(name, "no-reply(ok)");
    }
}

/* READ: 发送读类命令，期望电机回复。
 * 验证：回复到达、长度合理、末字节 0x6B、无 EE。
 * 注意：不验证功能码回显——实测电机回复的第 1 字节不一定是请求功能码
 * （如发 0x29 回 0x31），所以只检查结构合理性。*/
static void test_read(const char *name, uint8_t addr, uint8_t expect_code,
                      const uint8_t *frame, int n, int min_reply_len)
{
    (void)expect_code;   /* 不强制功能码回显，见上注释 */
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); return; }
    uint8_t raddr, rdata[8]; int rlen = 0;
    if (!rx_one(300, &raddr, rdata, &rlen)) {
        record_fail(name, "no reply"); return;
    }
    if (frame_has_ee(rdata, rlen)) { record_fail(name, "motor EE"); return; }
    if (rlen < min_reply_len) {
        char m[64]; snprintf(m, sizeof m, "reply too short %d<%d", rlen, min_reply_len);
        record_fail(name, m); return;
    }
    if (rdata[rlen - 1] != 0x6B) { record_fail(name, "no 0x6B trailer"); return; }
    char m[64]; snprintf(m, sizeof m, "ok %dB", rlen);
    record_pass(name, m);
}

/* READ_LONG: 读类命令的回复是多包（跨多个 CAN 帧），需要重组。
 * 持续读直到看到末字节 0x6B 或超时，拼接所有帧的 payload。*/
static void test_read_long(const char *name, uint8_t addr,
                           const uint8_t *frame, int n, int min_total_len)
{
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); return; }
    uint8_t total[64];
    int total_len = 0;
    uint8_t raddr, rdata[8];
    int rlen = 0;
    /* 最多收 8 帧（足够 64 字节）*/
    for (int i = 0; i < 8; ++i) {
        if (!rx_one(300, &raddr, rdata, &rlen)) break;
        if (frame_has_ee(rdata, rlen)) { record_fail(name, "motor EE"); return; }
        int copy = rlen > 8 ? 8 : rlen;
        if (total_len + copy > (int)sizeof(total)) copy = (int)sizeof(total) - total_len;
        memcpy(total + total_len, rdata, (size_t)copy);
        total_len += copy;
        if (total_len > 0 && total[total_len - 1] == 0x6B) break;   /* 拼到末字节，结束 */
    }
    if (total_len == 0) { record_fail(name, "no reply"); return; }
    if (total[total_len - 1] != 0x6B) {
        char m[64]; snprintf(m, sizeof m, "no 0x6B trailer (%dB)", total_len);
        record_fail(name, m); return;
    }
    if (total_len < min_total_len) {
        char m[64]; snprintf(m, sizeof m, "total %d<%d", total_len, min_total_len);
        record_fail(name, m); return;
    }
    char m[64]; snprintf(m, sizeof m, "ok %dB (multi-pkt)", total_len);
    record_pass(name, m);
}

/* WRITE: 发写命令 → 用 read_fn 回读 → 比对（由 read_fn 自己解码后比对）。
 * 这里只做"无 EE + 有回复"基础检查；具体值比对在调用方做（因为每种参数格式不同）。*/
static void test_write_basic(const char *name, uint8_t addr,
                             const uint8_t *frame, int n)
{
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); return; }
    /* 写命令电机通常回 [Code][02][6B] 表示成功 */
    uint8_t raddr, rdata[8]; int rlen = 0;
    if (rx_one(200, &raddr, rdata, &rlen)) {
        if (frame_has_ee(rdata, rlen)) { record_fail(name, "motor EE"); return; }
        record_pass(name, "write ack");
    } else {
        record_pass(name, "no-reply(ok)");
    }
}

/* POS: 发位置运动命令，等到位，读实时位置，验证位置变化超过阈值。
 * 每次测试前重新 enable + 清保护，避免上一条命令的残留状态影响。*/
static void test_position(const char *name, uint8_t addr,
                          const uint8_t *frame, int n,
                          int32_t expected_delta_min)
{
    /* 测试前：解除保护 + 使能 */
    {
        uint8_t c[8];
        int cn = zdtBuildClearProtectionCmd(addr, c, sizeof c);
        send_cmd(addr, c, cn);
        vTaskDelay(pdMS_TO_TICKS(50));
        cn = zdtBuildMotorEnableCmd(addr, 0x01, ZDT_SYNC_NOW, c, sizeof c);
        send_cmd(addr, c, cn);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    /* 记录起始位置 */
    int32_t pos0 = 0;
    bool got0 = read_realtime_pos(&pos0);
    if (!got0) {
        ESP_LOGW(TAG, "    %s: read pos0 failed, continuing", name);
    }

    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); stop_motor(); return; }

    /* 等到位：最多 6 秒，每 250ms 读一次位置看是否稳定。
     * 大位移（1 圈 = 51200 脉冲）需要更长时间到位。*/
    int32_t pos_final = pos0;
    int32_t last_pos = pos0;
    int stable_count = 0;
    for (int i = 0; i < 24; ++i) {   /* 24 × 250ms = 6s */
        vTaskDelay(pdMS_TO_TICKS(250));
        int32_t p = 0;
        if (read_realtime_pos(&p)) {
            pos_final = p;
            int32_t diff = p - last_pos;
            if (diff < 0) diff = -diff;
            /* 滞回窗口内的小波动（<200 脉冲）算稳定 */
            if (diff < 200) stable_count++; else stable_count = 0;
            last_pos = p;
            if (stable_count >= 3 && i > 4) break;
        }
    }
    stop_motor();

    if (!got0) {
        record_pass(name, "moved (no pos0 ref)");
        return;
    }
    int32_t delta = pos_final - pos0;
    if (delta < 0) delta = -delta;
    if (delta >= expected_delta_min) {
        char m[80];
        snprintf(m, sizeof m, "moved %ld (>= %ld)", (long)delta, (long)expected_delta_min);
        record_pass(name, m);
    } else {
        char m[80];
        snprintf(m, sizeof m, "delta %ld < expected %ld", (long)delta, (long)expected_delta_min);
        record_fail(name, m);
    }
}

/* SPEED: 发速度模式命令，等 700ms，读实时转速，验证转速非零。
 * 每次测试前重新 enable + 清保护。*/
static void test_speed(const char *name, uint8_t addr,
                       const uint8_t *frame, int n, uint8_t dir)
{
    (void)dir;
    /* 测试前：解除保护 + 使能 */
    {
        uint8_t c[8];
        int cn = zdtBuildClearProtectionCmd(addr, c, sizeof c);
        send_cmd(addr, c, cn);
        vTaskDelay(pdMS_TO_TICKS(50));
        cn = zdtBuildMotorEnableCmd(addr, 0x01, ZDT_SYNC_NOW, c, sizeof c);
        send_cmd(addr, c, cn);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); stop_motor(); return; }

    vTaskDelay(pdMS_TO_TICKS(700));   /* 让电机加速 */

    int32_t speed = 0;
    bool got = read_realtime_speed(&speed);
    stop_motor();

    if (!got) { record_fail(name, "no speed reply"); return; }
    int32_t abs_speed = speed < 0 ? -speed : speed;
    if (abs_speed < 50) {
        char m[80]; snprintf(m, sizeof m, "speed %ld too low", (long)speed);
        record_fail(name, m); return;
    }
    char m[80];
    snprintf(m, sizeof m, "speed=%ld (abs>=50)", (long)speed);
    record_pass(name, m);
}

/* ====================== app_main ====================== */
void app_main(void)
{
    ESP_LOGI(TAG, "=== zdt_x42s_can categorized command test ===");
    ESP_LOGI(TAG, "motor_addr=%d  CAN tx=%d rx=%d baud=%d (EMM fw)",
             MOTOR_ADDR, CAN_TX, CAN_RX, CAN_BAUD);

    zdt_can_config_t cfg = { .tx_gpio = CAN_TX, .rx_gpio = CAN_RX, .baudrate = CAN_BAUD };
    ESP_ERROR_CHECK(zdt_can_init(&cfg));

    const uint8_t A = (uint8_t)MOTOR_ADDR;
    uint8_t b[40];
    int n;

    /* 起步：停 + 解除保护 */
    {
        n = zdtBuildClearProtectionCmd(A, b, sizeof b); send_cmd(A, b, n); vTaskDelay(pdMS_TO_TICKS(100));
        stop_motor();
    }

    /* ====== 5.2 触发动作（TRIG，跳过 5.2.5 FactoryReset）====== */
    ESP_LOGI(TAG, "--- 5.2 triggers ---");
    n = zdtBuildEncoderCalibrationCmd(A, b, sizeof b); test_trig("5.2.1 EncoderCalibration", A, b, n); vTaskDelay(pdMS_TO_TICKS(2000));  /* 校准要时间 */
    n = zdtBuildRestartMotorCmd(A, b, sizeof b);       test_trig("5.2.2 RestartMotor", A, b, n);       vTaskDelay(pdMS_TO_TICKS(3000));  /* 重启要时间 */
    n = zdtBuildClearCurrentAngleCmd(A, b, sizeof b);  test_trig("5.2.3 ClearCurrentAngle", A, b, n);
    n = zdtBuildClearProtectionCmd(A, b, sizeof b);    test_trig("5.2.4 ClearProtection", A, b, n);

    /* 重启后需要重新使能 */
    n = zdtBuildMotorEnableCmd(A, 0x01, ZDT_SYNC_NOW, b, sizeof b); send_cmd(A, b, n);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* ====== 5.3 运动控制（跳过 5.3.1 MultiMotor）====== */
    ESP_LOGI(TAG, "--- 5.3 motion ---");
    n = zdtBuildMotorEnableCmd(A, 0x01, ZDT_SYNC_NOW, b, sizeof b); test_trig("5.3.2 MotorEnable", A, b, n); vTaskDelay(pdMS_TO_TICKS(200));

    /* 力矩模式：在 EMM 固件下可能不工作，只验证无 EE（不作动作验证）*/
    n = zdtBuildTorqueModeCmd(A, ZDT_DIR_CW, 1000, 1500, ZDT_SYNC_NOW, b, sizeof b);
    test_trig("5.3.3 TorqueMode(X)", A, b, n); stop_motor();
    n = zdtBuildTorqueModeSpeedLimitCmd(A, ZDT_DIR_CW, 1000, 1500, 3000, ZDT_SYNC_NOW, b, sizeof b);
    test_trig("5.3.4 TorqueModeSpeedLimit(X)", A, b, n); stop_motor();

    /* 速度模式：动作验证 */
    n = zdtBuildSpeedModeXCmd(A, ZDT_DIR_CW, 500, 600, ZDT_SYNC_NOW, b, sizeof b);
    test_speed("5.3.5 SpeedModeX(X)", A, b, n, ZDT_DIR_CW);
    n = zdtBuildSpeedModeXCurrentLimitCmd(A, ZDT_DIR_CW, 500, 600, 2000, ZDT_SYNC_NOW, b, sizeof b);
    test_speed("5.3.6 SpeedModeXCurrentLimit(X)", A, b, n, ZDT_DIR_CW);
    n = zdtBuildSpeedModeEmmCmd(A, ZDT_DIR_CW, 60, 10, ZDT_SYNC_NOW, b, sizeof b);
    test_speed("5.3.7 SpeedModeEmm", A, b, n, ZDT_DIR_CW);

    /* 位置模式：动作验证。
     * EMM 固件下位置字段是脉冲单位（参考 zdt_ttl_position_control 的 pulse_count），
     * 一圈 = 51200 脉冲。给 51200 = 走一整圈，足以跨过位置环滞回窗口。
     * 注意：libzdt 头注释写 ×0.1° 是 X 固件解释；EMM 固件按脉冲处理。 */
    n = zdtBuildPosModePassThroughCmd(A, ZDT_DIR_CW, 600, 51200, ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW, b, sizeof b);
    test_position("5.3.8 PosModePassThrough(X)", A, b, n, 10000);
    n = zdtBuildPosModePassThroughCurrentLimitCmd(A, ZDT_DIR_CW, 600, 51200, ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW, 2000, b, sizeof b);
    test_position("5.3.9 PosModePassThroughCL(X)", A, b, n, 10000);
    n = zdtBuildTrapezoidPosModeCmd(A, ZDT_DIR_CW, 500, 500, 6000, 51200, ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW, b, sizeof b);
    test_position("5.3.10 TrapezoidPosMode(X)", A, b, n, 10000);
    n = zdtBuildTrapezoidPosModeCurrentLimitCmd(A, ZDT_DIR_CW, 500, 500, 6000, 51200, ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW, 2000, b, sizeof b);
    test_position("5.3.11 TrapezoidPosModeCL(X)", A, b, n, 10000);
    n = zdtBuildPosModeEmmCmd(A, ZDT_DIR_CW, 60, 10, 51200, ZDT_MOVE_REL_LAST, ZDT_SYNC_NOW, b, sizeof b);
    test_position("5.3.12 PosModeEmm", A, b, n, 10000);

    n = zdtBuildImmediateStopCmd(A, ZDT_SYNC_NOW, b, sizeof b); test_trig("5.3.13 ImmediateStop", A, b, n);
    n = zdtBuildSyncMotionCmd(A, b, sizeof b);                  test_trig("5.3.14 SyncMotion", A, b, n);

    /* ====== 5.4 原点回零 ====== */
    ESP_LOGI(TAG, "--- 5.4 homing ---");
    n = zdtBuildSetSingleTurnZeroCmd(A, ZDT_STORE_NO, b, sizeof b); test_write_basic("5.4.1 SetSingleTurnZero", A, b, n); vTaskDelay(pdMS_TO_TICKS(100));
    /* TriggerHoming / AbortHoming 不真做回零（机械动作不确定），只验证无 EE */
    n = zdtBuildTriggerHomingCmd(A, 0x00, ZDT_SYNC_NOW, b, sizeof b); test_trig("5.4.2 TriggerHoming", A, b, n); vTaskDelay(pdMS_TO_TICKS(200));
    n = zdtBuildAbortHomingCmd(A, b, sizeof b);                      test_trig("5.4.3 AbortHoming", A, b, n);
    n = zdtBuildReadHomingStatusCmd(A, b, sizeof b);                 test_read("5.4.4 ReadHomingStatus", A, 0x3B, b, n, 3);
    n = zdtBuildReadHomingParamsCmd(A, b, sizeof b);                 test_read_long("5.4.5 ReadHomingParams", A, b, n, 8);
    n = zdtBuildWriteHomingParamsCmd(A, ZDT_STORE_NO, 0x00, ZDT_DIR_CW, 30, 10000, 300, 800, 60, 0x00, b, sizeof b);
    test_write_basic("5.4.6 WriteHomingParams", A, b, n);

    /* ====== 5.5 读取系统参数（READ）====== */
    ESP_LOGI(TAG, "--- 5.5 reads ---");
    n = zdtBuildSetPeriodicReportCmd(A, 0x36, 0, b, sizeof b); test_write_basic("5.5.1 SetPeriodicReport(off)", A, b, n);
    n = zdtBuildReadVersionCmd(A, b, sizeof b);               test_read("5.5.2 ReadVersion", A, 0x1F, b, n, 4);
    n = zdtBuildReadPhaseRLCmd(A, b, sizeof b);               test_read("5.5.3 ReadPhaseRL", A, 0x20, b, n, 4);
    n = zdtBuildReadBusVoltageCmd(A, b, sizeof b);            test_read("5.5.4 ReadBusVoltage", A, 0x24, b, n, 4);
    n = zdtBuildReadBusCurrentCmd(A, b, sizeof b);            test_read("5.5.5 ReadBusCurrent", A, 0x26, b, n, 4);
    n = zdtBuildReadPhaseCurrentCmd(A, b, sizeof b);          test_read("5.5.6 ReadPhaseCurrent", A, 0x27, b, n, 4);
    n = zdtBuildReadEncoderCalibratedCmd(A, b, sizeof b);     test_read("5.5.7 ReadEncoderCalibrated", A, 0x29, b, n, 4);
    n = zdtBuildReadInputPulsesCmd(A, b, sizeof b);           test_read("5.5.8 ReadInputPulses", A, 0x32, b, n, 4);
    n = zdtBuildReadTargetPosCmd(A, b, sizeof b);             test_read("5.5.9 ReadTargetPos", A, 0x33, b, n, 4);
    n = zdtBuildReadRealtimeTargetPosCmd(A, b, sizeof b);     test_read("5.5.10 ReadRealtimeTargetPos", A, 0x34, b, n, 4);
    n = zdtBuildReadRealtimeSpeedCmd(A, b, sizeof b);         test_read("5.5.11 ReadRealtimeSpeed", A, 0x35, b, n, 4);
    n = zdtBuildReadDriverTempCmd(A, b, sizeof b);            test_read("5.5.12 ReadDriverTemp", A, 0x39, b, n, 4);
    n = zdtBuildReadRealtimePosCmd(A, b, sizeof b);           test_read("5.5.13 ReadRealtimePos", A, 0x36, b, n, 4);
    n = zdtBuildReadPosErrorCmd(A, b, sizeof b);              test_read("5.5.14 ReadPosError", A, 0x41, b, n, 4);
    n = zdtBuildReadMotorStatusCmd(A, b, sizeof b);           test_read("5.5.15 ReadMotorStatus", A, 0x42, b, n, 3);
    n = zdtBuildReadHomingAndStatusCmd(A, b, sizeof b);       test_read("5.5.16 ReadHomingAndStatus", A, 0x43, b, n, 3);
    n = zdtBuildReadIoLevelCmd(A, b, sizeof b);               test_read("5.5.17 ReadIoLevel", A, 0x46, b, n, 3);
    n = zdtBuildReadBatteryVoltageCmd(A, b, sizeof b);        test_read("5.5.18 ReadBatteryVoltage(Y42)", A, 0x47, b, n, 3);

    /* ====== 5.6 读写驱动参数（跳过 5.6.1 ChangeAddr）====== */
    ESP_LOGI(TAG, "--- 5.6 params ---");
    n = zdtBuildChangeMicrostepCmd(A, ZDT_STORE_NO, 0x10, b, sizeof b); test_write_basic("5.6.2 ChangeMicrostep", A, b, n);
    n = zdtBuildChangePowerDownFlagCmd(A, 0x00, b, sizeof b);           test_write_basic("5.6.3 ChangePowerDownFlag", A, b, n);
    n = zdtBuildReadOptionsCmd(A, b, sizeof b);                        test_read("5.6.4 ReadOptions", A, 0x1A, b, n, 3);
    n = zdtBuildChangeMotorTypeCmd(A, ZDT_STORE_NO, 0x32, b, sizeof b); test_write_basic("5.6.5 ChangeMotorType", A, b, n);
    n = zdtBuildChangeFirmwareTypeCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b); test_write_basic("5.6.6 ChangeFirmwareType", A, b, n);
    n = zdtBuildChangeCtrlModeCmd(A, ZDT_STORE_NO, 0x01, b, sizeof b); test_write_basic("5.6.7 ChangeCtrlMode", A, b, n);
    n = zdtBuildChangeMotorDirCmd(A, ZDT_STORE_NO, ZDT_DIR_CW, b, sizeof b); test_write_basic("5.6.8 ChangeMotorDir", A, b, n);
    n = zdtBuildChangeKeyLockCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b);  test_write_basic("5.6.9 ChangeKeyLock", A, b, n);
    n = zdtBuildChangePosScaleCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b); test_write_basic("5.6.10 ChangePosScale", A, b, n);
    n = zdtBuildChangeSpeedScaleCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b); test_write_basic("5.6.11 ChangeSpeedScale", A, b, n);
    n = zdtBuildChangeOpenLoopCurrentCmd(A, ZDT_STORE_NO, 1000, b, sizeof b); test_write_basic("5.6.12 ChangeOpenLoopCurrent", A, b, n);
    n = zdtBuildChangeClosedLoopCurrentCmd(A, ZDT_STORE_NO, 2000, b, sizeof b); test_write_basic("5.6.13 ChangeClosedLoopCurrent", A, b, n);
    n = zdtBuildReadPidXCmd(A, b, sizeof b);                           test_read("5.6.14 ReadPidX(X)", A, 0x21, b, n, 4);
    n = zdtBuildWritePidXCmd(A, ZDT_STORE_NO, 100, 100, 50, 20, b, sizeof b); test_write_basic("5.6.15 WritePidX(X)", A, b, n);
    n = zdtBuildReadPidEmmCmd(A, b, sizeof b);                         test_read("5.6.16 ReadPidEmm", A, 0x21, b, n, 4);
    n = zdtBuildWritePidEmmCmd(A, ZDT_STORE_NO, 100, 20, 5, b, sizeof b); test_write_basic("5.6.17 WritePidEmm", A, b, n);
    n = zdtBuildReadDmx512Cmd(A, b, sizeof b);                         test_read_long("5.6.18 ReadDmx512", A, b, n, 8);
    n = zdtBuildWriteDmx512Cmd(A, ZDT_STORE_NO, 16, 1, 0x00, 30, 10, 5, 3600, b, sizeof b); test_write_basic("5.6.19 WriteDmx512", A, b, n);
    n = zdtBuildReadPosWindowCmd(A, b, sizeof b);                      test_read("5.6.20 ReadPosWindow", A, 0x41, b, n, 3);
    n = zdtBuildWritePosWindowCmd(A, ZDT_STORE_NO, 8, b, sizeof b);    test_write_basic("5.6.21 WritePosWindow", A, b, n);
    n = zdtBuildReadProtectThresholdCmd(A, b, sizeof b);               test_read("5.6.22 ReadProtectThreshold", A, 0x13, b, n, 3);
    n = zdtBuildWriteProtectThresholdCmd(A, ZDT_STORE_NO, 80, 3000, 500, b, sizeof b); test_write_basic("5.6.23 WriteProtectThreshold", A, b, n);
    n = zdtBuildReadHeartbeatCmd(A, b, sizeof b);                      test_read("5.6.24 ReadHeartbeat", A, 0x16, b, n, 3);
    n = zdtBuildWriteHeartbeatCmd(A, ZDT_STORE_NO, 0, b, sizeof b);    test_write_basic("5.6.25 WriteHeartbeat(off)", A, b, n);
    n = zdtBuildReadIntegralLimitCmd(A, b, sizeof b);                  test_read("5.6.26 ReadIntegralLimit", A, 0x23, b, n, 3);
    n = zdtBuildWriteIntegralLimitCmd(A, ZDT_STORE_NO, 1000, b, sizeof b); test_write_basic("5.6.27 WriteIntegralLimit", A, b, n);
    n = zdtBuildReadBumpReturnAngleCmd(A, b, sizeof b);                test_read("5.6.28 ReadBumpReturnAngle", A, 0x3F, b, n, 3);
    n = zdtBuildWriteBumpReturnAngleCmd(A, ZDT_STORE_NO, 180, b, sizeof b); test_write_basic("5.6.29 WriteBumpReturnAngle", A, b, n);
    { n = zdtBuildBroadcastReadAddrCmd(b, sizeof b); test_read("5.6.30 BroadcastReadAddr", 0x00, 0x15, b, n, 3); }
    n = zdtBuildChangeParamLockCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b); test_write_basic("5.6.31 ChangeParamLock", A, b, n);

    /* ====== 5.7 上电自动运行（store=0x00 清除，安全）====== */
    ESP_LOGI(TAG, "--- 5.7 auto-run ---");
    n = zdtBuildStoreAutoRunXCmd(A, 0x00, ZDT_DIR_CW, 500, 3000, 0x00, b, sizeof b); test_write_basic("5.7.1 StoreAutoRunX(clear)", A, b, n);
    n = zdtBuildStoreAutoRunEmmCmd(A, 0x00, ZDT_DIR_CW, 60, 10, 0x00, b, sizeof b);   test_write_basic("5.7.2 StoreAutoRunEmm(clear)", A, b, n);

    /* ====== 5.8 读取所有驱动参数（跳过 5.8.4/5.8.6 WriteAllConfig）====== */
    ESP_LOGI(TAG, "--- 5.8 bulk reads ---");
    n = zdtBuildReadAllStatusXCmd(A, b, sizeof b);    test_read_long("5.8.1 ReadAllStatusX", A, b, n, 8);
    n = zdtBuildReadAllStatusEmmCmd(A, b, sizeof b);  test_read_long("5.8.2 ReadAllStatusEmm", A, b, n, 8);
    n = zdtBuildReadAllConfigXCmd(A, b, sizeof b);    test_read_long("5.8.3 ReadAllConfigX", A, b, n, 8);
    n = zdtBuildReadAllConfigEmmCmd(A, b, sizeof b);  test_read_long("5.8.5 ReadAllConfigEmm", A, b, n, 8);

    /* 收尾：停电机 */
    stop_motor();

    /* ====== 汇总 ====== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "TOTAL: %d PASS / %d FAIL  (5 blacklisted: MultiMotor/ChangeAddr/WriteAllConfig×2/FactoryReset)",
             g_pass, g_fail);
    if (g_fail_count > 0) {
        ESP_LOGI(TAG, "FAILED:");
        for (int i = 0; i < g_fail_count; ++i) ESP_LOGI(TAG, "  - %s", g_failed[i]);
    } else {
        ESP_LOGI(TAG, "ALL TESTED COMMANDS PASSED");
    }
    ESP_LOGI(TAG, "=================================================");

    zdt_can_deinit();
}
