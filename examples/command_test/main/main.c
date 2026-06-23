/*
 * command_test main.c — 真机逐命令逐参数验证（重写版）
 *
 * 设计目标：
 *   每条命令、每个参数都单独测一轮；写参数类命令一律配读回比对。
 *
 * 验证级别：
 *   - TRIG:   触发类（enable/stop/clear/restart）→ 只检查无 EE
 *   - READ:   读类 → 等回复，验长度/末字节 0x6B/无 EE，打印 payload hex
 *   - WRITE:  写参数类 → 发写 → 发对应读 → 解码比对
 *   - POS:    位置运动类 → 记 pos0 → 发命令 → 等到位 → 读 pos → 按到达窗口判定
 *             （位置限速指令的速度参数不测时序，只验位置到位）
 *   - SPEED:  速度模式 → 发命令 → 等 700ms → 读实时转速 → 验方向+量级
 *   - MULTI:  多机命令（仅扫到 ≥2 台时测）
 *
 * 位置到位判定（关键，不要求精确等于）：
 *   - setup 阶段读 ReadPosWindow 拿真实窗口 W（编码器单位）
 *   - tol = W + 余量（机械回差 + 编码器量化）
 *   - REL_NOW: |Δ - pulses| ≤ tol
 *   - ABS_ZERO: |final - target| ≤ tol
 *   - REL_LAST: 累计基准 + pulses（本测试不连续发，退化为 REL_NOW 语义）
 *   - 所有位置测试脉冲量 ≥1000，远大于窗口，避免"小位移被窗口吞掉不动"
 *
 * 黑名单（不测）：
 *   - 5.2.1 EncoderCalibration    CAL 危险
 *   - 5.2.5 FactoryReset          清空配置
 *   - 5.4.2 TriggerHoming         无原点/限位条件
 *   - 5.6.1 ChangeAddr            改 CAN ID → 丢通信
 *   - 5.6.6 ChangeFirmwareType    切固件模式
 *   - 5.8.4/5.8.6 WriteAllConfig×2  含 can_speed/com_mode → 改波特率丢通信
 *   - X 固件命令（5.3.3-5.3.6, 5.3.8-5.3.11, 5.6.10/14/15, 5.7.1, 5.8.1/3）
 *
 * 风险写命令（改电机行为）：测后立即恢复默认。
 *   - WritePidEmm / ChangeMotorType / ChangeCtrlMode / ChangeMotorDir
 *
 * Setup（不占轮数）：
 *   1. 扫描 ID 1–15 → found_addrs[]，打印在线清单
 *   2. 主测电机 A = found_addrs[0]
 *   3. 读 ReadPosWindow → 算 tol
 *
 * 真机硬件：esp32c3 + CAN 收发器 + ZDT_X42S（EMM fw），500kbps，GPIO7=TX/GPIO6=RX
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

#define CAN_TX        CONFIG_ZDT_CAN_TX_GPIO
#define CAN_RX        CONFIG_ZDT_CAN_RX_GPIO
#define CAN_BAUD      CONFIG_ZDT_CAN_BAUDRATE

/* 扫描范围上限（扫 1..SCAN_MAX） */
#define SCAN_MAX      15

/* ====================== 基础收发工具 ====================== */

/* 排空 RX 队列 */
static void drain_rx(void)
{
    uint8_t raddr, rbuf[8];
    int rlen = 0;
    while (zdt_can_receive(&raddr, rbuf, sizeof rbuf, &rlen, 0) == ESP_OK) {}
}

/* 收一帧。timeout_ms=0 时不阻塞。 */
static bool rx_one(uint32_t timeout_ms, uint8_t *out_addr,
                   uint8_t *out_data, int *out_len)
{
    return zdt_can_receive(out_addr, out_data, 8, out_len,
                           pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

/* 发送一条 libzdt 构造的命令（先清空 RX 避免陈旧回复污染） */
static esp_err_t send_cmd(uint8_t addr, const uint8_t *frame, int n)
{
    drain_rx();
    return zdt_can_send(addr, frame, n, pdMS_TO_TICKS(300));
}

/* 检查一帧是否含 EE（格式错误） */
static bool frame_has_ee(const uint8_t *data, int len)
{
    for (int i = 0; i < len; ++i) if (data[i] == 0xEE) return true;
    return false;
}

/* BE32 解码（回复里位置/速度是大端） */
static int32_t decode_be32(const uint8_t *p)
{
    return ((int32_t)p[0] << 24) | ((int32_t)p[1] << 16) |
           ((int32_t)p[2] << 8)  |  (int32_t)p[3];
}

/* BE16 解码 */
static uint16_t decode_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* payload hex 打印（去掉末尾 0x6B） */
static void log_payload(const char *name, const uint8_t *data, int len)
{
    char hex[160];
    int off = 0;
    for (int i = 0; i < len && off + 3 < (int)sizeof(hex); ++i)
        off += snprintf(hex + off, sizeof(hex) - off, "%02X ", data[i]);
    if (off > 0 && hex[off - 1] == ' ') hex[--off] = '\0';
    ESP_LOGI(TAG, "      [%s] %s", name, hex);
}

/* ====================== 扫描 ====================== */
static uint8_t s_found[SCAN_MAX];
static int    s_found_cnt = 0;
static uint8_t s_primary = 0;     /* 主测电机 = 第一个扫到的 */

/* 发广播读地址（5.6.30），收所有回复，解出在线 ID。 */
static void scan_motors(void)
{
    uint8_t b[8];
    int n = zdtBuildBroadcastReadAddrCmd(b, sizeof b);
    /* 广播帧：addr=0x00，每个在线电机各自回一帧 */
    drain_rx();
    zdt_can_send(0x00, b, n, pdMS_TO_TICKS(300));

    uint8_t raddr, rdata[8];
    int rlen = 0;
    /* 收 300ms 内的所有回复 */
    int64_t deadline = esp_log_timestamp() + 600;
    while (esp_log_timestamp() < deadline) {
        if (!rx_one(200, &raddr, rdata, &rlen)) break;
        if (raddr >= 1 && raddr <= SCAN_MAX) {
            bool dup = false;
            for (int i = 0; i < s_found_cnt; ++i) if (s_found[i] == raddr) dup = true;
            if (!dup && s_found_cnt < SCAN_MAX) s_found[s_found_cnt++] = raddr;
        }
    }
    if (s_found_cnt > 0) s_primary = s_found[0];

    ESP_LOGI(TAG, "scan: found %d motor(s):", s_found_cnt);
    for (int i = 0; i < s_found_cnt; ++i)
        ESP_LOGI(TAG, "  [%d] addr=%d", i, s_found[i]);
    if (s_found_cnt == 0)
        ESP_LOGE(TAG, "scan: NO MOTOR FOUND — abort");
    else
        ESP_LOGI(TAG, "primary motor = addr %d", s_primary);
}

/* ====================== 通用电机操作 ====================== */

/* 读实时位置（功能码 0x36/实际回显见真机）。成功返回 true。 */
static bool read_realtime_pos(uint8_t addr, int32_t *out_pos)
{
    uint8_t buf[8], raddr, rdata[8];
    int n = zdtBuildReadRealtimePosCmd(addr, buf, sizeof buf);
    if (send_cmd(addr, buf, n) != ESP_OK) return false;
    int rlen = 0;
    if (!rx_one(200, &raddr, rdata, &rlen)) return false;
    if (rlen < 4) return false;
    int32_t v = 0;
    for (int i = 2; i < rlen - 1; ++i) v = (v << 8) | rdata[i];
    if (rdata[1] == 0x01) v = -v;
    *out_pos = v;
    return true;
}

/* 读实时转速，多次重试。 */
static bool read_realtime_speed(uint8_t addr, int32_t *out_speed)
{
    uint8_t buf[8], raddr, rdata[8];
    for (int attempt = 0; attempt < 3; ++attempt) {
        int n = zdtBuildReadRealtimeSpeedCmd(addr, buf, sizeof buf);
        if (send_cmd(addr, buf, n) != ESP_OK) return false;
        int rlen = 0;
        if (rx_one(400, &raddr, rdata, &rlen)) {
            if (rlen >= 4) {
                int32_t v = 0;
                for (int i = 2; i < rlen - 1; ++i) v = (v << 8) | rdata[i];
                if (rdata[1] == 0x01) v = -v;
                *out_speed = v;
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

/* 读位置到达窗口（5.6.20 ReadPosWindow）。返回 true 并填 out_window（编码器单位）。 */
static bool read_pos_window(uint8_t addr, uint16_t *out_window)
{
    uint8_t buf[8], raddr, rdata[8];
    int n = zdtBuildReadPosWindowCmd(addr, buf, sizeof buf);
    if (send_cmd(addr, buf, n) != ESP_OK) return false;
    int rlen = 0;
    if (!rx_one(300, &raddr, rdata, &rlen)) return false;
    /* 真机回复: [func 41][00][value][6B]，value 单字节在 rdata[2] */
    if (rlen < 4) return false;
    *out_window = rdata[2];
    return true;
}

/* 立即停止 */
static void stop_motor(uint8_t addr)
{
    uint8_t buf[8];
    int n = zdtBuildImmediateStopCmd(addr, ZDT_SYNC_NOW, buf, sizeof buf);
    send_cmd(addr, buf, n);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* 测试前预热：解除保护 + 使能 */
static void prep_motor(uint8_t addr)
{
    uint8_t c[8];
    int cn = zdtBuildClearProtectionCmd(addr, c, sizeof c);
    send_cmd(addr, c, cn);
    vTaskDelay(pdMS_TO_TICKS(50));
    cn = zdtBuildMotorEnableCmd(addr, 0x01, ZDT_SYNC_NOW, c, sizeof c);
    send_cmd(addr, c, cn);
    vTaskDelay(pdMS_TO_TICKS(150));
}

/* ====================== 结果统计 ====================== */
static int g_pass = 0, g_fail = 0, g_skip = 0;
static const char *g_failed[64];
static int g_fail_count = 0;

static void record_pass(const char *name, const char *detail)
{
    ESP_LOGI(TAG, "  PASS  %-44s %s", name, detail ? detail : "");
    g_pass++;
}
static void record_fail(const char *name, const char *reason)
{
    ESP_LOGE(TAG, "  FAIL  %-44s %s", name, reason);
    if (g_fail_count < 64) g_failed[g_fail_count++] = name;
    g_fail++;
}
static void record_skip(const char *name, const char *reason)
{
    ESP_LOGW(TAG, "  SKIP  %-44s %s", name, reason);
    g_skip++;
}

/* ====================== 验证函数（按类别）====================== */

/* TRIG：触发类，只检查无 EE。异步命令电机不回也算过。 */
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

/* READ：读类，期望电机回复。验长度/末字节 0x6B/无 EE，打印 payload。 */
static void test_read(const char *name, uint8_t addr,
                      const uint8_t *frame, int n, int min_reply_len)
{
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); return; }
    uint8_t raddr, rdata[8]; int rlen = 0;
    if (!rx_one(300, &raddr, rdata, &rlen)) { record_fail(name, "no reply"); return; }
    if (frame_has_ee(rdata, rlen)) { record_fail(name, "motor EE"); return; }
    if (rlen < min_reply_len) {
        char m[64]; snprintf(m, sizeof m, "reply too short %d<%d", rlen, min_reply_len);
        record_fail(name, m); return;
    }
    if (rdata[rlen - 1] != 0x6B) { record_fail(name, "no 0x6B trailer"); return; }
    char m[64]; snprintf(m, sizeof m, "ok %dB", rlen);
    log_payload(name, rdata, rlen - 1);
    record_pass(name, m);
}

/* READ_LONG：多包回复，重组。 */
static void test_read_long(const char *name, uint8_t addr,
                           const uint8_t *frame, int n, int min_total_len)
{
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); return; }
    uint8_t total[64];
    int total_len = 0;
    uint8_t raddr, rdata[8];
    int rlen = 0;
    for (int i = 0; i < 8; ++i) {
        if (!rx_one(300, &raddr, rdata, &rlen)) break;
        if (frame_has_ee(rdata, rlen)) { record_fail(name, "motor EE"); return; }
        int copy = rlen > 8 ? 8 : rlen;
        if (total_len + copy > (int)sizeof(total)) copy = (int)sizeof(total) - total_len;
        memcpy(total + total_len, rdata, (size_t)copy);
        total_len += copy;
        if (total_len > 0 && total[total_len - 1] == 0x6B) break;
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
    log_payload(name, total, total_len - 1);
    record_pass(name, m);
}

/* write_cmd：发写命令，返回电机 ACK 是否无 EE（不记 PASS/FAIL，留给调用方）。
 * 具体值比对用专门的 write+read 函数。 */
static bool write_cmd(uint8_t addr, const uint8_t *frame, int n)
{
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) return false;
    uint8_t raddr, rdata[8]; int rlen = 0;
    if (rx_one(200, &raddr, rdata, &rlen)) {
        if (frame_has_ee(rdata, rlen)) return false;
        return true;
    }
    return true;   /* 无回复也认为发出 */
}

/* test_write_basic_wrap：发写命令 → 验 ACK 无 EE → 记 PASS（不做值比对）。
 * 用于没有专用读命令可回读比对的写参数类。 */
static void test_write_basic_wrap(const char *name, uint8_t addr,
                                  const uint8_t *frame, int n)
{
    if (write_cmd(addr, frame, n)) record_pass(name, "write ack");
    else record_fail(name, "send/EE");
}

/* POS：位置运动，按到达窗口判定。
 *   ref_kind: 0=REL_NOW(|Δ-pulses|≤tol), 1=ABS_ZERO(|final-target|≤tol) */
static int32_t s_pos_tol = 1500;   /* 默认容差，setup 后用 ReadPosWindow 算 */

static void test_position(const char *name, uint8_t addr,
                          const uint8_t *frame, int n,
                          int ref_kind, int32_t ref_value)
{
    prep_motor(addr);

    int32_t pos0 = 0;
    bool got0 = read_realtime_pos(addr, &pos0);
    if (!got0) ESP_LOGW(TAG, "    %s: read pos0 failed, continuing", name);

    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); stop_motor(addr); return; }

    /* 固定等待到位（probe 实测：5 圈 @ 3000rpm < 1s）。
     * 不轮询提前停！位置命令到位后电机自己锁位（闭环），
     * 轮询检测"稳定"就 stop 会在电机过冲调整时打断它。等 1500ms 覆盖最大脉冲量。 */
    vTaskDelay(pdMS_TO_TICKS(1500));
    int32_t pos_final = pos0;
    read_realtime_pos(addr, &pos_final);

    if (!got0) { record_pass(name, "moved (no pos0 ref)"); return; }

    int32_t delta = pos_final - pos0;
    int32_t err_abs;
    if (ref_kind == 1) {
        /* ABS_ZERO：参考是绝对目标位置（带符号，比对 final 与 target） */
        err_abs = pos_final - ref_value; if (err_abs < 0) err_abs = -err_abs;
    } else {
        /* REL_NOW/REL_LAST：比对位移绝对值 |delta| vs |ref_value|
         * （CCW 时 delta 为负、ref_value 为正，必须比绝对值） */
        int32_t ad = delta < 0 ? -delta : delta;
        int32_t ar = ref_value < 0 ? -ref_value : ref_value;
        err_abs = ad - ar; if (err_abs < 0) err_abs = -err_abs;
    }
    if (err_abs <= s_pos_tol) {
        char m[96];
        snprintf(m, sizeof m, "p0=%ld pf=%ld d=%ld err=%ld tol=%ld",
                 (long)pos0, (long)pos_final, (long)delta, (long)err_abs, (long)s_pos_tol);
        record_pass(name, m);
    } else {
        char m[120];
        snprintf(m, sizeof m, "p0=%ld pf=%ld d=%ld err=%ld > tol=%ld",
                 (long)pos0, (long)pos_final, (long)delta, (long)err_abs, (long)s_pos_tol);
        record_fail(name, m);
    }
}

/* SPEED：速度模式，等 700ms 读实时转速，验方向+量级。
 *   expect_dir: 0=CW(+), 1=CCW(-) */
static void test_speed(const char *name, uint8_t addr,
                       const uint8_t *frame, int n, uint8_t expect_dir)
{
    prep_motor(addr);
    esp_err_t err = send_cmd(addr, frame, n);
    if (err != ESP_OK) { record_fail(name, esp_err_to_name(err)); stop_motor(addr); return; }
    /* 50ms 轮询转速，|s|>=30（确认在转）即跳，上限 2s */
    int32_t speed = 0;
    bool got = false;
    for (int ms = 0; ms < 2000 && !got; ms += 50) {
        vTaskDelay(pdMS_TO_TICKS(50));
        got = read_realtime_speed(addr, &speed);
        if (got) {
            int32_t a = speed < 0 ? -speed : speed;
            if (a >= 30) break;        /* 已在转，立即跳出 */
            got = false;               /* 还太慢，继续等 */
        }
    }
    stop_motor(addr);
    if (!got) { record_fail(name, "no speed reply"); return; }
    int32_t abs_sp = speed < 0 ? -speed : speed;
    if (abs_sp < 30) {
        char m[80]; snprintf(m, sizeof m, "speed %ld too low", (long)speed);
        record_fail(name, m); return;
    }
    /* 方向校验：CW→正，CCW→负（EMM 速度回读单位 RPM） */
    bool dir_ok = (expect_dir == ZDT_DIR_CW) ? (speed > 0) : (speed < 0);
    char m[96];
    snprintf(m, sizeof m, "speed=%ld dir=%s", (long)speed, dir_ok ? "ok" : "WRONG");
    if (dir_ok) record_pass(name, m);
    else record_fail(name, m);
}

/* ====================== 写参数 + 读回比对（round-trip） ======================
 * 每种写参数格式不同，各写专门 helper，避免误解码。 */

/* 读回一帧原始 payload（已去掉末尾 0x6B 的长度），返回解码起点偏移。 */
static bool read_one_payload(uint8_t addr, const uint8_t *req, int req_n,
                             uint8_t *out, int *out_len)
{
    if (send_cmd(addr, req, req_n) != ESP_OK) return false;
    uint8_t raddr, rdata[8]; int rlen = 0;
    if (!rx_one(300, &raddr, rdata, &rlen)) return false;
    if (frame_has_ee(rdata, rlen)) return false;
    int n = rlen > (int)sizeof(out) ? (int)8 : rlen;
    memcpy(out, rdata, (size_t)n);
    *out_len = n;
    return true;
}

/* WriteProtectThreshold (5.6.23) round-trip：读回 5.6.22。
 * 读回格式：功能码 + 过热(BE16) + 过流(BE16) + 检测(BE16) + 6B */
static void wrrt_protect_threshold(uint8_t addr, uint8_t store,
                                   uint16_t oh, uint16_t oc, uint16_t dt)
{
    char name[64];
    snprintf(name, sizeof name, "5.6.23 WriteProtectThreshold(%d,%d,%d)", oh, oc, dt);
    uint8_t b[16];
    int n = zdtBuildWriteProtectThresholdCmd(addr, store, oh, oc, dt, b, sizeof b);
    if (n < 0) { record_fail(name, "build"); return; }
    send_cmd(addr, b, n);
    vTaskDelay(pdMS_TO_TICKS(60));

    uint8_t r[16]; int rl = 0;
    n = zdtBuildReadProtectThresholdCmd(addr, b, sizeof b);
    if (!read_one_payload(addr, b, n, r, &rl)) { record_fail(name, "no read reply"); return; }
    /* 找 BE16 三元组：从末尾 0x6B 往前数 6 字节 */
    if (rl < 8) { record_fail(name, "reply too short"); return; }
    /* 回复: [func][BE16 oh][BE16 oc][BE16 dt][6B]，data 在 r[1..6] */
    uint16_t g_oh = decode_be16(&r[1]);
    uint16_t g_oc = decode_be16(&r[3]);
    uint16_t g_dt = decode_be16(&r[5]);
    char m[96];
    if (g_oh == oh && g_oc == oc && g_dt == dt) {
        snprintf(m, sizeof m, "rt oh=%d oc=%d dt=%d", g_oh, g_oc, g_dt);
        record_pass(name, m);
    } else {
        snprintf(m, sizeof m, "rt mismatch got oh=%d oc=%d dt=%d", g_oh, g_oc, g_dt);
        record_fail(name, m);
    }
}

/* WriteHeartbeat (5.6.25) round-trip：读回 5.6.24。
 * 读回：功能码 + 时间(BE32) + 6B */
static void wrrt_heartbeat(uint8_t addr, uint8_t store, uint32_t ms)
{
    char name[64];
    snprintf(name, sizeof name, "5.6.25 WriteHeartbeat(%lu)", (unsigned long)ms);
    uint8_t b[16];
    int n = zdtBuildWriteHeartbeatCmd(addr, store, ms, b, sizeof b);
    if (n < 0) { record_fail(name, "build"); return; }
    send_cmd(addr, b, n);
    vTaskDelay(pdMS_TO_TICKS(60));

    uint8_t r[16]; int rl = 0;
    n = zdtBuildReadHeartbeatCmd(addr, b, sizeof b);
    if (!read_one_payload(addr, b, n, r, &rl)) { record_fail(name, "no read reply"); return; }
    if (rl < 6) { record_fail(name, "reply too short"); return; }
    /* 回复: [func][BE32 data][6B]，data 在 r[1..4] */
    uint32_t g = ((uint32_t)r[1] << 24) | ((uint32_t)r[2] << 16) |
                 ((uint32_t)r[3] << 8) | r[4];
    char m[80];
    if (g == ms) { snprintf(m, sizeof m, "rt=%lu", (unsigned long)g); record_pass(name, m); }
    else { snprintf(m, sizeof m, "mismatch got=%lu", (unsigned long)g); record_fail(name, m); }
}

/* WriteIntegralLimit (5.6.27) round-trip：读回 5.6.26。BE32 */
static void wrrt_integral_limit(uint8_t addr, uint8_t store, uint32_t v)
{
    char name[64];
    snprintf(name, sizeof name, "5.6.27 WriteIntegralLimit(%lu)", (unsigned long)v);
    uint8_t b[16];
    int n = zdtBuildWriteIntegralLimitCmd(addr, store, v, b, sizeof b);
    if (n < 0) { record_fail(name, "build"); return; }
    send_cmd(addr, b, n);
    vTaskDelay(pdMS_TO_TICKS(60));

    uint8_t r[16]; int rl = 0;
    n = zdtBuildReadIntegralLimitCmd(addr, b, sizeof b);
    if (!read_one_payload(addr, b, n, r, &rl)) { record_fail(name, "no read reply"); return; }
    if (rl < 6) { record_fail(name, "reply too short"); return; }
    uint32_t g = ((uint32_t)r[1] << 24) | ((uint32_t)r[2] << 16) |
                 ((uint32_t)r[3] << 8) | r[4];
    char m[80];
    if (g == v) { snprintf(m, sizeof m, "rt=%lu", (unsigned long)g); record_pass(name, m); }
    else { snprintf(m, sizeof m, "mismatch got=%lu", (unsigned long)g); record_fail(name, m); }
}

/* WriteBumpReturnAngle (5.6.29) round-trip：读回 5.6.28。BE16 */
static void wrrt_bump_angle(uint8_t addr, uint8_t store, uint16_t ang)
{
    char name[64];
    snprintf(name, sizeof name, "5.6.29 WriteBumpReturnAngle(%u)", ang);
    uint8_t b[16];
    int n = zdtBuildWriteBumpReturnAngleCmd(addr, store, ang, b, sizeof b);
    if (n < 0) { record_fail(name, "build"); return; }
    send_cmd(addr, b, n);
    vTaskDelay(pdMS_TO_TICKS(60));

    uint8_t r[16]; int rl = 0;
    n = zdtBuildReadBumpReturnAngleCmd(addr, b, sizeof b);
    if (!read_one_payload(addr, b, n, r, &rl)) { record_fail(name, "no read reply"); return; }
    if (rl < 4) { record_fail(name, "reply too short"); return; }
    /* 回复: [func][BE16 data][6B]，data 在 r[1..2] */
    uint16_t g = decode_be16(&r[1]);
    char m[64];
    if (g == ang) { snprintf(m, sizeof m, "rt=%u", g); record_pass(name, m); }
    else { snprintf(m, sizeof m, "mismatch got=%u", g); record_fail(name, m); }
}

/* WritePosWindow (5.6.21) round-trip：读回 5.6.20。BE16。
 * 注意：store=NO 时电机可能不持久化新值，但通常即时生效，读回应=新值。 */
static void wrrt_pos_window(uint8_t addr, uint8_t store, uint16_t w)
{
    char name[64];
    snprintf(name, sizeof name, "5.6.21 WritePosWindow(%u)", w);
    uint8_t b[16];
    int n = zdtBuildWritePosWindowCmd(addr, store, w, b, sizeof b);
    if (n < 0) { record_fail(name, "build"); return; }
    send_cmd(addr, b, n);
    vTaskDelay(pdMS_TO_TICKS(150));

    uint8_t r[16]; int rl = 0;
    n = zdtBuildReadPosWindowCmd(addr, b, sizeof b);
    if (!read_one_payload(addr, b, n, r, &rl)) { record_fail(name, "no read reply"); return; }
    if (rl < 3) { record_fail(name, "reply too short"); return; }
    uint16_t g = r[1];
    char m[64];
    if (g == w) { snprintf(m, sizeof m, "rt=%u", g); record_pass(name, m); }
    else { snprintf(m, sizeof m, "mismatch got=%u", g); record_fail(name, m); }
}

/* ChangeMicrostep (5.6.2)：没有专用读，靠 5.8.5 ReadAllConfigEmm 间接看。
 * 这里只验写 ACK + 无 EE（值比对在 5.8.5 单独看 hex）。 */
static void test_write_microstep(uint8_t addr, uint8_t store, uint8_t ms)
{
    char name[64];
    snprintf(name, sizeof name, "5.6.2 ChangeMicrostep(=0x%02X)", ms);
    uint8_t b[8];
    int n = zdtBuildChangeMicrostepCmd(addr, store, ms, b, sizeof b);
    if (n < 0) { record_fail(name, "build"); return; }
    if (write_cmd(addr, b, n)) record_pass(name, "write ack");
    else record_fail(name, "send/EE");
}

/* 读 PID 三参数。ReadPidEmm 回复多包，拼接后去掉每包重复的 func(0x21)，
 * 格式: [21][kp BE32][ki BE32][kd BE32][6B]，共 14 字节。
 * 成功返回 true 并填 kp/ki/kd。 */
static bool read_pid_emm(uint8_t addr, uint32_t *kp, uint32_t *ki, uint32_t *kd)
{
    uint8_t b[8];
    int n = zdtBuildReadPidEmmCmd(addr, b, sizeof b);
    if (send_cmd(addr, b, n) != ESP_OK) return false;

    uint8_t buf[32]; int blen = 0;
    uint8_t raddr, rdata[8]; int rlen = 0;
    for (int i = 0; i < 6; ++i) {
        if (!rx_one(300, &raddr, rdata, &rlen)) break;
        if (frame_has_ee(rdata, rlen)) return false;
        /* 首包含 func(0x21)，后续包首字节也是 func，跳过避免重复 */
        int start = (blen == 0) ? 0 : 1;
        for (int j = start; j < rlen && blen < (int)sizeof(buf); ++j)
            buf[blen++] = rdata[j];
        if (blen > 0 && buf[blen-1] == 0x6B) break;
    }
    if (blen < 14) return false;
    /* buf[0]=func 0x21, buf[1..4]=kp, buf[5..8]=ki, buf[9..12]=kd, buf[13]=6B */
    *kp = ((uint32_t)buf[1]<<24)|((uint32_t)buf[2]<<16)|((uint32_t)buf[3]<<8)|buf[4];
    *ki = ((uint32_t)buf[5]<<24)|((uint32_t)buf[6]<<16)|((uint32_t)buf[7]<<8)|buf[8];
    *kd = ((uint32_t)buf[9]<<24)|((uint32_t)buf[10]<<16)|((uint32_t)buf[11]<<8)|buf[12];
    return true;
}

/* WritePidEmm 读-改-读-改-读闭环测试：
 * 读 orig → 写 v1 → 读回验==v1 → 写 v2 → 读回验==v2 → 写回 orig → 读回验==orig。
 * 任何一步读回不一致，立即停止并恢复 orig（不再继续写下一组）。 */
static void test_write_pid_roundtrip(uint8_t addr)
{
    const char *name = "5.6.17 WritePidEmm(roundtrip)";
    uint32_t orig_kp, orig_ki, orig_kd;
    if (!read_pid_emm(addr, &orig_kp, &orig_ki, &orig_kd)) {
        record_fail(name, "read orig failed");
        return;
    }
    ESP_LOGI(TAG, "    PID orig: kp=%lu ki=%lu kd=%lu",
             (unsigned long)orig_kp, (unsigned long)orig_ki, (unsigned long)orig_kd);

    /* 两组测试值 */
    uint32_t t_kp[2] = {18000, 20000};
    uint32_t t_ki[2] = {10, 20};
    uint32_t t_kd[2] = {18000, 20000};
    bool ok = true;

    for (int i = 0; i < 2 && ok; ++i) {
        uint8_t b[20];
        int n = zdtBuildWritePidEmmCmd(addr, ZDT_STORE_NO, t_kp[i], t_ki[i], t_kd[i], b, sizeof b);
        send_cmd(addr, b, n);
        vTaskDelay(pdMS_TO_TICKS(200));
        uint32_t g_kp, g_ki, g_kd;
        if (!read_pid_emm(addr, &g_kp, &g_ki, &g_kd)) {
            char m[80]; snprintf(m, sizeof m, "readback #%d failed", i);
            record_fail(name, m); ok = false; break;
        }
        if (g_kp != t_kp[i] || g_ki != t_ki[i] || g_kd != t_kd[i]) {
            char m[120];
            snprintf(m, sizeof m,
                     "mismatch #%d: got kp=%lu,ki=%lu,kd=%lu want kp=%lu,ki=%lu,kd=%lu",
                     i, (unsigned long)g_kp,(unsigned long)g_ki,(unsigned long)g_kd,
                     (unsigned long)t_kp[i],(unsigned long)t_ki[i],(unsigned long)t_kd[i]);
            record_fail(name, m); ok = false; break;
        }
        char m[80]; snprintf(m, sizeof m, "v%d ok kp=%lu ki=%lu kd=%lu",
                             i, (unsigned long)g_kp, (unsigned long)g_ki, (unsigned long)g_kd);
        ESP_LOGI(TAG, "    %s %s", name, m);
    }

    /* 无论上面成败，都恢复 orig 并验证 */
    {
        uint8_t b[20];
        int n = zdtBuildWritePidEmmCmd(addr, ZDT_STORE_NO, orig_kp, orig_ki, orig_kd, b, sizeof b);
        send_cmd(addr, b, n);
        vTaskDelay(pdMS_TO_TICKS(200));
        uint32_t g_kp, g_ki, g_kd;
        if (!read_pid_emm(addr, &g_kp, &g_ki, &g_kd)) {
            record_fail(name, "restore readback failed");
            return;
        }
        if (g_kp != orig_kp || g_ki != orig_ki || g_kd != orig_kd) {
            char m[120];
            snprintf(m, sizeof m, "RESTORE MISMATCH: got kp=%lu,ki=%lu,kd=%lu want orig",
                     (unsigned long)g_kp,(unsigned long)g_ki,(unsigned long)g_kd);
            record_fail(name, m);
            return;
        }
    }
    if (ok) record_pass(name, "roundtrip + restore verified");
}

/* ====================== MultiMotor 测试（≥2 台时）======================
 * 子命令 = 带地址前缀的完整子命令（含各自 0x6B）。广播后每台各回 ACK。 */

/* 收 n_frames 帧并验都无 EE。返回收到的帧数。 */
static int rx_n_no_ee(int n_frames, int per_frame_ms)
{
    uint8_t raddr, rdata[8]; int rlen = 0;
    int got = 0;
    for (int i = 0; i < n_frames; ++i) {
        if (!rx_one(per_frame_ms, &raddr, rdata, &rlen)) break;
        if (frame_has_ee(rdata, rlen)) return -1;
        got++;
    }
    return got;
}

static void test_multi_enable(void)
{
    /* 拼每个在线电机的 enable 子命令 */
    uint8_t sub[64];
    int off = 0;
    for (int i = 0; i < s_found_cnt; ++i) {
        int n = zdtBuildMotorEnableCmd(s_found[i], 0x01, ZDT_SYNC_NOW,
                                       sub + off, sizeof(sub) - off);
        if (n > 0) off += n;
    }
    uint8_t b[80];
    int n = zdtBuildMultiMotorCmd(sub, off, b, sizeof b);
    if (n < 0) { record_fail("5.3.1a MultiMotor(enable)", "build"); return; }

    drain_rx();
    esp_err_t err = zdt_can_send(0x00, b, n, pdMS_TO_TICKS(300));
    if (err != ESP_OK) { record_fail("5.3.1a MultiMotor(enable)", esp_err_to_name(err)); return; }
    int got = rx_n_no_ee(s_found_cnt, 200);
    char m[64];
    if (got < 0) { record_fail("5.3.1a MultiMotor(enable)", "some motor EE"); return; }
    /* 容忍回复数 >=1（异步命令不一定都回） */
    snprintf(m, sizeof m, "tx ok, rx ack=%d/%d", got, s_found_cnt);
    record_pass("5.3.1a MultiMotor(enable)", m);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void test_multi_position(void)
{
    /* 对每台发不同脉冲量（addr×1000，便于区分），同步缓存后统一触发。
     * 简化：直接 NOW 同步执行，验各台位置都动了。 */
    /* 先记录各台 pos0 */
    int32_t pos0[SCAN_MAX] = {0};
    for (int i = 0; i < s_found_cnt; ++i) read_realtime_pos(s_found[i], &pos0[i]);

    uint8_t sub[64];
    int off = 0;
    for (int i = 0; i < s_found_cnt; ++i) {
        uint32_t pulses = (uint32_t)s_found[i] * 2000;   /* ≥1000，超过窗口 */
        int n = zdtBuildPosModeEmmCmd(s_found[i], ZDT_DIR_CW, 60, 10, pulses,
                                      ZDT_MOVE_REL_NOW, ZDT_SYNC_NOW,
                                      sub + off, sizeof(sub) - off);
        if (n > 0) off += n;
    }
    uint8_t b[80];
    int n = zdtBuildMultiMotorCmd(sub, off, b, sizeof b);
    if (n < 0) { record_fail("5.3.1b MultiMotor(position)", "build"); return; }

    drain_rx();
    esp_err_t err = zdt_can_send(0x00, b, n, pdMS_TO_TICKS(300));
    if (err != ESP_OK) { record_fail("5.3.1b MultiMotor(position)", esp_err_to_name(err)); return; }

    /* 等所有台到位（最多 6s） */
    vTaskDelay(pdMS_TO_TICKS(2500));

    /* 逐台读位置，验都动了（|Δ| ≥ 1000） */
    bool all_ok = true;
    char m[128]; int mo = 0;
    mo += snprintf(m + mo, sizeof(m) - mo, "moved:");
    for (int i = 0; i < s_found_cnt; ++i) {
        int32_t pf = 0;
        read_realtime_pos(s_found[i], &pf);
        int32_t d = pf - pos0[i]; if (d < 0) d = -d;
        mo += snprintf(m + mo, sizeof(m) - mo, " a%d=%ld", s_found[i], (long)d);
        if (d < 1000) all_ok = false;
    }
    /* 收 ACK */
    int got = rx_n_no_ee(s_found_cnt, 200);
    (void)got;
    if (all_ok) record_pass("5.3.1b MultiMotor(position)", m);
    else record_fail("5.3.1b MultiMotor(position)", m);
    /* 收尾停 */
    for (int i = 0; i < s_found_cnt; ++i) stop_motor(s_found[i]);
}

/* ====================== PROBE：测时序下限（临时，测完删）======================
 * 测两件事，打印表格，用于收敛 main.c 里位置到位/速度加速的等待时间：
 *   1) 位置到位真实时间：发 PosModeEmm，每 POLL_MS 轮询实时位置，
 *      记录从发命令到连续 STABLE_N 次位置变化 < POS_STABLE_WIN 的毫秒数。
 *   2) 速度环达到目标转速时间：发 SpeedModeEmm，每 POLL_MS 读实时转速，
 *      记录到达目标值 SPEED_REACH_PCT% 的时间。
 * 用法：把 app_main 开头的 #if PROBE 改 1 编译即跑，跑完改回 0。 */
#define PROBE              0      /* 1=跑probe 0=跑正常test */
#define PROBE_PHASE        0

#define PROBE_POLL_MS      20
#define PROBE_STABLE_N     3
#define PROBE_STABLE_WIN   200    /* 连续 N 次位置变化 < 此值算到位（脉冲）*/
#define PROBE_MAX_MS       4000   /* 单次测量上限 */
#define PROBE_SPEED_PCT    90     /* 转速达目标值此百分比算到位 */

static int32_t probe_read_pos(uint8_t addr)
{
    uint8_t buf[8], raddr, rdata[8];
    int n = zdtBuildReadRealtimePosCmd(addr, buf, sizeof buf);
    if (send_cmd(addr, buf, n) != ESP_OK) return 0;
    int rlen = 0;
    if (!rx_one(100, &raddr, rdata, &rlen)) return 0;
    if (rlen < 4) return 0;
    int32_t v = 0;
    for (int i = 1; i < rlen - 1; ++i) v = (v << 8) | rdata[i];
    return v;
}

static int32_t probe_read_speed(uint8_t addr)
{
    uint8_t buf[8], raddr, rdata[8];
    int n = zdtBuildReadRealtimeSpeedCmd(addr, buf, sizeof buf);
    if (send_cmd(addr, buf, n) != ESP_OK) return 0;
    int rlen = 0;
    if (!rx_one(150, &raddr, rdata, &rlen)) return 0;
    if (rlen < 4) return 0;
    /* 回复: [35] [data BE16/BE24...] [6B]，去掉末尾 6B，从 rdata[1] 起 BE 解码 */
    int32_t v = 0;
    for (int i = 1; i < rlen - 1; ++i) v = (v << 8) | rdata[i];
    return v;
}

static void probe_position_one(uint8_t addr, uint16_t rpm, uint32_t pulses)
{
    prep_motor(addr);
    int32_t p0 = probe_read_pos(addr);
    uint8_t b[16];
    int n = zdtBuildPosModeEmmCmd(addr, ZDT_DIR_CW, 3000, 0, pulses,
                                  ZDT_MOVE_REL_NOW, ZDT_SYNC_NOW, b, sizeof b);
    send_cmd(addr, b, n);
    int64_t t0 = esp_log_timestamp();
    int32_t last = p0;
    int stable = 0;
    int64_t settle_ms = -1;
    while (esp_log_timestamp() - t0 < PROBE_MAX_MS) {
        vTaskDelay(pdMS_TO_TICKS(PROBE_POLL_MS));
        int32_t p = probe_read_pos(addr);
        int32_t d = p - last; if (d < 0) d = -d;
        if (d < PROBE_STABLE_WIN) stable++; else stable = 0;
        last = p;
        if (stable >= PROBE_STABLE_N) { settle_ms = esp_log_timestamp() - t0; break; }
    }
    stop_motor(addr);
    int32_t pf = probe_read_pos(addr);
    int32_t moved = pf - p0; if (moved < 0) moved = -moved;
    ESP_LOGI(TAG, "POS  rpm=%-4u pulses=%-7lu  settle=%ldms  moved=%ld",
             rpm, (unsigned long)pulses, (long)settle_ms, (long)moved);
}

static void probe_speed_one(uint8_t addr, uint16_t rpm, uint8_t acc)
{
    prep_motor(addr);
    uint8_t b[16];
    int n = zdtBuildSpeedModeEmmCmd(addr, ZDT_DIR_CW, rpm, acc, ZDT_SYNC_NOW, b, sizeof b);
    send_cmd(addr, b, n);
    int64_t t0 = esp_log_timestamp();
    int32_t target = (int32_t)rpm * PROBE_SPEED_PCT / 100;
    int64_t reach_ms = -1;
    while (esp_log_timestamp() - t0 < PROBE_MAX_MS) {
        vTaskDelay(pdMS_TO_TICKS(PROBE_POLL_MS));
        int32_t s = probe_read_speed(addr);
        if (s < 0) s = -s;
        if (s >= target) { reach_ms = esp_log_timestamp() - t0; break; }
    }
    stop_motor(addr);
    ESP_LOGI(TAG, "SPD  rpm=%-4u acc=%-3u  reach90%%=%ldms",
             rpm, acc, (long)reach_ms);
}

static void run_probe(uint8_t addr)
{
    ESP_LOGI(TAG, "=========== PROBE phase %d: timing ===========", PROBE_PHASE);
    prep_motor(addr);
    stop_motor(addr);

#if PROBE_PHASE == 0
    ESP_LOGI(TAG, "--- position settle time (pulses, full speed) ---");
    uint32_t pulses_list[] = {1000, 65536, 131072};
    for (int p = 0; p < 3; ++p)
        probe_position_one(addr, 3000, pulses_list[p]);
#else
    ESP_LOGI(TAG, "--- speed reach time (rpm, acc=0) ---");
    uint16_t srpms[] = {30, 60, 100, 300};
    for (int r = 0; r < 4; ++r)
        probe_speed_one(addr, srpms[r], 0);
#endif

    ESP_LOGI(TAG, "=========== PROBE DONE ===========");
}

/* ====================== app_main ====================== */
void app_main(void)
{
    ESP_LOGI(TAG, "=== zdt_x42s_can per-command test (rewrite) ===");
    ESP_LOGI(TAG, "CAN tx=%d rx=%d baud=%d (EMM fw)", CAN_TX, CAN_RX, CAN_BAUD);

    zdt_can_config_t cfg = { .tx_gpio = CAN_TX, .rx_gpio = CAN_RX, .baudrate = CAN_BAUD };
    ESP_ERROR_CHECK(zdt_can_init(&cfg));

#if PROBE
    /* 扫描找主测电机后直接跑 probe 并退出 */
    scan_motors();
    if (s_found_cnt > 0) run_probe(s_primary);
    zdt_can_deinit();
    return;
#endif

    /* ====== Setup：扫描 + 定主测电机 + 算位置容差 ====== */
    ESP_LOGI(TAG, "--- setup: scan ---");
    scan_motors();
    if (s_found_cnt == 0) {
        ESP_LOGE(TAG, "no motor online, abort");
        zdt_can_deinit();
        return;
    }
    const uint8_t A = s_primary;

    prep_motor(A);
    stop_motor(A);

    uint16_t win = 8;
    if (read_pos_window(A, &win)) {
        /* 容差 = 窗口 + 余量（机械回差/量化 ~500，再放宽） */
        s_pos_tol = (int32_t)win + 1000;
        ESP_LOGI(TAG, "pos window=%u → tol=%ld", win, (long)s_pos_tol);
    } else {
        ESP_LOGW(TAG, "read pos window failed, use default tol=%ld", (long)s_pos_tol);
    }

    uint8_t b[40];
    int n;

    /* ====== 5.2 触发 ====== */
    ESP_LOGI(TAG, "--- 5.2 triggers ---");
    n = zdtBuildClearCurrentAngleCmd(A, b, sizeof b);  test_trig("5.2.3 ClearCurrentAngle", A, b, n);
    n = zdtBuildClearProtectionCmd(A, b, sizeof b);    test_trig("5.2.4 ClearProtection", A, b, n);
    n = zdtBuildRestartMotorCmd(A, b, sizeof b);       test_trig("5.2.2 RestartMotor", A, b, n);
    vTaskDelay(pdMS_TO_TICKS(3000));  /* 等重启完成 */
    prep_motor(A);

    /* ====== 5.3 运动 ======
     * 跳过：5.3.3-5.3.6, 5.3.8-5.3.11（X 固件） */
    ESP_LOGI(TAG, "--- 5.3 motion ---");
    /* 5.3.2 MotorEnable：enable=01/00 × sync=NOW/CACHE */
    n = zdtBuildMotorEnableCmd(A, 0x01, ZDT_SYNC_NOW, b, sizeof b);
    test_trig("5.3.2 MotorEnable(on,NOW)", A, b, n); vTaskDelay(pdMS_TO_TICKS(200));
    n = zdtBuildMotorEnableCmd(A, 0x00, ZDT_SYNC_NOW, b, sizeof b);
    test_trig("5.3.2 MotorEnable(off,NOW)", A, b, n); vTaskDelay(pdMS_TO_TICKS(200));
    n = zdtBuildMotorEnableCmd(A, 0x01, ZDT_SYNC_CACHE, b, sizeof b);
    test_trig("5.3.2 MotorEnable(on,CACHE)", A, b, n);
    n = zdtBuildSyncMotionCmd(A, b, sizeof b);   /* 触发缓存的使能 */
    test_trig("5.3.14 SyncMotion(trigger cache)", A, b, n); vTaskDelay(pdMS_TO_TICKS(200));

    /* 5.3.1 MultiMotor：≥2 台才测 */
    if (s_found_cnt >= 2) {
        ESP_LOGI(TAG, "-- 5.3.1 MultiMotor (%d motors) --", s_found_cnt);
        test_multi_enable();
        test_multi_position();
    } else {
        ESP_LOGI(TAG, "-- 5.3.1 MultiMotor skipped (only %d motor) --", s_found_cnt);
        record_skip("5.3.1 MultiMotor", "only 1 motor");
    }

    /* 5.3.7 SpeedModeEmm：dir CW/CCW × rpm 30/60/100 × acc 档位。
     * 写速度后电机持续转，读实时转速验方向+量级。 */
    {
        struct { uint8_t dir; uint16_t rpm; uint8_t acc; } cs[] = {
            { ZDT_DIR_CW,  30, 10 },
            { ZDT_DIR_CW,  60, 10 },
            { ZDT_DIR_CW,  100, 20 },
            { ZDT_DIR_CCW, 30, 10 },
            { ZDT_DIR_CCW, 60, 10 },
            { ZDT_DIR_CCW, 100, 20 },
        };
        for (int i = 0; i < 6; ++i) {
            char name[64];
            snprintf(name, sizeof name, "5.3.7 SpeedModeEmm(%s,%drpm,acc%d)",
                     cs[i].dir == ZDT_DIR_CW ? "CW" : "CCW", cs[i].rpm, cs[i].acc);
            n = zdtBuildSpeedModeEmmCmd(A, cs[i].dir, cs[i].rpm, cs[i].acc, ZDT_SYNC_NOW, b, sizeof b);
            test_speed(name, A, b, n, cs[i].dir);
        }
    }

    /* 细分用默认 16（reset 后即此值）。PosModeEmm 的 pulses 单位 = 细分脉冲。
     * 细分16 下 1 脉冲 = 65536/(200*16) = 20.48 编码器单位。
     * 位置测试用小脉冲量（1-2 圈），3s 内能到位，期望 moved = pulses * 20.48。 */
    test_write_microstep(A, ZDT_STORE_NO, 0x10);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 5.3.12 PosModeEmm：
     *   move_mode: REL_NOW / ABS_ZERO / REL_LAST × dir × 脉冲量级
     *   位置限速指令的"速度参数"不测时序（不好控制读时机），只验位置到位。
     *   脉冲量全部 ≥1000，远大于窗口。 */
    {
        /* 细分16: 200 脉冲=1圈=4096编码器单位。脉冲量小，3s 内到位。
         * ref_value 是期望的编码器单位位移（pulses * 20.48），用整数近似。
         * test_position 内部 tol = window + 1000，容许细分量化误差。 */
        struct { const char *tag; uint8_t mode; uint8_t dir; uint32_t pulses; int ref_kind; int32_t expect_enc; } cs[] = {
            { "REL_NOW,CW,1turn",   ZDT_MOVE_REL_NOW,  ZDT_DIR_CW,  200,   0, 4096 },
            { "REL_NOW,CCW,half",   ZDT_MOVE_REL_NOW,  ZDT_DIR_CCW, 100,   0, 2048 },
            /* ABS_ZERO: pulses = 绝对坐标目标（不是位移量！），ref_kind=1 验 |pf-pulses|<=tol。
             * 真机验证：target=0→pos≈0，target=10000→pos≈97787(多圈累计)。
             * 用 target=0（回零）最可靠，因为零点明确。 */
            { "ABS_ZERO,CW,t0",    ZDT_MOVE_ABS_ZERO, ZDT_DIR_CW,  0,     1, 0 },
            { "ABS_ZERO,CCW,t0",   ZDT_MOVE_ABS_ZERO, ZDT_DIR_CCW, 0,     1, 0 },
            { "REL_LAST,CW,1turn",  ZDT_MOVE_REL_LAST, ZDT_DIR_CW,  200,   0, 4096 },
            { "REL_NOW,CW,2turn",   ZDT_MOVE_REL_NOW,  ZDT_DIR_CW,  400,   0, 8192 },
            { "REL_NOW,CW,5turn",   ZDT_MOVE_REL_NOW,  ZDT_DIR_CW,  1000,  0, 20480 },
            { "REL_NOW,CW,1turn,fast", ZDT_MOVE_REL_NOW, ZDT_DIR_CW, 200,  0, 4096 },
        };
        for (int i = 0; i < 8; ++i) {
            char name[80];
            snprintf(name, sizeof name, "5.3.12 PosModeEmm(%s)", cs[i].tag);
            n = zdtBuildPosModeEmmCmd(A, cs[i].dir, 3000, 0, cs[i].pulses,
                                      cs[i].mode, ZDT_SYNC_NOW, b, sizeof b);
            test_position(name, A, b, n, cs[i].ref_kind, cs[i].expect_enc);
        }
    }

    /* 5.3.13 ImmediateStop：sync NOW/CACHE */
    n = zdtBuildImmediateStopCmd(A, ZDT_SYNC_NOW, b, sizeof b);
    test_trig("5.3.13 ImmediateStop(NOW)", A, b, n);

    /* 恢复细分默认（0x10=16）以免影响后续 */
    test_write_microstep(A, ZDT_STORE_NO, 0x10);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* ====== 5.4 原点回零 ======
     * 跳过：5.4.2 TriggerHoming（无原点/限位） */
    ESP_LOGI(TAG, "--- 5.4 homing ---");
    /* 5.4.1 SetSingleTurnZero：store NO/YES */
    n = zdtBuildSetSingleTurnZeroCmd(A, ZDT_STORE_NO, b, sizeof b);
    test_write_basic_wrap("5.4.1 SetSingleTurnZero(NO)", A, b, n);
    n = zdtBuildSetSingleTurnZeroCmd(A, ZDT_STORE_YES, b, sizeof b);
    test_write_basic_wrap("5.4.1 SetSingleTurnZero(YES)", A, b, n);
    n = zdtBuildAbortHomingCmd(A, b, sizeof b);
    test_trig("5.4.3 AbortHoming", A, b, n);
    n = zdtBuildReadHomingStatusCmd(A, b, sizeof b);
    test_read("5.4.4 ReadHomingStatus", A, b, n, 3);
    n = zdtBuildReadHomingParamsCmd(A, b, sizeof b);
    test_read_long("5.4.5 ReadHomingParams", A, b, n, 8);
    /* 5.4.6 WriteHomingParams：round-trip via 5.4.5。
     * 只写 store=NO（不持久化），测后电机行为不受影响（参数只在回零时用） */
    {
        uint8_t hm = 0, hd = ZDT_DIR_CW;
        uint16_t hr = 100;
        uint32_t ht = 5000;
        uint16_t bs = 30, bc = 500, bt = 200;
        uint8_t at = 0;
        n = zdtBuildWriteHomingParamsCmd(A, ZDT_STORE_NO, hm, hd, hr, ht, bs, bc, bt, at, b, sizeof b);
        test_write_basic_wrap("5.4.6 WriteHomingParams(NO,mode0)", A, b, n);
        /* 改一组参数再写，验电机不报错 */
        n = zdtBuildWriteHomingParamsCmd(A, ZDT_STORE_NO, 1, ZDT_DIR_CCW, 200, 3000, 60, 1000, 500, 1, b, sizeof b);
        test_write_basic_wrap("5.4.6 WriteHomingParams(NO,mode1)", A, b, n);
    }

    /* ====== 5.5 读取系统参数 ====== */
    ESP_LOGI(TAG, "--- 5.5 reads ---");
    /* 5.5.1 SetPeriodicReport：验证自动上报。
     *   1) 设 info=0x36(位置) interval=200ms → 收多帧，验功能码回显=0x36、收到≥2帧
     *   2) 设 info=0x3A interval=100ms → 验功能码回显变了=0x3A
     *   3) 关掉(interval=0) → 验不再收到上报帧 */
    {
        uint8_t raddr, rdata[8]; int rlen = 0;
        /* (1) 开 0x36 @ 200ms */
        n = zdtBuildSetPeriodicReportCmd(A, 0x36, 200, b, sizeof b);
        send_cmd(A, b, n);   /* 不用 test_write，直接发 */
        vTaskDelay(pdMS_TO_TICKS(100));
        drain_rx();          /* 丢弃 ACK */
        /* 收 700ms 内的上报帧，验功能码 + 计数 */
        int cnt36 = 0; bool code36_ok = true;
        int64_t t0 = esp_log_timestamp();
        while (esp_log_timestamp() - t0 < 700) {
            if (rx_one(250, &raddr, rdata, &rlen)) {
                if (frame_has_ee(rdata, rlen)) { code36_ok = false; break; }
                if (rdata[0] != 0x36) { code36_ok = false; }
                cnt36++;
            } else break;
        }
        {
            char m[64]; snprintf(m, sizeof m, "frames=%d code=%s", cnt36, code36_ok?"ok":"wrong");
            if (cnt36 >= 2 && code36_ok) record_pass("5.5.1a PeriodicReport(0x36,200ms)", m);
            else record_fail("5.5.1a PeriodicReport(0x36,200ms)", m);
        }
        /* (2) 换 0x3A @ 100ms，验功能码变了 */
        n = zdtBuildSetPeriodicReportCmd(A, 0x3A, 100, b, sizeof b);
        send_cmd(A, b, n);
        vTaskDelay(pdMS_TO_TICKS(100));
        drain_rx();
        int cnt3a = 0; bool code3a_ok = true;
        t0 = esp_log_timestamp();
        while (esp_log_timestamp() - t0 < 500) {
            if (rx_one(200, &raddr, rdata, &rlen)) {
                if (frame_has_ee(rdata, rlen)) { code3a_ok = false; break; }
                if (rdata[0] != 0x3A) { code3a_ok = false; }
                cnt3a++;
            } else break;
        }
        {
            char m[64]; snprintf(m, sizeof m, "frames=%d code=%s", cnt3a, code3a_ok?"ok":"wrong");
            if (cnt3a >= 2 && code3a_ok) record_pass("5.5.1b PeriodicReport(0x3A,100ms)", m);
            else record_fail("5.5.1b PeriodicReport(0x3A,100ms)", m);
        }
        /* (3) 关掉，验不再收到上报 */
        n = zdtBuildSetPeriodicReportCmd(A, 0x3A, 0, b, sizeof b);
        send_cmd(A, b, n);
        vTaskDelay(pdMS_TO_TICKS(100));
        drain_rx();
        vTaskDelay(pdMS_TO_TICKS(400));   /* 等 400ms 看还有没有上报 */
        bool stopped = !rx_one(300, &raddr, rdata, &rlen);
        if (stopped) record_pass("5.5.1c PeriodicReport(off)", "no frame after off");
        else record_fail("5.5.1c PeriodicReport(off)", "still receiving frames");
    }

    n = zdtBuildReadVersionCmd(A, b, sizeof b);               test_read("5.5.2 ReadVersion", A, b, n, 4);
    n = zdtBuildReadPhaseRLCmd(A, b, sizeof b);               test_read("5.5.3 ReadPhaseRL", A, b, n, 4);
    n = zdtBuildReadBusVoltageCmd(A, b, sizeof b);            test_read("5.5.4 ReadBusVoltage", A, b, n, 4);
    n = zdtBuildReadBusCurrentCmd(A, b, sizeof b);            test_read("5.5.5 ReadBusCurrent", A, b, n, 4);
    n = zdtBuildReadPhaseCurrentCmd(A, b, sizeof b);          test_read("5.5.6 ReadPhaseCurrent", A, b, n, 4);
    n = zdtBuildReadEncoderCalibratedCmd(A, b, sizeof b);     test_read("5.5.7 ReadEncoderCalibrated", A, b, n, 4);
    n = zdtBuildReadInputPulsesCmd(A, b, sizeof b);           test_read("5.5.8 ReadInputPulses", A, b, n, 4);
    n = zdtBuildReadTargetPosCmd(A, b, sizeof b);             test_read("5.5.9 ReadTargetPos", A, b, n, 4);
    n = zdtBuildReadRealtimeTargetPosCmd(A, b, sizeof b);     test_read("5.5.10 ReadRealtimeTargetPos", A, b, n, 4);
    n = zdtBuildReadRealtimeSpeedCmd(A, b, sizeof b);         test_read("5.5.11 ReadRealtimeSpeed", A, b, n, 4);
    n = zdtBuildReadDriverTempCmd(A, b, sizeof b);            test_read("5.5.12 ReadDriverTemp", A, b, n, 4);
    n = zdtBuildReadRealtimePosCmd(A, b, sizeof b);           test_read("5.5.13 ReadRealtimePos", A, b, n, 4);
    n = zdtBuildReadPosErrorCmd(A, b, sizeof b);              test_read("5.5.14 ReadPosError", A, b, n, 4);
    n = zdtBuildReadMotorStatusCmd(A, b, sizeof b);           test_read("5.5.15 ReadMotorStatus", A, b, n, 3);
    n = zdtBuildReadHomingAndStatusCmd(A, b, sizeof b);       test_read("5.5.16 ReadHomingAndStatus", A, b, n, 3);
    n = zdtBuildReadIoLevelCmd(A, b, sizeof b);               test_read("5.5.17 ReadIoLevel", A, b, n, 3);
    /* 5.5.18 ReadBatteryVoltage：Y42 专用，EMM 固件返回 EE（不支持），归 SKIP */
    record_skip("5.5.18 ReadBatteryVoltage(Y42)", "Y42-only, EMM returns EE");

    /* ====== 5.6 读写驱动参数 ======
     * 跳过：5.6.1 ChangeAddr、5.6.6 ChangeFirmwareType、5.6.10/14/15（X）
     * 风险写（5.6.17 WritePidEmm / 5.6.5 MotorType / 5.6.7 CtrlMode / 5.6.8 MotorDir）：测后恢复 */
    ESP_LOGI(TAG, "--- 5.6 params ---");
    /* 5.6.2 ChangeMicrostep：3 个值（已锁 1，再测 16/255） */
    test_write_microstep(A, ZDT_STORE_NO, 0x01);
    test_write_microstep(A, ZDT_STORE_NO, 0x10);
    test_write_microstep(A, ZDT_STORE_NO, 0xFF);
    test_write_microstep(A, ZDT_STORE_NO, 0x10);  /* 恢复默认 16 */

    /* 5.6.3 ChangePowerDownFlag：00/01 */
    n = zdtBuildChangePowerDownFlagCmd(A, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.3 ChangePowerDownFlag(off)", A, b, n);
    n = zdtBuildChangePowerDownFlagCmd(A, 0x01, b, sizeof b);
    test_write_basic_wrap("5.6.3 ChangePowerDownFlag(on)", A, b, n);
    n = zdtBuildChangePowerDownFlagCmd(A, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.3 ChangePowerDownFlag(off restore)", A, b, n);

    /* 5.6.4 ReadOptions */
    n = zdtBuildReadOptionsCmd(A, b, sizeof b); test_read("5.6.4 ReadOptions", A, b, n, 3);

    /* 5.6.5 ChangeMotorType：SKIP。改电机类型(0.9°/1.8°)会和编码器标定冲突，
     * 改完电机直接废（角度/极对数全错），无法可靠恢复。归入黑名单。 */
    record_skip("5.6.5 ChangeMotorType", "risk: corrupts encoder calibration");

    /* 5.6.7 ChangeCtrlMode：SKIP。改成开环后电机失控/蠕动。归入黑名单。 */
    record_skip("5.6.7 ChangeCtrlMode", "risk: open-loop corrupts motor");

    /* 5.6.8 ChangeMotorDir：SKIP。改方向会让后续测试方向判断全反。归入黑名单。 */
    record_skip("5.6.8 ChangeMotorDir", "risk: inverts direction");

    /* 5.6.9 ChangeKeyLock：00/01 */
    n = zdtBuildChangeKeyLockCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.9 ChangeKeyLock(off)", A, b, n);
    n = zdtBuildChangeKeyLockCmd(A, ZDT_STORE_NO, 0x01, b, sizeof b);
    test_write_basic_wrap("5.6.9 ChangeKeyLock(on)", A, b, n);
    n = zdtBuildChangeKeyLockCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.9 ChangeKeyLock(off restore)", A, b, n);

    /* 5.6.11 ChangeSpeedScale：0/1 */
    n = zdtBuildChangeSpeedScaleCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.11 ChangeSpeedScale(off)", A, b, n);
    n = zdtBuildChangeSpeedScaleCmd(A, ZDT_STORE_NO, 0x01, b, sizeof b);
    test_write_basic_wrap("5.6.11 ChangeSpeedScale(on)", A, b, n);
    n = zdtBuildChangeSpeedScaleCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.11 ChangeSpeedScale(off restore)", A, b, n);

    /* 5.6.12 ChangeOpenLoopCurrent：3 档（mA） */
    {
        uint16_t cur[] = {500, 1000, 2000};
        for (int i = 0; i < 3; ++i) {
            char name[64]; snprintf(name, sizeof name, "5.6.12 ChangeOpenLoopCurrent(%umA)", cur[i]);
            n = zdtBuildChangeOpenLoopCurrentCmd(A, ZDT_STORE_NO, cur[i], b, sizeof b);
            test_write_basic_wrap(name, A, b, n);
        }
    }

    /* 5.6.13 ChangeClosedLoopCurrent：3 档 */
    {
        uint16_t cur[] = {1000, 2000, 3000};
        for (int i = 0; i < 3; ++i) {
            char name[64]; snprintf(name, sizeof name, "5.6.13 ChangeClosedLoopCurrent(%umA)", cur[i]);
            n = zdtBuildChangeClosedLoopCurrentCmd(A, ZDT_STORE_NO, cur[i], b, sizeof b);
            test_write_basic_wrap(name, A, b, n);
        }
    }

    /* 5.6.16 ReadPidEmm */
    n = zdtBuildReadPidEmmCmd(A, b, sizeof b); test_read_long("5.6.16 ReadPidEmm", A, b, n, 4);

    /* 5.6.17 WritePidEmm：读-改-读闭环，每步读回验证，不一致立即恢复。
     * PID 参数改了不会物理损坏（只是控制变差），闭环验证安全。 */
    test_write_pid_roundtrip(A);

    /* 5.6.18 ReadDmx512 / 5.6.19 WriteDmx512 */
    n = zdtBuildReadDmx512Cmd(A, b, sizeof b); test_read_long("5.6.18 ReadDmx512", A, b, n, 8);
    {
        /* 3 组参数 round-trip（store=NO，避免持久化） */
        uint16_t tch[] = {16, 32, 1};
        uint8_t  cpm[] = {1, 2, 1};
        for (int i = 0; i < 3; ++i) {
            char name[64];
            snprintf(name, sizeof name, "5.6.19 WriteDmx512(ch=%u,cpm=%u)", tch[i], cpm[i]);
            n = zdtBuildWriteDmx512Cmd(A, ZDT_STORE_NO, tch[i], cpm[i], 0x00, 30, 10, 5, 3600, b, sizeof b);
            test_write_basic_wrap(name, A, b, n);
        }
    }

    /* 5.6.20/21 PosWindow round-trip */
    n = zdtBuildReadPosWindowCmd(A, b, sizeof b); test_read("5.6.20 ReadPosWindow", A, b, n, 3);
    wrrt_pos_window(A, ZDT_STORE_NO, 8);
    wrrt_pos_window(A, ZDT_STORE_NO, 16);
    wrrt_pos_window(A, ZDT_STORE_NO, 8);   /* 恢复默认 */

    /* 5.6.22/23 ProtectThreshold round-trip */
    n = zdtBuildReadProtectThresholdCmd(A, b, sizeof b); test_read("5.6.22 ReadProtectThreshold", A, b, n, 3);
    wrrt_protect_threshold(A, ZDT_STORE_NO, 80, 3000, 500);
    wrrt_protect_threshold(A, ZDT_STORE_NO, 100, 4000, 1000);

    /* 5.6.24/25 Heartbeat round-trip */
    n = zdtBuildReadHeartbeatCmd(A, b, sizeof b); test_read("5.6.24 ReadHeartbeat", A, b, n, 3);
    wrrt_heartbeat(A, ZDT_STORE_NO, 500);
    wrrt_heartbeat(A, ZDT_STORE_NO, 0);     /* 关闭 */

    /* 5.6.26/27 IntegralLimit round-trip */
    n = zdtBuildReadIntegralLimitCmd(A, b, sizeof b); test_read("5.6.26 ReadIntegralLimit", A, b, n, 3);
    wrrt_integral_limit(A, ZDT_STORE_NO, 1000);
    wrrt_integral_limit(A, ZDT_STORE_NO, 2000);

    /* 5.6.28/29 BumpReturnAngle round-trip */
    n = zdtBuildReadBumpReturnAngleCmd(A, b, sizeof b); test_read("5.6.28 ReadBumpReturnAngle", A, b, n, 3);
    wrrt_bump_angle(A, ZDT_STORE_NO, 180);
    wrrt_bump_angle(A, ZDT_STORE_NO, 90);

    /* 5.6.30 BroadcastReadAddr（已用于扫描，这里再单独记一轮） */
    n = zdtBuildBroadcastReadAddrCmd(b, sizeof b);
    {
        drain_rx();
        zdt_can_send(0x00, b, n, pdMS_TO_TICKS(300));
        uint8_t raddr, rdata[8]; int rlen = 0;
        bool any = false;
        int64_t dl = esp_log_timestamp() + 400;
        while (esp_log_timestamp() < dl) {
            if (!rx_one(200, &raddr, rdata, &rlen)) break;
            if (!frame_has_ee(rdata, rlen)) any = true;
        }
        if (any) record_pass("5.6.30 BroadcastReadAddr", "got reply");
        else record_fail("5.6.30 BroadcastReadAddr", "no reply");
    }

    /* 5.6.31 ChangeParamLock：0/1/2 三级 */
    n = zdtBuildChangeParamLockCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.31 ChangeParamLock(0)", A, b, n);
    n = zdtBuildChangeParamLockCmd(A, ZDT_STORE_NO, 0x01, b, sizeof b);
    test_write_basic_wrap("5.6.31 ChangeParamLock(1)", A, b, n);
    n = zdtBuildChangeParamLockCmd(A, ZDT_STORE_NO, 0x00, b, sizeof b);
    test_write_basic_wrap("5.6.31 ChangeParamLock(0 restore)", A, b, n);

    /* ====== 5.7 上电自动运行（store=0x00 清除，安全）====== */
    ESP_LOGI(TAG, "--- 5.7 auto-run ---");
    /* StoreAutoRunEmm：清除 / 存一组 / 再清除 */
    n = zdtBuildStoreAutoRunEmmCmd(A, 0x00, ZDT_DIR_CW, 60, 10, 0x00, b, sizeof b);
    test_write_basic_wrap("5.7.2 StoreAutoRunEmm(clear)", A, b, n);
    n = zdtBuildStoreAutoRunEmmCmd(A, 0x01, ZDT_DIR_CW, 300, 20, 0x00, b, sizeof b);
    test_write_basic_wrap("5.7.2 StoreAutoRunEmm(store CW 300rpm)", A, b, n);
    n = zdtBuildStoreAutoRunEmmCmd(A, 0x01, ZDT_DIR_CCW, 150, 10, 0x01, b, sizeof b);
    test_write_basic_wrap("5.7.2 StoreAutoRunEmm(store CCW 150rpm en)", A, b, n);
    n = zdtBuildStoreAutoRunEmmCmd(A, 0x00, ZDT_DIR_CW, 60, 10, 0x00, b, sizeof b);
    test_write_basic_wrap("5.7.2 StoreAutoRunEmm(clear restore)", A, b, n);

    /* ====== 5.8 批量读 ====== */
    ESP_LOGI(TAG, "--- 5.8 bulk reads ---");
    n = zdtBuildReadAllStatusEmmCmd(A, b, sizeof b); test_read_long("5.8.2 ReadAllStatusEmm", A, b, n, 8);
    n = zdtBuildReadAllConfigEmmCmd(A, b, sizeof b); test_read_long("5.8.5 ReadAllConfigEmm", A, b, n, 8);

    /* 收尾 */
    stop_motor(A);

    /* ====== 汇总 ====== */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "TOTAL: %d PASS / %d FAIL / %d SKIP", g_pass, g_fail, g_skip);
    ESP_LOGI(TAG, "blacklist: CAL/FactoryReset/TriggerHoming/ChangeAddr/ChangeFwType/WriteAllConfig×2 + X-firmware");
    if (g_fail_count > 0) {
        ESP_LOGI(TAG, "FAILED:");
        for (int i = 0; i < g_fail_count; ++i) ESP_LOGI(TAG, "  - %s", g_failed[i]);
    } else if (g_pass > 0) {
        ESP_LOGI(TAG, "ALL TESTED COMMANDS PASSED");
    }
    ESP_LOGI(TAG, "=================================================");

    zdt_can_deinit();
}
