/*
 * split_host_test.c - host-mode 自测：验证 zdt_can_split 的分帧规则。
 *
 * 分帧规则（手册 4.2.1 原文，已真机验证）：
 *   - CAN 扩展帧 ID = (addr << 8) | packet,  packet = 0, 1, 2, ...
 *   - 地址字节**不进 payload**（只编码进 CAN ID）
 *   - payload 从 FuncCode 开始，每包 8 字节，按 packet 拆分
 *   - 末包 DLC 可能 < 8
 *
 * 编译运行（不依赖 ESP-IDF）：
 *   gcc -std=c99 -Wall -Wextra -DZDT_CAN_HOST_TEST \
 *       components/zdt_can/test/split_host_test.c -o /tmp/split_host_test && \
 *   /tmp/split_host_test
 *
 * 期望输出末尾：3/3 PASS
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define ZDT_CAN_MAX_PACKETS 8

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} zdt_can_packet_t;

/* === 与 zdt_can.c::zdt_can_split 等价的纯逻辑副本 === */
static int split(uint8_t addr, const uint8_t *frame, int frame_len,
                 zdt_can_packet_t *out, int *count)
{
    if (!frame || !out || !count) return -1;
    if (frame_len < 3) return -1;
    if (frame[0] != addr) return -1;

    /* 地址字节 frame[0] 不进 payload；从 frame[1] 开始每包 8 字节 */
    int idx = 1, remaining = frame_len - 1, pkt = 0;
    while (remaining > 0 && pkt < ZDT_CAN_MAX_PACKETS) {
        zdt_can_packet_t *p = &out[pkt];
        p->id = ((uint32_t)addr << 8) | (uint32_t)pkt;
        int take = remaining > 8 ? 8 : remaining;
        memcpy(&p->data[0], &frame[idx], (size_t)take);
        p->dlc = (uint8_t)take;
        idx += take; remaining -= take; pkt++;
    }
    if (remaining > 0) return -2;
    *count = pkt;
    return 0;
}

static int g_pass = 0, g_fail = 0;

static void dump_pkt(const char *tag, const zdt_can_packet_t *p)
{
    printf("    %-6s id=0x%05lX dlc=%d data:",
           tag, (unsigned long)p->id, p->dlc);
    for (int i = 0; i < p->dlc; ++i) printf(" %02X", p->data[i]);
    printf("\n");
}

static int expect(const char *name, const zdt_can_packet_t *got, int got_n,
                  const zdt_can_packet_t *exp, int exp_n)
{
    int ok = (got_n == exp_n);
    for (int i = 0; ok && i < exp_n; ++i) {
        if (got[i].id != exp[i].id) ok = 0;
        if (got[i].dlc != exp[i].dlc) ok = 0;
        if (memcmp(got[i].data, exp[i].data, exp[i].dlc) != 0) ok = 0;
    }
    if (ok) { g_pass++; printf("  PASS  %s  (%d pkts)\n", name, got_n); }
    else {
        g_fail++;
        printf("  FAIL  %s\n", name);
        printf("    got %d pkts:\n", got_n);
        for (int i = 0; i < got_n; ++i) dump_pkt("got", &got[i]);
        printf("    exp %d pkts:\n", exp_n);
        for (int i = 0; i < exp_n; ++i) dump_pkt("exp", &exp[i]);
    }
    return ok;
}

int main(void)
{
    zdt_can_packet_t got[ZDT_CAN_MAX_PACKETS];
    int n;

    /* ----------------------------------------------------------------
     * Case 1: 3 字节短命令  01 36 6B  → 1 包
     * 地址 01 不进 payload，payload = 36 6B
     * ---------------------------------------------------------------- */
    {
        const uint8_t f[] = {0x01, 0x36, 0x6B};
        split(0x01, f, (int)sizeof f, got, &n);
        zdt_can_packet_t exp[] = {
            {0x0100, 2, {0x36, 0x6B, 0,0,0,0,0,0}},
        };
        expect("case1 short 3B", got, n, exp, 1);
    }

    /* ----------------------------------------------------------------
     * Case 2: 13 字节命令（手册 4.2.1 原例）
     *   01 FD 01 0F A0 00 00 01 FA 00 00 00 6B
     * 地址 01 不进 payload，剩 12 字节 payload → ceil(12/8) = 2 包
     *   pkt0: id=0x0100 dlc=8 data=FD 01 0F A0 00 00 01 FA
     *   pkt1: id=0x0101 dlc=4 data=00 00 00 6B
     * ---------------------------------------------------------------- */
    {
        const uint8_t f[] = {0x01, 0xFD, 0x01, 0x0F, 0xA0, 0x00, 0x00, 0x01,
                             0xFA, 0x00, 0x00, 0x00, 0x6B};
        split(0x01, f, (int)sizeof f, got, &n);
        zdt_can_packet_t exp[] = {
            {0x0100, 8, {0xFD, 0x01, 0x0F, 0xA0, 0x00, 0x00, 0x01, 0xFA}},
            {0x0101, 4, {0x00, 0x00, 0x00, 0x6B, 0, 0, 0, 0}},
        };
        expect("case2 13B manual ex", got, n, exp, 2);
    }

    /* ----------------------------------------------------------------
     * Case 3: 5.3.1 多电机广播命令（libzdt 测试用例同款，34 字节）
     *   00 AA 00 22 02 FD ... 6B
     * 地址 00 不进 payload，剩 33 字节 → ceil(33/8) = 5 包
     * 每包 id 0x0000..0x0004
     * ---------------------------------------------------------------- */
    {
        const uint8_t f[] = {
            0x00, 0xAA, 0x00, 0x22, 0x02, 0xFD, 0x01, 0x05, 0xDC, 0x08,
            0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x6B, 0x03, 0xFD, 0x00,
            0x03, 0xE8, 0x0A, 0x00, 0x00, 0xFA, 0x00, 0x01, 0x01, 0x6B,
            0x04, 0x36, 0x6B, 0x6B
        };
        split(0x00, f, (int)sizeof f, got, &n);
        /* 34B → addr 占 1，剩 33 字节 payload，每包 8 字节 → 5 包 (8*4+1) */
        zdt_can_packet_t exp[5] = {0};
        int idx = 1;
        for (int i = 0; i < 4; ++i) {
            exp[i].id  = (uint32_t)i;
            exp[i].dlc = 8;
            memcpy(&exp[i].data[0], &f[idx], 8);
            idx += 8;
        }
        /* pkt4: 剩余 1 字节 (33-32=1) → dlc=1 */
        exp[4].id  = 0x0004;
        exp[4].dlc = 1;
        exp[4].data[0] = f[idx];
        expect("case3 broadcast 34B", got, n, exp, 5);
    }

    printf("\n=========================================\n");
    printf("TOTAL: %d PASS / %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
