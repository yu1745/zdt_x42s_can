# zdt_x42s_can

> ESP-IDF 组件，把 [libzdt](https://github.com/yu1745/libzdt) 构建的 ZDT_X42S 第二代闭环步进电机命令字节流，按 ZDT 手册 4.2 章通过 **CAN（TWAI）** 发给电机。

`libzdt` 只负责构建主机命令的字节帧（含地址字节、含 `0x6B` 校验码），但步进电机驱动器在 CAN 链路上要求把这条字节流按特定规则拆分到扩展帧里。本组件封装这一层：调用方仍用 libzdt 那 82 个 `zdtBuildXxxCmd`，再用 `zdt_can_send()` 发出去即可。

## CAN 分帧规则（手册 4.2.1，真机验证）

按 ZDT 手册 4.2.1 原文实现：地址字节**只编码进 CAN ID**，不进 payload；payload 从功能码开始，每包 8 字节。详见 [`docs/CAN_FRAMING.md`](./docs/CAN_FRAMING.md)（含真机对照实验证据）。

## 协议要点

- **扩展帧（29 位 ID）**，标准帧不支持。
- **CAN ID = `(motor_addr << 8) | packet`**，`packet` 从 0 起按 8 字节 payload 递增。
- **地址字节不进 payload**，payload 从 FuncCode 开始；长命令（payload > 8B）按 packet 拆分，末包 DLC 可能 < 8。
- **默认 500 kbit/s**（手册 4.2 / 5.6.x，参数索引 7），可配置。
- 电机返回也是 `(Addr<<8)|Packet` 扩展帧，状态码 `02/E2/EE/9F` 在 payload 里（手册 4.2.2）。

## 目录结构

```
zdt_x42s_can/                   # 组件即仓库根，可被 idf_component.yml 直接导入
├── CMakeLists.txt              # idf_component_register
├── idf_component.yml           # 声明 git 依赖 libzdt
├── include/zdt_can.h
├── zdt_can.c                   # TWAI 安装 + 分帧发送 + 接收
├── examples/
│   ├── command_test/           # esp32c3 真机遍历命令测试
│   └── esp32c3_motor_test/     # esp32c3 真机测试工程
│       ├── sdkconfig.defaults  # CONFIG_IDF_TARGET="esp32c3"
│       └── main/
│           ├── main.c          # 使能→读版本→梯形运动→读位置→停止
│           └── Kconfig.projbuild  # CAN 引脚/波特率/电机地址
└── docs/CAN_FRAMING.md         # CAN 分帧规则 + 真机验证证据
```

## API

```c
#include "zdt_can.h"

typedef struct {
    int      tx_gpio;     // CAN TX 引脚（默认 GPIO0）
    int      rx_gpio;     // CAN RX 引脚（默认 GPIO2）
    uint32_t baudrate;    // bit/s（默认 500000）
} zdt_can_config_t;

esp_err_t zdt_can_init(const zdt_can_config_t *cfg);
esp_err_t zdt_can_deinit(void);

/* 把 libzdt 命令字节流按勘误规则拆分发送到 TWAI。
 * frame = zdtBuildXxxCmd 的输出（含地址、含 0x6B）
 * addr  = 电机地址（通常 = frame[0]，0x00 广播） */
esp_err_t zdt_can_send(uint8_t addr, const uint8_t *frame, int frame_len,
                       TickType_t timeout);

/* 阻塞接收一帧电机返回（手册 4.2.2）。 */
esp_err_t zdt_can_receive(uint8_t *out_addr, uint8_t *out_buf, int buf_size,
                          int *out_len, TickType_t timeout);
```

## 用法示例

```c
#include "libzdt.h"
#include "zdt_can.h"

void app_main(void) {
    zdt_can_config_t cfg = { .tx_gpio = 0, .rx_gpio = 2, .baudrate = 500000 };
    zdt_can_init(&cfg);

    uint8_t buf[16];
    int n = zdtBuildImmediateStopCmd(0x01, ZDT_SYNC_NOW, buf, sizeof buf);
    zdt_can_send(0x01, buf, n, pdMS_TO_TICKS(100));

    zdt_can_deinit();
}
```

## 编译测试工程（esp32c3 真机）

环境激活（用 fast_export 跳过冗长检测）：

```powershell
. C:\Users\wangyu\esp\v5.5\esp-idf\fast_export.ps1
```

构建、烧录、看串口：

```powershell
cd examples\esp32c3_motor_test
idf.py set-target esp32c3
idf.py build flash monitor
```

menuconfig 里可改：`ZDT_CAN_TX_GPIO` / `ZDT_CAN_RX_GPIO` / `ZDT_CAN_BAUDRATE` / `ZDT_MOTOR_ADDR`。

**本板实测正确接线**：`ZDT_CAN_TX_GPIO=7`，`ZDT_CAN_RX_GPIO=6`（GPIO6/7 接 CAN 收发器，
方向反了会因收不到 ACK 立刻进 Bus-Off）。

## 硬件接线

```
esp32c3 GPIO[TX]  ── TXD ──┐
esp32c3 GPIO[RX]  ── RXD ──┤
esp32c3 3V3 / GND  ─────────┤   CAN 收发器
                           ├── CAN_H ──┬── 120Ω ──┬── 电机 R/A/H 引脚
                           └── CAN_L ──┴── 120Ω ──┴── 电机 T/B/L 引脚
                                                  共地
```

- CAN 收发器：SN65HVD230（3.3V）/ TJA1050（5V，需电平转换）等。
- 总线两端各加 120Ω 终端电阻。
- 电机端需把"通讯端口复用模式"设为 **03（CAN）**（手册 5.6.x / 上位机参数）。
- esp32c3 引脚经 GPIO matrix 任意选，避开 strapping/Flash 占用脚。

## 依赖

- ESP-IDF ≥ 5.0（开发用 v5.5 验证）。TWAI 驱动在 `driver/twai.h`。
- [libzdt](https://github.com/yu1745/libzdt)：通过 `idf_component.yml` 的 git 依赖自动拉取。

## 许可证

MIT。
