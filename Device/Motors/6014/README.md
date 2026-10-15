# IM6014 / N6014B-12.6 通信驱动

本目录实现宇树 IM6014（N6014B-12.6）的 RS485 半双工通信。协议依据：[宇树官方 IM6014 电机通讯协议](https://support.unitree.com/home/zh/Motor_IM6014%20Development%20Guide/Motor_Protocol)。相关资料：[电机配置](https://support.unitree.com/home/zh/Motor_IM6014%20Development%20Guide/Motor_configuration)、[官方 FAQ](https://support.unitree.com/home/zh/Motor_IM6014%20Development%20Guide/FAQ)。本工程使用 STM32H723、USART1、硬件自动 DE、DMA 非阻塞发送及循环 DMA 接收。

**电机 ID 仅允许 1–12。** `motor6014.cmd[id]` 是最近一次成功设置的内部命令，`motor6014.data[id]` 是最近一次有效反馈。数组下标 0 保留，不使用；不同 USART 上的物理电机 ID 也必须全局唯一。这是电机配置约束，驱动不替应用设置或永久绑定 ID；设置和发送接口始终显式选择串口。

## 1. 接线和 CubeMX 配置

| 项目 | 本工程设置 |
| --- | --- |
| USART | USART1，6000000 baud，8 数据位、无校验、1 停止位（8N1） |
| TX / RX | PB14 / PB15，连接 RS485 收发器的 DI / RO |
| 硬件 DE | PA12，连接收发器 DE，DE 高电平发送 |
| DE 产生方式 | `HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0)` |
| RX DMA | DMA1_Stream0，循环模式，字节对齐，内存地址自增 |
| TX DMA | DMA1_Stream1，普通模式，字节对齐，内存地址自增 |
| 中断 | 开启 USART1、RX DMA、TX DMA 中断 |
| GPIO 速度 | PB14、PB15、PA12 均为 `GPIO_SPEED_FREQ_VERY_HIGH` |

RS485 收发器必须支持 6 Mbps，单片机与收发器需共地，A/B 接电机对应总线。根据收发器手册处理 `/RE`：若与 DE 相连，发送时接收器关闭；若始终使能接收器，可能看到本机命令回显。驱动按反馈帧头筛选，但接线仍应保证电机反馈期间接收器开启。总线终端、电平及供电按电机和收发器要求配置。

这里的“485 流控”是 USART 硬件自动控制 DE，**不是 RTS/CTS**。因此 `huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE` 是正确设置，不要另写 GPIO 切换 DE。

`.ioc` 与 `usart.c` 的 RX 循环 DMA、GPIO 速度已同步；`.cproject` 的 Debug 和 Release 均已加入 `Device` 源码目录及 `../Device/Motors/6014` 头文件路径。重新生成代码后应检查这些配置仍然存在。

## 2. 初始化

工程已经在 `main.c` 的 USER CODE 区域接入以下内容，初始化不会自动发命令，也不会使能电机：

```c
#include "motor6014.h"

static Motor6014_Port_t motor6014_port1;

/* HAL、DMA、USART 初始化完成后调用一次。 */
if (Motor6014_PortInit(&motor6014_port1, &huart1,
                      MOTOR6014_DEFAULT_TIMEOUT_MS) != MOTOR6014_OK)
{
    Error_Handler();
}

/* 主循环不需要调用额外的周期服务函数。 */
while (1)
{
    /* 根据应用需要调用 SetCommand() 和 SendCommand()。 */
}
```

`Motor6014_PortInit(&motor6014_port1, &huart1, 2)` 的三个参数分别是：

1. `&motor6014_port1`：生命周期覆盖整个使用过程的端口上下文，保存该串口的私有收发缓冲和事务状态；不要使用初始化函数的临时局部变量。
2. `&huart1`：要使用的 HAL 串口句柄。
3. `2`：发送及等待反馈的事务超时，单位 ms；默认宏为 `MOTOR6014_DEFAULT_TIMEOUT_MS`。发送函数在检查到事务超过这个时间时自动恢复端口，不需要额外的 Service 调用。

端口初始化绑定串口并启动循环 DMA 接收。2 ms 是主机驱动的事务期限，**与电机内置的 1 秒通信保护不同**。后者由发送参数 `timeout_enable` 决定，默认关闭。

同一端口和串口只初始化一次，超时参数须处于 1–`INT32_MAX` ms；重复初始化返回参数错误。注册后该串口的 HAL DMA 收发由本驱动使用，应用不要同时调用其他 HAL 收发函数争用该串口。

`Motor6014_PortInit()` 和 `Motor6014_SetCommand()` 从主上下文调用。`Motor6014_SendCommand()` 可从主上下文或 UART 空闲的 ISR 调用；如果端口正在超时恢复，ISR 调用只返回 `MOTOR6014_BUSY`，不会执行阻塞恢复。不自动轮询、不自动发送、不自动重试；应用在需要发送时显式调用发送函数。反馈仍由 HAL 回调在中断上下文解析。

## 3. 设置并发送命令

驱动参考 8010 电机的调用方式，将命令参数保存在内部 `motor6014.cmd[id]`，再按 ID 发送。设置参数不会启动 DMA，只有 `Motor6014_SendCommand()` 才会实际发送。

```c
Motor6014_Status_t Motor6014_SetCommand(
    UART_HandleTypeDef *huart,
    uint8_t id,
    Motor6014_Mode_t mode,
    float position,
    float velocity,
    float torque,
    float kp,
    float kd,
    bool timeout_enable
);

Motor6014_Status_t Motor6014_SendCommand(
    UART_HandleTypeDef *huart,
    uint8_t id
);
```

| 参数 | 含义 |
| --- | --- |
| `huart` | 已通过 `Motor6014_PortInit()` 注册的串口 |
| `id` | 电机 ID，仅 1–12 |
| `mode` | 停机 `MOTOR6014_MODE_STOP`，或 FOC `MOTOR6014_MODE_FOC` |
| `position` | 转子轴目标位置，rad |
| `velocity` | 转子轴目标角速度，rad/s |
| `torque` | 转子轴前馈力矩，N·m |
| `kp` / `kd` | 转子轴位置增益 N·m/rad / 速度增益 N·m/(rad/s) |
| `timeout_enable` | 是否开启电机内置 1 秒通信保护，关闭传 `false` |

设置并发送停机命令：

```c
Motor6014_Status_t status = Motor6014_SetCommand(
    &huart1, 1, MOTOR6014_MODE_STOP,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false
);
if (status == MOTOR6014_OK) {
    status = Motor6014_SendCommand(&huart1, 1);
}
```

FOC 控制调用形式如下。目标位置、速度、力矩和增益由应用依据机械条件给定，示例不在工程中自动执行：

```c
status = Motor6014_SetCommand(
    &huart1, 1, MOTOR6014_MODE_FOC,
    target_position, target_velocity, target_torque,
    kp, kd, false
);
if (status == MOTOR6014_OK) {
    status = Motor6014_SendCommand(&huart1, 1);
}
```

`Motor6014_SetCommand()` 检查 ID、模式、浮点参数和协议范围；成功后将未量化的参数保存到 `motor6014.cmd[id]`。`Motor6014_SendCommand()` 从该内部命令记录编码并启动 DMA。发送函数返回 `MOTOR6014_OK` 仅表示 DMA 已启动，不代表电机已经收到命令或执行完成。发送时若端口忙，函数按初始化时的 `timeout_ms` 检查并恢复超时事务；仍在期限内则返回 `MOTOR6014_BUSY`。发送启动失败不会改写已保存的命令。浮点量必须有限，增益不能为负；超出协议编码范围的数值返回错误，不做静默限幅。

```c
/* 调试器可直接展开这些成员。修改 cmd 不会自动触发发送；
   推荐始终使用 SetCommand() 更新内部参数。 */
float last_target_position = motor6014.cmd[1].position;
float last_target_velocity = motor6014.cmd[1].velocity;
float last_target_torque   = motor6014.cmd[1].torque;
```

每个串口同一时刻只允许一笔事务，发送期间使用端口的私有缓冲；之后再次调用 `SetCommand()` 不会改变已经提交的帧。应用应根据返回状态安排后续发送，避免无条件连续提交。

端口 `busy` 表示事务是否进行中，`last_status` 在发送中为 `MOTOR6014_BUSY`，完成后为 `MOTOR6014_OK`，超时或串口故障时为 `MOTOR6014_TIMEOUT` / `MOTOR6014_UART_ERROR`。`stats` 成员提供 `tx_count`、`rx_count`、`crc_error_count`、`invalid_frame_count`、`unexpected_reply_count`、`timeout_count` 和 `uart_error_count`，例如 `motor6014_port1.stats.timeout_count`，便于在调试器中排查通信问题。`last_uart_error` 保留最近的 HAL 串口错误码。未收到反馈时保留最近一次历史有效反馈，结合更新时间使用。反馈解析通过 HAL 回调独立完成，不需要应用轮询 Service。

## 4. 反馈读取和多串口

`motor6014.data[id]` 由中断更新，可直接在调试器中查看。应用需要多个字段属于同一帧时，使用快照接口：

```c
Motor6014_Data_t feedback;
if (Motor6014_GetDataSnapshot(1, &feedback) == MOTOR6014_OK &&
    feedback.valid)
{
    float rotor_position = feedback.position; /* rad */
    float rotor_velocity = feedback.velocity; /* rad/s */
    float rotor_torque   = feedback.torque;   /* N.m */
    uint32_t age_ms = HAL_GetTick() - feedback.last_update_ms;
    /* 使用 age_ms 判断这份历史有效反馈是否仍满足应用实时性要求。 */
}
```

反馈还包含 `mode`、`timeout_triggered`、`driver_temperature`（°C，带符号）、`winding_temperature`（°C，无符号）、`voltage`（V）、`error`、`warning`、`last_update_ms` 和 `sequence`。`valid` 表示曾收到有效帧，不表示电机当前在线；应用应结合更新时间判断时效。错误和警告按官方位域保留，应结合官方协议解释。

新增串口时先在 CubeMX 配置独立 TX/RX DMA 和硬件 DE，随后为其创建独立、长期有效的上下文：

```c
static Motor6014_Port_t motor6014_port2;
/* MX_USART2_UART_Init() 完成后，huart2 已具有独立的 DMA。 */
if (Motor6014_PortInit(&motor6014_port2, &huart2,
                      MOTOR6014_DEFAULT_TIMEOUT_MS) != MOTOR6014_OK)
{
    Error_Handler();
}

/* 例如 USART1 使用 ID 1，USART2 使用 ID 2。 */
Motor6014_SetCommand(&huart2, 2, MOTOR6014_MODE_STOP,
                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
Motor6014_SendCommand(&huart2, 2);
```

各端口的 DMA 缓冲、事务和超时互相独立，可以并行通信；所有串口仍共用 `motor6014`。不同物理电机不能在不同总线上重复使用同一 ID。同一 ID 的并发事务返回忙；驱动不永久绑定 ID 与串口，路由变更由应用负责。

以下 HAL 回调已在 `main.c` 中定义，统一按传入句柄分发，可覆盖所有已注册端口。**每个工程仅定义一次**；若应用已有这些回调，应将转发合并进去，不要重复定义或只限定 USART1：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Motor6014_OnRxEvent(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Motor6014_OnTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Motor6014_OnError(huart);
}
```

接收支持拆帧、连续帧、DMA 回绕及损坏 CRC 后重新同步。只有来自对应串口、当前请求 ID 的有效反馈才能完成当前事务。协议没有事务序号：超时后同一 ID 的迟到旧反馈与新请求反馈不能完全区分；应用应结合机械通信延迟安排超时后的再次发送。

RX DMA 的半传输 HT、全传输 TC 中断必须保留开启，不要调用 `__HAL_DMA_DISABLE_IT(..., DMA_IT_HT)`。接收使用实际 DMA 游标处理空闲及 DMA 中断的先后顺序；中断若被延迟到整个 128 字节接收环被覆盖，已丢失的字节无法恢复。因此避免长时间关闭中断。

## 5. 协议帧和数值换算

命令为 20 字节，反馈为 26 字节。多字节整数、CRC 字段均按小端传输。驱动逐字节编解码，不依赖 C 结构体打包或对齐。

### 命令帧：20 字节

| 字节偏移 | 长度 | 类型 | 字段 / 编码 |
| --- | --- | --- | --- |
| 0 | 2 | 字节 | 帧头 `FE EE` |
| 2 | 1 | 位域 | bit 0–3：ID；bit 4–6：模式；bit 7：通信保护开关 |
| 3 | 1 | 保留 | 0 |
| 4 | 2 | `int16_t` | 力矩 `torque × 2560` |
| 6 | 2 | `int16_t` | 角速度 `velocity × 64 / (2π)` |
| 8 | 4 | `int32_t` | 位置 `position × 32768 / (2π)` |
| 12 | 2 | `int16_t` | Kp `kp × 12800`，负值保留，不发送 |
| 14 | 2 | `int16_t` | Kd `kd × 51200`，负值保留，不发送 |
| 16 | 4 | `uint32_t` | CRC32，覆盖偏移 0–15，包含帧头 |

### 反馈帧：26 字节

| 字节偏移 | 长度 | 类型 | 字段 / 解码 |
| --- | --- | --- | --- |
| 0 | 2 | 字节 | 帧头 `FC EE` |
| 2 | 1 | 位域 | bit 0–3：ID；bit 4–6：模式；bit 7：通信保护触发状态 |
| 3 | 1 | `int8_t` | 驱动温度，°C |
| 4 | 1 | `uint8_t` | 绕组温度，°C |
| 5 | 1 | `uint8_t` | 母线电压，原始值 `/ 2` 得到 V |
| 6 | 2 | `int16_t` | 力矩，原始值 `/ 2560`，N·m |
| 8 | 2 | `int16_t` | 角速度，原始值 `× 2π / 64`，rad/s |
| 10 | 4 | `int32_t` | 位置，原始值 `× 2π / 32768`，rad |
| 14 | 4 | `uint32_t` | 电机错误位 |
| 18 | 2 | 位域 | 高 3 位为警告，`(word >> 13) & 7` |
| 20 | 2 | 保留 | CRC 校验时仍须包含 |
| 22 | 4 | `uint32_t` | CRC32，覆盖偏移 2–21，不包含帧头 |

编码量先检查连续值是否处于整数类型可表示的范围，再四舍五入，恰好半步时向远离零的方向取整。Kp / Kd 按官方协议采用带符号整数，原始值只允许 0–32767，负值是保留编码；相应 Kp 上限为 `32767 / 12800 = 2.559921875`，Kd 上限为 `32767 / 51200 = 0.63998046875`，浮点入参还须满足实际量化前的范围检查。CRC 不是常见的反射式以太网 CRC32：初值 `0xFFFFFFFF`，多项式 `0x04C11DB7`，无最终异或。每 4 字节先按小端组合成 32 位字，再从该字的最高位到最低位逐位计算；CRC 结果按小端写入帧。TX 校验 4 个字（16 字节），RX 校验 5 个字（20 字节）。

### 转子轴与输出轴

发送和反馈的 `position`、`velocity`、`torque` 均为**转子轴**量，不是减速器输出轴量。减速比采用精确值 `38/3`，而非将型号上的 12.6 当作精确值：

```text
输出轴位置 = 转子轴位置 / (38/3)
输出轴速度 = 转子轴速度 / (38/3)
输出轴力矩 = 转子轴力矩 × (38/3)
```

```c
Motor6014_MechanicalState_t rotor = {
    .position = feedback.position,
    .velocity = feedback.velocity,
    .torque = feedback.torque
};
Motor6014_MechanicalState_t output = Motor6014_RotorToOutput(rotor);
Motor6014_MechanicalState_t rotor_again = Motor6014_OutputToRotor(output);
```

力矩换算是忽略传动损耗的理想关系，不代表实测输出力矩。位置转换仅转换单位，不设置机械零位。

## 6. 配置和排查

- **新电机没有反馈：**出厂 ID 为 0；先用官方上位机将电机改为 1–12，驱动拒绝 ID 0。确认电机型号和固件使用上述 IM6014 协议，而不是其他宇树型号的帧格式。
- **初始化失败：**检查 USART 已初始化、RX DMA 为循环模式、TX DMA 为普通模式，并且句柄的 `hdmarx` / `hdmatx` 已正确链接。
- **一直忙或超时：**确认 USART 和 DMA 中断及三个 HAL 回调均转发，检查 DE / `/RE`、A/B、共地、收发器速率及波特率。发送函数会按端口 `timeout_ms` 检查并恢复超时事务；增大主机超时时间应根据实测延迟决定。持续硬件故障会保持忙状态，检查 `last_status`、`last_uart_error` 和 `stats`。
- **命令被拒绝：**检查 ID、模式、浮点有限性和协议可编码范围。编码范围不是电机机械安全范围，实际目标和增益应依据机械系统设定。
- **数据时好时坏：**检查接线和 6 Mbps 信号完整性、终端及主循环阻塞。本工程当前未启用 DCache，DMA 缓冲位于 DMA 可访问的 RAM_D1；未来启用 DCache 或改变链接布局时，必须额外处理缓存一致性并确保缓冲不位于 DMA1 无法访问的 DTCM。
- **电机通信保护：**`timeout_enable = false` 仅关闭电机内置的 1 秒通信保护，主机的事务超时检测仍正常工作。需要该保护时明确传 `true`。

构建和主机测试只能验证软件协议及状态处理；电机实机反馈、DE 时序和运动效果仍需连接硬件验收。

## 7. 软件验证

当前工作区已完成协议/传输源码的严格语法检查，以及 CubeIDE 1.19.0 的 Debug、Release 编译和链接；两种配置均为 0 错误、0 警告。严格检查需将 CMSIS 目录按系统头文件加入，以避免第三方 CMSIS 头文件自身的 `-Wsign-conversion` 告警干扰结果：

```powershell
arm-none-eabi-gcc -std=c11 -mcpu=cortex-m7 -mthumb `
  -mfpu=fpv5-d16 -mfloat-abi=hard `
  -DSTM32H723xx -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY `
  -ICore/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc `
  -isystem Drivers/CMSIS/Device/ST/STM32H7xx/Include `
  -isystem Drivers/CMSIS/Include -IDevice/Motors/6014 `
  -Wall -Wextra -Wconversion -Werror -fsyntax-only `
  Device/Motors/6014/motor6014.c `
  Device/Motors/6014/motor6014_protocol.c
```

软件构建不能替代 RS485 接线、DE 时序、实际反馈和电机运动验收；这些仍需连接硬件验证。
