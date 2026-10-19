# 3508 / BMI088 修复与验证记录

日期：2026-10-19。本记录替代上一轮待修复清单，描述当前实现。范围为两套驱动、姿态状态及 FreeRTOS 接入；姿态输出采用四元数名义状态和三轴误差状态 EKF，保留原有姿态轴向与欧拉角接口。

## 配置与修复结果

| 项目 | 当前行为 |
| --- | --- |
| CAN / 电调映射 | FDCAN1：ID 1、2；FDCAN2：ID 3、4；发送 0x200 的对应槽位，其余槽位为零 |
| 消息 RAM | 已核对代码及 `.ioc`：字偏移 0 / 20 / 40；当前半开区间 [0,17)、[20,37)、[40,57)，互不重叠 |
| 接收校验 | 实例与 ID 配对、标准经典数据帧、DLC=8、编码器 0…8191；拒收计数可查询 |
| 启动与失效 | D3508_Init 为 ID 1…4 登记一次上电使能请求；各电机收到首帧有效反馈后自动使能，提交新目标前输出仍为零。20 ms 无反馈即清零输出、积分、历史误差和目标 |
| 上电使能取消 | 主动禁用、反馈超时或 bus-off 取消对应电机的上电使能请求；禁用或故障恢复后不自动使能。收到首帧前主动禁用同样有效 |
| 恢复反馈 | 首帧重建绝对编码器基准，圈数归零，保持禁用；重新使能后仍需提交新目标。旧多圈位置不可继续用作目标 |
| 故障恢复 | bus-off 锁定整条总线，另一条总线继续运行；显式请求恢复，由 CAN 任务 stop/init/start；失败保持锁定，成功等待新反馈且不自动使能 |
| 发送 | 查询并返回两条总线的 HAL 提交结果；FIFO 满本周期返回 BUSY，无阻塞重试；清零/故障时请求撤销旧 TX |
| PID | 先求速度微分后更新历史；位置环不覆盖速度历史；位置积分使用自身 Max_Sum，零禁用；默认 dt=5 ms，支持显式 dt |
| 数值处理 | 圈数乘法先提升 int64；MIT 先求整数位置差；处理 INT64 两端相减；索引、非有限参数与异常 dt 均检查 |
| 力矩 | 转子 Nm → 电流前馈 → C620 内部电流环；去掉不可达 MCU 力矩 PID；电流限制 ±16000 |
| 功率 | 保留每台电机 162 W 的机械功率估算限制；受转矩常数、速度滤波等影响，不能保证整机或直流输入功率小于 162 W |
| SPI | 每个寄存器事务一次 HAL 调用；加速度计保留一个 dummy；每事务 1 ms 超时，任何返回路径释放 CS |
| 完整样本 | 加速度、陀螺、温度全部成功才发布；包含时间戳、序号、通信状态；失败保留旧样本和有效时间，错误通过返回值及诊断查询 |
| 初始化 | BMI088 在 main 中、GPIO/SPI 初始化之后、MX_FREERTOS_Init 之前初始化一次；两颗传感器错误分别记录，任务只接管 SPI 所有权，不重新初始化 |
| 延时 | 微秒等待使用 DWT，支持回绕并检测停止的计数器；调度器运行时毫秒等待用 osDelay，启动前使用 TIM4 的 HAL_Delay |
| 校准 | 手动请求，采样任务执行；400 个连续有效静止近水平样本，2.5 s 超时；运动/读取失败重置连续计数，重复序号不计数 |
| 姿态就绪 | 必须已校准、通信成功、样本年龄 <50 ms、数据有限且原始/校正加速度 Z>0；失败不刷新有效时间；重复序号不积分 |
| 姿态算法 | 陀螺仪传播四元数，归一化加速度作为重力观测；3×3 小角度误差协方差进行 EKF 更新，动态加速度超出 0.85…1.15g 时跳过观测校正 |
| 姿态输出 | `Quaternion_W/X/Y/Z` 顺序为 w、x、y、z；`Angle_Pitch/Roll/Yaw` 从四元数转换，Yaw 为无磁力计的相对航向；快照同时提供 `EKF_Valid`、协方差对角线和创新量 |
| 任务 | BMI088 200 Hz 调用 BMI088Read，统一采样、校准处理和姿态更新；AttitudeControl 保留给应用控制，不重复解算；3508 200 Hz 执行 Set/Send；栈分别 1024 / 2048 / 1024 字节，检查任务创建结果 |
| 数据共享 | 短临界区复制快照和变更状态；SPI、CAN 调用及浮点计算在临界区外；控制计算通过 epoch 防止跨越禁用/故障提交旧结果 |
| GPIO | 使用轮询，关闭传感器 DRDY 输出/映射；PE10、PE12 改为普通输入并同步 `.ioc` |

减速比保持 15.7。新的 `Motor3508_SetCommand` 专用于 MIT：位置单位为转子编码器 tick，速度为转子 RPM，前馈力矩为轮端 Nm，kp/kd 分别为轮端 Nm/rad、Nm/(rad/s)。调用形式采用 6014 的 Set/Send，但沿用原 3508 的单位，不直接套用 6014 的转子 rad/rad/s/Nm 数值。旧 `PID_Calc_Torque()` 仍接收转子 Nm。`PID_Calc_Position()` 的旧 float 参数仍有精度限制。

PID 的 Max_Sum 保留原来的“5 ms 周期误差累加值”单位；带 dt 的接口按 dt / 0.005 加权，积分输出仍乘 0.005。这样默认周期的原有 Ki、积分限幅含义保持一致。位置环的速度阻尼及默认参数未重新整定。

## 接口与调用方法

`Motor3508_SetCommand(handle, id, position, velocity, torque, kp, kd)` 中的 id 为物理电调 ID 1…4：FDCAN1 对应 1/2，FDCAN2 对应 3/4。Set 内立即完成 MIT 力矩、电流计算和限幅，仅缓存结果；`Motor3508_SendCommand(handle)` 发送该总线所有已缓存电流，不重新计算 PID/MIT。C620 的 0x200 是组帧，因此同一总线先 Set 两台、再 Send 一次。

旧使能/反馈接口的电机索引仍为 0…3，总线诊断编号为 1、2。不要跨任务直接读写 `Motors`、`motor3508` 或 `g_Attitude`；使用快照接口。Set/Send 由同一应用/CAN 任务拥有；本工程在 `D3508_ApplicationControl(dt)` 中提交 Set，随后任务每条总线各 Send 一次。默认钩子不设置目标，输出保持为零。

`D3508_Init()` 在 CAN 接收通知启动前执行，因此初始化时只登记 ID 1…4 的上电使能请求；首帧有效反馈建立位置基准后完成使能。应用上电无需再调用四次 Enable；反馈未就绪时 Set 返回 NOT_READY 且不缓存目标，需要在后续控制周期重新提交。每次上电仅自动使能一次，超时、主动禁用和 bus-off 后仍需显式恢复并重新使能。该初始化函数只在启动阶段调用，不用于故障恢复。

```c
/* main 中已执行一次 D3508_Init()，ID 1…4 收到首帧后自动使能。
 * 周期内只需 Set/Send，无需反复调用 Enable。 */

/* 周期中：position, velocity, torque, kp, kd；限幅无需作为参数传入。 */
Motor3508_SetCommand(&hfdcan1, 1, p1_ticks, v1_rpm, t1_wheel_nm, kp1, kd1);
Motor3508_SetCommand(&hfdcan1, 2, p2_ticks, v2_rpm, t2_wheel_nm, kp2, kd2);
Motor3508_SendCommand(&hfdcan1);
/* FDCAN2 同理：先 Set ID 3、4，再 Send(&hfdcan2)。
 * 在本工程钩子内只写 Set；外层任务已经负责 Send，不要再重复发送。 */

/* 与 6014 类似的状态字段：port1/2 分别表示 FDCAN1/2。 */
Motor3508_Status set_result = motor3508.Set_status_port1;
Motor3508_Status send_result = motor3508.Send_status_port1;
Motor3508_Command last_command;
Motor3508_GetCommandSnapshot(&hfdcan1, 1, &last_command);

D3508_Feedback feedback;
D3508_GetFeedback(0, &feedback);
D3508_BusStatus bus_status;
D3508_GetBusStatus(1, &bus_status);
D3508_Disable(0);

/* 确认总线故障原因消除后请求；任务处理，可查询 fault_latched。 */
D3508_RequestBusRecovery(1);
/* 等待总线恢复且重新收到有效反馈后，由应用显式重新使能。 */
uint8_t enabled = D3508_Enable(0); /* 索引 0 对应物理 ID 1；1 表示成功。 */
/* 成功后重新提交 Set 目标。 */
```

限幅集中在 `Device/Motors/3508/3508_driver.h`：`D3508_CURRENT_LIMIT_UNITS`（默认 16000，合法范围 0…16384）、`D3508_MECHANICAL_POWER_LIMIT_W`（默认 162 W/台）、`D3508_MIT_MAX_ROTOR_RPM`（默认 6000 RPM）、`D3508_MIT_MAX_WHEEL_TORQUE_NM`（默认由电流上限、转矩常数和减速比计算，约 4.78 Nm）。最后一个宏可按需要改成固定轮端力矩值，最终仍受电流和机械功率估算限制。kp/kd 要求有限且非负，P、D 与前馈合力矩统一限幅。

`motor3508.cmd[id]` 记录最近成功 Set 的命令，速度和前馈力矩记录限幅后的值；它是提交记录，不代表当前电机已使能或仍在执行该目标。超时/禁用后活动目标和电流清除，Send 不会使用这份历史记录恢复输出。Set/Send 结果分别保存在 `Set_status_portN`、`Send_status_portN`，未知总线参数的结果可查 `last_set_status` / `last_send_status`。

`D3508_GetFeedback()` 返回一致的电机状态及反馈年龄。旧的 PID/MIT、`D3508_SendCurrent()`、`send_current()` 保留兼容；应用无需在新的 MIT Set 后单独调用旧 PID。Send 的 OK 只表示进入发送 FIFO，不表示已获得 CAN ACK 或电机执行确认。bus-off、FIFO 满、线路断开时无法保证零电流帧能到达电调，现场应验证接收端超时行为及硬件急停。

`FDCAN2_Filter_Init()` 为实际 bus2 入口。保留旧 `FDCAN3_Filter_Init()` 作为已弃用的 bus2 别名。接收解码接口改为 `D3508_Decode(handle, header, data)`，必须携带总线与帧类型，不能使用旧的仅 ID 接口。

```c
/* 板卡静止且近水平时，由任意应用任务提交一次请求。 */
uint8_t accepted = BMI088_RequestCalibration();
BMI088_Calibration cal;
BMI088_GetCalibration(&cal);  /* REQUESTED -> RUNNING -> SUCCEEDED / FAILED */

BMI088_Sample sample;
BMI088_GetSample(&sample);   /* 获取上一份完整样本；应用仍需检查年龄。 */
BMI088_Diagnostics diag;
BMI088_GetDiagnostics(&diag);
Attitude_Data_t attitude;
Attitude_GetSnapshot(&attitude); /* 使用 Data_Valid、Calibrated、EKF_Valid 判断能否使用。 */
/* Quaternion_W/X/Y/Z: w, x, y, z; Angle_* 由四元数导出。 */
```

校准默认条件：角速度模长 <0.05 rad/s，加速度模长 0.95…1.05g，|X|、|Y| 分别 <0.05g，Z>0。开始重新校准后姿态即不就绪；失败需要重新请求，不自动尝试。

启动顺序：`MX_GPIO_Init()` / `MX_SPI2_Init()` → `BMI088_init()` → `Attitude_Init()` → `osKernelInitialize()` → `MX_FREERTOS_Init()` → `osKernelStart()`。初始化依赖 TIM4/DWT，不依赖尚未启动的 SysTick。之前的任务初始化位于 for 循环外，本来也只执行一次；现在按要求统一放到启动阶段。

FreeRTOS 中 BMI088 任务每 5 ms 只调用 `(void)BMI088Read();`：函数内部取得 SPI 所有权、读取完整样本、处理校准请求、更新 `g_Attitude`，不会调用初始化。返回 1 表示本次姿态已就绪，0 表示未校准/通信失败/姿态无效等。手动校准接口保持不变。其他任务使用 `Attitude_GetSnapshot()` 读取角度和就绪状态；普通读者不要调用 BMI088Read 或重复解算。

旧 `BMI088_ReadSample()`、`BMI088_read()` 及低层寄存器事务仍只允许 BMI088 所有者任务使用，其他任务读取原始缓存用 `BMI088_GetSample()`。兼容 `BMI088_read()` 成功返回 1，失败返回 0 并保持调用者输出缓冲区不变。

`Attitude_Init()` 只复位算法状态，不访问 SPI；`Attitude_Calibrate_ZeroBias()` 为异步校准请求包装；旧 `Attitude_Update()` 保留兼容，统一流程中不再单独调用。`Attitude_ResetYaw()` 由下一次 BMI088Read 更新处理。保留 Pitch 对应 X 轴转动、Roll 对应 Y 轴转动的约定；算法用于近直立运动，Yaw 是会漂移的相对角。

## 软件验证

执行环境：Windows，STM32CubeIDE 1.19.0，ARM GCC，工程内真实 C 驱动配合原生 LCC 编译器和 HAL/RTOS 桩运行。LCC 运行库缺失的 fmaxf 在测试中提供标准等价实现；没有用 Python 重写控制或采样算法。断言数包含每次模拟 I/O 对片选、超时参数和临界区的检查。

复现回归：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/drivers/run.ps1
```

| 检查 | 结果 |
| --- | --- |
| 原生 C 回归及故障注入 | PASS，18294 个断言；包含四元数模长、陀螺传播、加速度重新校正和 Yaw 复位 |
| 上电自动使能 | ID 1…4 仅在各自首帧有效反馈后使能；未提交目标时组帧电流全零；Set 无需先手动 Enable；首帧前禁用/bus-off 取消请求；超时恢复不自动使能；重新初始化清理旧目标并重新登记请求 |
| Set / Send 接口 | Set 立即计算但不发送，Send 不重新计算；双电机组帧/分总线发送、头文件宏限幅、端口状态、非法参数/映射、FIFO 满、反馈超时和 bus-off |
| 初始化与统一读取 | 调度器前初始化、任务首次接管、每次 BMI088Read 恰好三个完整采样事务且不重复初始化、400 样本校准完成后姿态就绪、失败不刷新时间、其他任务不能抢占 SPI |
| 3508 | 正反跨圈、首帧、重复初始化、正负 262144 圈边界、非零 Kd、不同 dt、位置积分限幅/禁用、MIT 大位置小误差、非法总线/ID/编码器/帧类型/DLC/参数 |
| 故障 | 20 ms 超时清零、恢复反馈保持禁用、使能后需新目标、TX FIFO 满不重试、提交失败可查、bus-off 单总线锁定、恢复失败/成功、tick 回绕 |
| BMI088 | 小端有符号换算、负温度、加速度 dummy、BUSY/超时及三类事务失败、片选释放、旧样本/调用者缓冲区保持、SPI 所有者检查、两颗传感器初始化错误分别记录 |
| 延时、校准和姿态 | DWT 回绕/计数停止，调度器前后毫秒等待，400 连续样本、运动重计数、重复序号不计数、校准超时、校准前不可用、样本过期、SPI 失败、倒置/NaN、不重复积分；四元数归一化、加速度观测门控、误差协方差更新、Yaw 复位 |
| 配置静态回归 | PASS；RAM 占用/偏移与 `.ioc` 一致，栈字数一致，轮询 GPIO 一致，RTOS tick=1 kHz |
| Debug 完整清理构建 | 编译与链接成功、无编译警告；CubeIDE 生成的 `size/objdump` 后处理仍因项目名空格失败 |
| Release 完整清理构建 | 编译与链接成功、无编译警告；CubeIDE 生成的 `size/objdump` 后处理仍因项目名空格失败 |

本次上电自动使能修改后的完整构建和回归日志：`.build/driver-regression/{startup-enable-build.log,startup-enable-regression.log}`。可执行产物：`Debug/27 dog1.elf`、`Release/27 dog1.elf`。原生回归不等同于 STM32 上执行；桩没有模拟完整的总线仲裁、寄存器时序或真实任务调度。

## 尚待实际验证

- 未执行 CubeMX 重新生成操作：已同步 `.ioc` 并做代码一致性检查；重新生成后应再次运行配置回归及完整构建。
- 未烧录或连接硬件。需实测 200 Hz 节拍、SPI 模式/波形/dummy/CS 释放、启动时 DWT 和 TIM4、两路 CAN 并行及真实 bus-off 恢复。
- 在线检查任务栈最低剩余量及 RTOS 堆余量。GCC 的 `.su` 仅提供单函数静态用量，不能代替运行时栈检查。
- EKF 噪声方差和创新门限目前是软件默认值，需用实测 BMI088 噪声、运动数据和控制响应进一步整定。
- 未烧录前不能确认四元数轴向、动态加速度门控和 Yaw 漂移是否符合整机安装方向；需按六面静止、各轴慢转和长时间静止流程实测。
- 悬空/限流条件下验证 ID 映射、转向、减速比、反馈频率、超过半圈采样间隔的解圈歧义、PID 整定、电流/功率估算和反馈丢失后的电调行为。
- 共享区只复制快照；发送与中断仍具有真实硬件延迟。断开通信后电调是否停机须由实际电调超时与急停措施验证。
