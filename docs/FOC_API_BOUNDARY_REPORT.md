# FOC 库 API 边界与结构调整报告

## 1. 结论

本次重构将 FOC 核心收敛为：

~~~text
传感器样本 Topic -> FOC 控制计算 -> 功率输出回调
                              └-> Snapshot Topic
~~~

当前实现允许直接依赖 ESP32/FreeRTOS，删除了原有的三缓冲交换、虚拟
`phase_driver`、传感器读取接口和公共数学头文件。核心仍然是第一阶段的
D-Q 电流环，不包含速度环、位置环和真实硬件适配器。

当前仓库没有生产代码调用 FOC，因此本次 API 切换没有兼容迁移负担。

## 2. 当前文件边界

~~~text
user_lib/drivers/foc/
├── foc_core.h/.cpp                 # 公共 API、Topic、控制实现和内部数学
├── foc_types.h                     # 配置、样本、目标、Snapshot 和结果类型
└── tests/
    ├── foc_host_test.cpp           # 主机行为测试
    └── host_include/freertos/      # 主机测试用的 FreeRTOS Queue 替身
~~~

`foc_core.h` 依赖 `system/topic.h`，因此 FOC 公共边界明确依赖 FreeRTOS。
这符合当前 ESP32 项目的使用方式，也避免为了平台抽象继续增加接口层。

## 3. 公共 API

`foc_core` 对外保留：

~~~cpp
foc_result init(const foc_config &config,
    const foc_output &output);
foc_result enable();
void disable();
foc_result set_target(const foc_target &target);
foc_result core_loop(uint32_t timestamp_us);
foc_result core_loop_from_isr(uint32_t timestamp_us,
    BaseType_t &higher_priority_task_woken);
foc_result clear_fault();
foc_topic_access topics();
~~~

### 3.1 功率输出边界

`foc_output` 使用固定回调表，不再使用虚拟类：

~~~cpp
struct foc_output
{
    void *context = nullptr;
    foc_result (*init)(void *context) = nullptr;
    foc_result (*enable)(void *context) = nullptr;
    void (*disable)(void *context) = nullptr;
    foc_result (*apply_duty)(void *context,
        const phase_duty &duty) = nullptr;
    foc_result (*apply_duty_from_isr)(void *context,
        const phase_duty &duty) = nullptr;
    bool (*fault_active)(void *context) = nullptr;
};
~~~

`disable()` 和 `fault_active()` 会从控制故障路径调用，必须可在 ISR 中执行。
`apply_duty_from_isr()` 必须位于 IRAM、不可阻塞、不可使用任务 API。

### 3.2 Topic 边界

~~~cpp
struct foc_topic_access
{
    topic::latest_topic<rotor_sample> &rotor;
    topic::latest_topic<phase_current_sample> &current;
    const topic::latest_topic<foc_snapshot> &snapshot;
};
~~~

每个 `foc_core` 实例私有持有四个 `latest_topic`：

| Topic | 所有者 | 对外访问方式 |
|---|---|---|
| Target | `foc_core` | 外部仅由 `set_target()` 写入；生命周期内部可发布 Disabled Target |
| 转子样本 | `foc_core` | `topics().rotor` 发布 |
| 三相电流样本 | `foc_core` | `topics().current` 发布 |
| Snapshot | `foc_core` | `topics().snapshot` 只读 |

Target 仍由 `set_target()` 做模式、浮点数和电流幅值校验，然后发布到内部
Target Topic。这样既使用了 `topic.h` 的跨上下文传递能力，又不会允许外部
绕过安全校验直接写入目标。

`set_target()` 是任务上下文 API；FAULT 状态下必须先完成
`clear_fault()`，清故障会重新发布 Disabled Target。

传感器驱动应在初始化完成后保存 Topic 引用，并按执行上下文选择：

~~~cpp
topics.rotor.publish(sample);
topics.rotor.publish_from_isr(sample, higher_priority_task_woken);
topics.current.publish(sample);
topics.current.publish_from_isr(sample, higher_priority_task_woken);
~~~

`foc_core` 不再调用传感器的 `read()`，因此 I2C、UART 等可能阻塞的操作不
会进入控制循环。

## 4. 两种控制循环

`core_loop()` 和 `core_loop_from_isr()` 通过模板实例化共用同一份控制顺序：

~~~text
读取 Target、转子和电流 Topic
    -> 检查样本有效性和新鲜度
    -> Clarke/Park
    -> D-Q PI
    -> 电压限幅
    -> 反 Park/SVPWM
    -> 写入占空比
    -> 按频率发布 Snapshot
~~~

普通循环使用 `peek()`、`publish()` 和普通输出回调；ISR 循环只使用
`peek_from_isr()`、`publish_from_isr()` 和 ISR 输出回调。

`core_loop_from_isr()` 不调用 `portYIELD_FROM_ISR()`。调用者负责在一次 ISR
中完成所有样本发布和控制计算后统一调用：

~~~cpp
portYIELD_FROM_ISR(higher_priority_task_woken);
~~~

一个实例只能选择一种循环方式，不能让普通任务和 ISR 同时修改同一个核心
运行状态。

## 5. ISR、IRAM 和非阻塞约束

ISR 控制路径包含以下约束：

- 不调用 I2C、UART、日志、延时、堆分配、锁或任务版 Queue API。
- 只读取已经初始化的静态 Topic。
- `topic::latest_topic` 的 `peek_from_isr()` 和 `publish_from_isr()` 已标记
  `IRAM_ATTR`，内部只调用 FreeRTOS FromISR Queue API。
- `foc_core` 状态、PI 积分、占空比和 Topic 存储必须在实例生命周期内有效。
- 使用 ISR 循环时，`foc_core` 实例、Topic 存储和输出回调上下文应放在芯片
  内部 RAM，不应依赖 ISR 不可访问的外部 PSRAM。
- 功率输出的 ISR 回调必须由平台侧放入 IRAM，并且直接使用 ISR 安全的硬件
  写入路径。
- 内部数学函数位于 `foc_core.cpp`，不再依赖公共 `foc_math.h`。
- 控制路径使用内部有限性检查、三角函数近似和固定次数平方根倒数迭代，避
 免把标准 `libm` 函数或浮点除法辅助函数作为隐含的 Flash 依赖；编译器生成
  的 `memcpy`/`memset` 依赖仍必须在目标链接图中确认位于 ROM 或 IRAM。

最终固件仍需通过链接图检查 `core_loop_from_isr()` 的传递调用链；平台输出
回调属于应用侧责任，不能仅由 FOC 核心判断其 IRAM 属性。

## 6. 样本新鲜度与故障

转子样本和电流样本均要求：

~~~cpp
sample.valid == true
static_cast<uint32_t>(timestamp_us - sample.timestamp_us) <= 5000U
~~~

样本年龄为 5000 µs 时仍可使用，超过 5000 µs、Topic 未初始化、Topic 无样本
或 `valid == false` 时进入对应传感器故障。

故障动作固定为：

1. 记录 `ROTOR_SENSOR` 或 `CURRENT_SENSOR`。
2. 关闭功率输出。
3. 清零 PI 积分、输出电压和占空比。
4. 进入 `FAULT`。
5. 立即发布一次故障 Snapshot。

过流、驱动故障、非法浮点和输出范围错误沿用相同的安全停机路径。

## 7. Snapshot 语义

`foc_snapshot` 继续包含：

- `state`
- `fault_flags`
- 转子角度和速度
- 三相电流与 Alpha-Beta/D-Q 电流
- D-Q 电压
- 三相占空比
- 母线电压和母线电流字段

状态和故障不再通过 `state()`、`faults()` 或 `snapshot()` 单独读取，只通过
Snapshot Topic 对外提供。Snapshot 正常情况下按时间降频，最短发布间隔为
1000 µs（最高 1 kHz）；初始化、故障、使能、禁用和清故障会立即发布。

Snapshot 消费者使用：

~~~cpp
foc_snapshot snapshot{};
auto access = motor.topics();
if(access.snapshot.peek(snapshot, 0U))
{
    // 使用 snapshot.sequence 判断是否有新数据
}
~~~

## 8. 生命周期约束

以下 API 都是任务上下文 API：

| API | 前置条件 | 结果 |
|---|---|---|
| `enable()` | ISR 已停止，状态为 `READY`，Target 为有效电流目标 | 中性占空比后使能功率级，进入 `RUNNING` |
| `disable()` | ISR 已停止 | 关闭输出、发布 Disabled Target、清零控制状态 |
| `clear_fault()` | ISR 已停止、输出已关闭、硬件故障已解除 | 清故障、发布 Disabled Target、回到 `READY` |

这里的“停止 ISR”包括：停止控制定时器/中断源，并确认已经进入的 ISR 调用
已经返回。核心不增加运行时锁或 ISR 检测；这是应用层调用契约。

清故障不会自动使能电机。调用顺序必须是：

~~~text
停止 ISR -> clear_fault() -> set_target() -> enable() -> 启动 ISR
~~~

## 9. 已完成的精简点

- `high_freq_loop()` 改为 `core_loop()`。
- 新增 `core_loop_from_isr()`。
- 删除自定义 Target/Snapshot 三缓冲和相关原子变量。
- 删除 `phase_driver` 虚拟接口。
- 删除 `rotor_sensor`、`current_sensor` 虚拟读取接口。
- 删除 `foc_math.h/.cpp`。
- 数学中间类型和函数改为 `foc_core.cpp` 内部实现。
- 内部状态改为普通成员，由生命周期串行契约保证一致性。
- Snapshot 改用 `topic::latest_topic`。
- Topic FromISR 接口和 FOC ISR 控制函数均已放入 IRAM 段，数学路径不再依赖
  `libm` 或 `__divsf3`。

## 10. 验证和剩余边界

已完成：

- 主机编译：GCC、C++17、`-Wall -Wextra -Werror -pthread`。
- 主机行为测试：坐标变换、PI/SVPWM、双实例隔离、Topic 并发、传感器
  5 ms 边界、过流、驱动故障、故障恢复和 ISR 专用 API。
- ESP-IDF 6.0.2 直接 CMake 构建成功。
- ISR 测试确认核心使用 FromISR Topic API 和 ISR 输出回调。
- ESP32 FOC 对象段审计确认 `core_loop_from_isr()`、共同控制实现、数学函数和
  Topic FromISR 实例位于 `.iram1`；当前示例固件没有生产调用者，因此 FOC
  符号未被拉入最终 ELF，应用接入后仍需审计输出回调和最终链接图。

尚未完成：

- 真实 ESP32 ADC、转子传感器和 PWM 输出适配器。
- 实机电流环稳定性、采样同步和控制周期测量。
- 最终应用输出回调的 IRAM/硬件寄存器审计。
- 20 kHz 控制频率下三个 Topic `peek_from_isr()` 的实际耗时测量。

因此当前状态适合作为“已收缩边界的 FOC 软件核心”，还不是可以直接驱动
真实电机的完整产品驱动。
