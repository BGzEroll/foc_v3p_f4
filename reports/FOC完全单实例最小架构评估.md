# FOC 完全单实例最小架构评估

> 日期：2026-08-12  
> 目标：先完成一套结构简单、完全单实例的有感 FOC 核心  
> 当前约束：传感器位于 `foc/sensors`，使用虚函数 link；核心路径集中在 `foc_core`；其他模块暂缓决定。

## 1. 结论

这个方向可行，并且比现在立即建设多实例通用库更适合第一阶段硬件验证。

建议暂时明确放弃以下目标：

- 同一固件控制两台电机。
- FOC 核心作为独立通用库直接复用。
- 复杂的目录和模块拆分。
- 一开始加入速度环、位置环、LADRC、MTPA 和完整状态机。

第一阶段只保留：

```text
一个 foc_core namespace
一个私有核心状态 core_context
一个位置传感器绑定
一个电流传感器绑定
一条电流采样 → FOC 计算 → PWM 输出的核心路径
```

传感器仍然使用抽象类和虚函数，因此 AS5600、其他编码器、双电阻电流采样等具体实现可以替换，而不需要修改 `foc_core` 算法。

需要注意：完全单实例是本阶段主动选择的约束，不再满足前两份报告中的多实例目标。如果以后恢复双电机，需要重构核心状态归属。

## 2. 当前目录检查

目前已经创建：

```text
user_lib/drivers/foc/
├── foc_core.h
└── for_core.cpp
```

两个文件当前都是空文件，因此现在调整结构没有兼容成本。

`for_core.cpp` 看起来是文件名笔误，建议正式实现前改为：

```text
foc_core.cpp
```

这样 `.cpp` 的第一个 include 可以按项目规则写成：

```cpp
#include "foc_core.h"
```

## 3. 第一阶段推荐目录

先采用下面的最小结构：

```text
user_lib/drivers/foc/
├── foc_types.h
├── foc_core.h
├── foc_core.cpp
└── sensors/
    ├── rotor_sensor.h
    ├── current_sensor.h
    ├── as5600_rotor_sensor.h
    ├── as5600_rotor_sensor.cpp
    ├── stm32_two_shunt_current_sensor.h
    └── stm32_two_shunt_current_sensor.cpp
```

第一阶段不必立即创建：

```text
foc_math.cpp
pi_controller.cpp
low_pass_filter.cpp
foc_state_machine.cpp
foc_manager.cpp
```

`foc_types.h` 只保存核心和传感器共同依赖的结果、配置与样本类型，用于避免循环 include，不算额外的算法层。Clarke、Park、PI、SVPWM 等私有 helper 可以先放在 `foc_core.cpp`。当文件职责明显变多或需要主机单元测试时，再按实际需要拆出，不提前设计大量空模块。

## 4. 模块职责

### 4.1 foc_core

`foc_core` 是全工程唯一的 FOC 实例，负责：

- 保存唯一一套配置和运行状态。
- 保存传感器虚函数指针。
- 初始化和校验绑定关系。
- 电流零偏校准流程协调。
- 机械角到电角度换算。
- Clarke、Park 和逆 Park。
- D/Q 电流控制。
- SVPWM 计算。
- PWM 输出协调。
- 运行目标、状态和故障锁存。
- 生成低频只读调试快照。

`foc_core` 不负责：

- UART 输出。
- FreeRTOS 任务创建。
- I2C Mutex 或 DMA 完成等待。
- 具体 ADC 通道读取方式。
- 具体 AS5600 总线通信。
- 上层业务逻辑。

### 4.2 sensors

`foc/sensors` 负责把不同硬件转换成 FOC 核心需要的统一物理量。

位置传感器输出：

- 机械角，单位 rad。
- 机械角速度，单位 rad/s。
- 微秒时间戳。
- 序号和有效性。

电流传感器输出：

- `Ia/Ib/Ic`，单位 A。
- 微秒时间戳。
- 序号和有效性。

电流传感器内部负责：

- ADC 原始值读取。
- 零偏扣除。
- ADC 到安培的增益换算。
- 采样方向修正。
- 两电阻的第三相重构。
- 实际采样相别映射。

这些硬件细节不应放进 `foc_core`。

## 5. 传感器抽象接口

不建议设计一个万能 `sensor` 基类。位置和电流的时序、校准和输出类型完全不同，应使用两个独立接口。

### 5.1 公共结果

公共结果类型应放入很小的 `foc_types.h`，让核心与传感器都只依赖公共类型：

```cpp
enum class foc_result : uint8_t
{
    OK = 0,
    INVALID_STATE,
    NOT_LINKED,
    NOT_INITIALIZED,
    INVALID_CONFIG,
    SAMPLE_NOT_READY,
    SAMPLE_STALE,
    SENSOR_FAULT,
    OUTPUT_FAULT
};
```

后续需要共享的 `foc_config`、`foc_target` 和 `foc_snapshot` 也放在这个文件中，但算法状态仍留在 `foc_core.cpp` 内部。

### 5.2 位置传感器

建议 `rotor_sensor.h`：

```cpp
#ifndef ROTOR_SENSOR_H
#define ROTOR_SENSOR_H

#include "drivers/foc/foc_types.h"
#include <stdint.h>

struct rotor_sample
{
    uint32_t sequence = 0U;
    uint32_t timestamp_us = 0U;
    float mechanical_angle_rad = 0.0f;
    float mechanical_velocity_rad_s = 0.0f;
    bool valid = false;
};

class rotor_sensor
{
    public:
        virtual ~rotor_sensor() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result read_latest(rotor_sample &sample) = 0;
};

#endif
```

这可以避免 `foc_core.h` 和传感器头文件循环 include。`foc_types.h` 是少数值得从一开始独立的文件：

```text
foc_types.h
foc_core.h
sensors/rotor_sensor.h
sensors/current_sensor.h
```

### 5.3 电流传感器

建议 `current_sensor.h`：

```cpp
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "drivers/foc/foc_types.h"
#include <stdint.h>

struct phase_current_sample
{
    uint32_t sequence = 0U;
    uint32_t timestamp_us = 0U;
    float current_a = 0.0f;
    float current_b = 0.0f;
    float current_c = 0.0f;
    bool valid = false;
};

class current_sensor
{
    public:
        virtual ~current_sensor() = default;

    public:
        virtual foc_result init() = 0;
        virtual foc_result calibrate() = 0;

    public:
        virtual foc_result read_latest(phase_current_sample &sample) = 0;
};

#endif
```

`read_latest()` 必须遵循统一契约：

- 非阻塞。
- 不等待 Mutex、Semaphore、Queue 或 DMA。
- 在 FOC ISR 调用时必须 ISR 安全。
- 返回已经完成采集的最新数据，而不是启动采集后等待。
- 样本必须携带时间戳和有效性。

## 6. foc_core 的单例 API

建议 `foc_core.h` 只暴露真正需要外部调用的接口：

```cpp
#ifndef FOC_CORE_H
#define FOC_CORE_H

#include "drivers/foc/foc_types.h"
#include "drivers/foc/sensors/current_sensor.h"
#include "drivers/foc/sensors/rotor_sensor.h"
#include <stdint.h>

namespace foc_core
{
    foc_result link_rotor_sensor(rotor_sensor &sensor);
    foc_result link_current_sensor(current_sensor &sensor);
    foc_result set_target(const foc_target &target);
    foc_snapshot snapshot();
    foc_result enable();
    void disable();
    foc_result run_current_loop_from_isr(uint32_t timestamp_us);
    foc_result init(const foc_config &config);
}

#endif
```

头文件中不应暴露：

- PI 内部状态。
- 当前传感器指针。
- 三角函数缓存。
- 电流和电压中间量。
- PWM 分频计数。
- 故障确认计数器。
- 可直接修改的全局结构体。

## 7. 单实例状态组织

完全单实例不代表把状态写成几十个散落的全局变量。建议在 `foc_core.cpp` 内只保存一个私有上下文：

```cpp
struct foc_context
{
    foc_config config{};
    foc_target target{};
    foc_snapshot current_snapshot{};
    pi_state d_axis_pi{};
    pi_state q_axis_pi{};
    rotor_sensor *rotor = nullptr;
    current_sensor *current = nullptr;
    uint32_t loop_sequence = 0U;
    uint32_t stale_sample_count = 0U;
    foc_state state = foc_state::UNINITIALIZED;
    foc_fault fault = foc_fault::NONE;
};

static foc_context core_context;
```

优点：

- 明确整个模块只有一份状态。
- 不会重现 SguanFOC 中函数内 `static` 计数器相互隐藏的问题。
- 调试器中只需观察一个上下文。
- 以后如果确实需要多实例，可以把 `foc_context` 移入 class，而不必重新寻找所有全局变量。

规则：

- 所有可变状态必须进入 `core_context`。
- 私有 helper 不允许再创建有持续状态的函数内 `static` 变量。
- 只读数学常量和只读查表可以是独立 `static constexpr`。
- 外部不能取得 `core_context &`。

## 8. link 的生命周期

推荐调用顺序：

```text
创建具体传感器静态对象
    → foc_core::link_rotor_sensor()
    → foc_core::link_current_sensor()
    → foc_core::init(config)
    → 电流零偏校准
    → 启动 PWM/ADC 同步触发
    → foc_core::enable()
```

link 约束：

- 具体传感器对象必须具有静态生命周期。
- link 只保存指针，不拥有对象。
- `init()` 后禁止重新 link。
- 重复绑定不同对象应返回 `INVALID_STATE`。
- 缺少必需传感器时 `init()` 必须失败。
- 初始化失败时保持 PWM 禁能。

## 9. foc_core 核心路径

第一阶段建议只实现下面这条路径：

```text
ADC 注入转换完成 ISR
    → foc_core::run_current_loop_from_isr(timestamp_us)
        → 检查 state/fault
        → current_sensor::read_latest()
        → rotor_sensor::read_latest()
        → 检查样本 valid/sequence/timestamp
        → 机械角换算电角度
        → sin/cos
        → Clarke: Iabc → Iαβ
        → Park: Iαβ → Id/Iq
        → D/Q 电流 PI
        → Ud/Uq 电压矢量限幅
        → 逆 Park
        → SVPWM
        → 输出三相 duty
        → 更新轻量快照
```

首版可以只支持：

```text
VOLTAGE_MODE   直接给定 Ud/Uq，用于相序和角度方向验证
CURRENT_MODE   给定 Id/Iq，运行 D/Q 电流 PI
```

速度环和位置环以后再讨论，避免首版核心路径同时承担过多职责。

## 10. 目前不能省略的决策

“其余待商讨”是合理的，但下面几项必须在真正运行电机前确定。

### 10.1 PWM 输出边界

传感器 link 已确定，但 FOC 最终必须把 duty 写到硬件。可选方案：

1. `foc_core` 直接操作 TIM1。
2. 再增加一个虚函数 `phase_driver` 并通过 `link_driver()` 绑定。
3. 使用函数指针回调写 duty。

从结构上最推荐 `phase_driver`，但可以暂缓到数学和传感器接口完成后再定。不要在没有明确安全禁能接口时上电闭环。

### 10.2 高频调度入口

需要确定使用 ADC 注入完成中断还是 TIM 更新中断。当前硬件已经配置 PWM 同步 ADC 注入触发，推荐最终由 ADC 数据就绪事件运行电流环。

### 10.3 电流零偏校准

必须确定：

- PWM/门极处于什么状态时校准。
- 采多少次。
- 采样间隔。
- 失败阈值。
- 校准值由电流传感器还是核心持有。

推荐由 `current_sensor` 持有校准状态，`foc_core` 只负责协调时机。

### 10.4 配置来源

需要确定极对数、角度方向、零偏、采样周期、电流限幅、PI 参数和母线电压如何传入。首版推荐一个 `foc_config` 结构体，在 `init()` 时复制并校验。

### 10.5 故障后的硬件动作

必须有真正的门极禁能路径。将 Ud/Uq 设为零或三相写 50% duty 不等于关闭功率输出。

## 11. AS5600 的现实限制

现有 `as5600_dev` 使用：

```text
FreeRTOS 任务
    → I2C 总线
    → latest_topic
    → 普通 xQueuePeek
```

它不能直接作为高频 ISR 的 `rotor_sensor::read_latest()` 实现，因为：

- 普通 `xQueuePeek()` 不能在 ISR 使用。
- I2C Mutex 和完成信号量不能进入 FOC ISR。
- 目前 1 ms 更新周期远低于 20 kHz 电流环。
- 毫秒时间戳不足以准确补偿高频角度延迟。

因此 `as5600_rotor_sensor` 需要一个 ISR 安全、非阻塞的缓存层。虚函数只能统一接口，不能自动解决采样带宽和延迟。

首版可以先在低速、低电压条件下使用最新角度加速度估算做有限前推，但必须设置最大样本年龄，过期就禁能。

## 12. 安全与 RTOS 边界

即使目标是最小实现，下面这些不能删掉：

- 初始化前不允许 enable。
- 任一传感器未 link 或初始化失败时不允许 enable。
- 相电流和 D/Q 电流双向限幅。
- 母线电压为零、过低或异常时禁止调制计算。
- 样本过期、无效或不连续时锁存故障。
- NaN/Inf 检查。
- duty 范围限制。
- 明确的硬件 disable。
- 故障不会自动恢复输出。

高频 ISR 内禁止：

- UART 打印。
- `snprintf()`。
- 普通 Queue/Mutex/Semaphore API。
- 等待 I2C/SPI/DMA 完成。
- `vTaskDelay()`。
- 动态内存。

调试输出仍由 `user_lib/debug` 中的消费者读取低频快照完成。

## 13. 最小实施顺序

建议按以下顺序编码：

1. 把 `for_core.cpp` 更名为 `foc_core.cpp`。
2. 建立 `foc_types.h`。
3. 建立 `sensors/rotor_sensor.h` 和 `sensors/current_sensor.h`。
4. 在 `foc_core.h` 写出单例 link、init、enable、disable API。
5. 在 `foc_core.cpp` 建立唯一 `core_context`。
6. 实现角度归一化、Clarke、Park、逆 Park 和 SVPWM 私有 helper。
7. 为数学 helper 编写主机侧测试。
8. 确定 PWM 输出边界。
9. 实现 STM32 双电阻电流传感器。
10. 实现 AS5600 的非阻塞缓存适配。
11. 先验证低压电压模式。
12. 再实现和调试 D/Q 电流 PI。

## 14. 最终判断

完全单实例方案适合作为当前第一版，推荐最终结构为：

```text
foc_core namespace
    ├── 唯一 core_context
    ├── link rotor_sensor
    ├── link current_sensor
    ├── 初始化、使能、故障与目标 API
    └── 完整高频 FOC 核心路径

foc/sensors
    ├── 独立的抽象传感器接口
    └── 具体硬件实现
```

现阶段不需要引入 `foc_motor` 类、motor ID、context 数组或双电机管理器。

但应保留三个结构纪律：

1. 所有持续状态统一放进私有 `core_context`。
2. 传感器只通过虚函数 link，核心不直接依赖具体 AS5600/ADC 类。
3. 高频路径不依赖 RTOS 等待和通信。

在这些边界下，先做完全单实例不会把后续重构彻底堵死，也能显著降低第一阶段的实现和调试复杂度。
