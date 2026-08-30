# STM32F407 TIM8 FOC 电流环、速度环与调用链现状

## 1. 文档基线

- 分支：`foc_test`
- 基线日期：2026-08-30
- 对象：当前 STM32 工程的 `TIM8 + ADC2 + AS5600` FOC 实机链路
- 参考实机日志：`/tmp/foc_test_speed_loop_weak_v13_clean.log`

### 结论先行

当前工程已经具备并实机运行过以下两层闭环：

1. **内层电流环**：ADC2 注入采样相电流，在 ADC ISR 中完成 Clarke/Park、D/Q PI、逆 Park、SVPWM，并把三相 duty 写入 TIM8。
2. **外层速度环**：安全任务读取 AS5600 累计机械角，按 20 ms 周期估算并滤波速度，生成 Q 轴电流目标，再交给内层电流环执行。

当前仍是固定参数的投运/测试路径，不是完整的产品级速度命令接口：目标速度固定为 `20 rad/s`，速度环运行 `8 s` 后自动关闭功率输出。三静态矢量关系检查目前记录为失败但被配置项放行，因此本次结果应称为“受限电流环和速度环已跑通”，不能等同于所有板级标定项均已验收。

## 2. 当前硬件映射

| 功能 | 当前配置 | 代码位置 |
| --- | --- | --- |
| 三相 PWM 定时器 | `TIM8`，中心对齐，`Period=4199`，重复计数器 `3` | [`Core/Src/tim.c`](../Core/Src/tim.c#L154-L234) |
| 逻辑 A 相 | `TIM8_CH3 -> PC8` | [`foc_dev.cpp`](../user_lib/devices/foc_dev.cpp#L117-L128)、[`tim.c`](../Core/Src/tim.c#L311-L328) |
| 逻辑 B 相 | `TIM8_CH2 -> PC7` | 同上 |
| 逻辑 C 相 | `TIM8_CH1 -> PC6` | 同上 |
| 驱动使能 | `MOTOR_EN -> PE5`，高有效 | [`main.h`](../Core/Inc/main.h#L60-L62)、[`gpio.c`](../Core/Src/gpio.c#L54-L71) |
| 电流采样 | `ADC2` 注入序列：`JDR1=PA2/ADC2_IN2`，`JDR2=PA3/ADC2_IN3` | [`Core/Src/adc.c`](../Core/Src/adc.c#L105-L177) |
| ADC 触发 | `TIM8_CH4` 的 `OC4REF`，上升沿触发 ADC2 注入转换 | [`Core/Src/tim.c`](../Core/Src/tim.c#L190-L216)、[`adc.c`](../Core/Src/adc.c#L149-L169) |
| 母线电压 | `ADC3_IN10/PC0` 连续采样，软件换算为母线电压 | [`foc_dev.cpp`](../user_lib/devices/foc_dev.cpp#L157-L183) |
| 转子传感器 | AS5600，I2C1，7 位地址 `0x36` | [`foc_dev.cpp`](../user_lib/devices/foc_dev.cpp#L17-L18)、[`as5600_rotor_sensor.cpp`](../user_lib/drivers/foc/sensors/encoder/as5600_rotor_sensor.cpp#L20-L61) |
| I2C1 引脚 | `PB6=SCL`，`PB7=SDA`，100 kHz | [`Core/Src/i2c.c`](../Core/Src/i2c.c#L34-L62)、[`i2c.c`](../Core/Src/i2c.c#L93-L117) |
| 电机参数 | 母线 `12 V`，极对数 `7` | [`foc_dev.cpp`](../user_lib/devices/foc_dev.cpp#L59-L65) |

`tim1_phase_driver` 是历史类名，但实现允许 `TIM1` 或 `TIM8`；当前实例实际绑定的是 `htim8`。逻辑相位与定时器通道的反向映射由 `PHASE_DRIVER_CONFIG` 明确给出：`A->CH3`、`B->CH2`、`C->CH1`。

## 3. 总体调用链

```text
main.c 外设初始化
└── osKernelInitialize()
    └── MX_FREERTOS_Init()
        └── start_init_all()
            ├── foc_dev::init()
            │   ├── 绑定 AS5600、双分流电流传感器、TIM8 三相驱动
            │   ├── foc_core::init(make_control_config())
            │   ├── 启动 ADC3 母线采样
            │   ├── 启动 ADC2 注入中断 + TIM8_CH4 采样触发
            │   ├── 创建 foc_sensor 任务（1 ms）
            │   └── 创建 foc_safety 任务（10 ms）
            ├── led_dev::init()
            └── sensor_debug::init()

TIM8_CH4 -> ADC2 注入转换 -> ADC_IRQHandler
                              └── HAL_ADCEx_InjectedConvCpltCallback()
                                  └── foc_core::run_control_from_isr()
                                      └── 电流 PI -> SVPWM -> TIM8 CCR1/2/3

foc_sensor 任务（1 ms）
└── foc_core::update_bus_sensors()
    └── AS5600 I2C 读取 -> rotor latest_topic

foc_safety 任务（10 ms）
├── 读取最新 foc_snapshot
├── 推进投运状态机
├── update_speed_loop()
│   └── AS5600 速度反馈 -> Q 轴电流目标 topic
└── foc_core::update_safety()
    └── 故障检查 -> foc_snapshot
```

初始化入口是 [`start_init_all()`](../user_lib/system/start.cpp#L8-L13)。`foc_dev::init()` 负责把具体硬件适配器绑定到 `foc_core`，随后启动采样外设和两个 FOC 任务；调试任务每 100 ms 消费 `foc_snapshot` 与投运状态并通过 UART 输出。[`foc_dev::init()`](../user_lib/devices/foc_dev.cpp#L1560-L1647) 和 [`sensor_debug`](../user_lib/debug/sensor_debug.cpp#L372-L436) 是这条启动与观测链的主要入口。

## 4. 内层电流环

### 4.1 采样到控制 ISR

电流采样不是由 TIM8 更新回调直接调用 FOC，而是由 TIM8 的 CH4 事件触发 ADC2 注入序列：

```text
TIM8_CH4 OC4REF
└── ADC2 injected rank 1/2
    ├── JDR1：A 路 ADC 原始值
    └── JDR2：B 路 ADC 原始值
        └── ADC_IRQHandler()
            └── HAL_ADCEx_InjectedConvCpltCallback(ADC2)
                └── foc_core::run_control_from_isr(timestamp_us)
```

ADC 完成回调位于 [`foc_dev.cpp`](../user_lib/devices/foc_dev.cpp#L1627-L1647)。因此控制周期的实际入口是 ADC 注入转换完成事件；`TIM8_UP_TIM13_IRQHandler()` 当前只分派 HAL 定时器处理，不直接调用 FOC 控制函数。

### 4.2 三相电流换算

当前换算参数为：

```text
ADC参考电压 = 3.3 V
ADC满量程 = 4095
分流电阻 = 0.01 Ω
放大倍数 = 50
每 count = 3.3 / (4095 × 0.01 × 50)
          = 0.00161172 A/count
```

双分流适配器在 ISR 中执行：

```text
Ia = (JDR1 - offset_A) × 0.00161172 × direction_A
Ib = (JDR2 - offset_B) × 0.00161172 × direction_B
Ic = -Ia - Ib
```

当前固定电流方向为 `direction_A=+1`、`direction_B=-1`。零偏由功率级处于三相 50% 中性 PWM、门极已使能的条件下累计 `4096` 个同步样本获得；电流传感器实现见 [`stm32_two_shunt_current_sensor.cpp`](../user_lib/drivers/foc/sensors/current_sense/stm32_two_shunt_current_sensor.cpp#L50-L166)。

### 4.3 FOC 计算顺序

运行态每次有效控制计算的顺序为：

```text
read_conversion_from_isr()
└── 校准偏移并生成 Ia/Ib/Ic
    ├── 检查驱动 Fault、样本有效性、NaN 和过流
    ├── 按 control_isr_divider 分频
    ├── 从 rotor latest_topic Peek 最新转子样本
    ├── 电角度 = 机械角 × rotor_direction × 7 + electrical_zero_offset
    ├── Clarke(current)
    ├── Park(current, electrical_angle)
    ├── D 轴 PI：target_id - measured_id
    ├── Q 轴 PI：target_iq - measured_iq
    ├── 电压矢量限幅
    ├── 逆 Park
    ├── SVPWM
    └── phase_driver::write_duty_from_isr()
        └── 写入 TIM8 的 CCR1/CCR2/CCR3
```

核心实现见 [`foc_core::run_control_from_isr()`](../user_lib/drivers/foc/foc_core.cpp#L636-L838)。其中，转子 ISR 读取只做 Topic Peek，不会在控制中断里访问 I2C；AS5600 的实际总线读取始终在任务上下文执行。

### 4.4 电流环当前参数

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `monitor_only` | `false` | 允许真实功率输出 |
| `control_period_s` | `0.0001 s` | 配置的电流控制计算周期，10 kHz |
| `control_isr_divider` | `2` | 每两个 ADC 完成回调计算一次电流控制 |
| D/Q `Kp` | `0.1251` | 两轴相同 |
| D/Q `Ki` | `370.8825` | 两轴相同 |
| PI 积分限幅 | `1.0 V` | PI 内部积分输出限幅 |
| 电压矢量限幅 | `3.0 V` | 当前代码值；本轮未使用允许的 6 V 上限 |
| 相电流过流限幅 | `1.8 A` | FOC 核心保护阈值 |
| 母线电压 | `12.0 V` | 控制配置值 |
| 转子外推窗口 | `2 ms` | 在此窗口内按速度外推机械角 |
| 转子硬超时 | `5 ms` | ISR 侧立即拒绝继续控制 |
| 命令超时 | `100 ms` | 运行态目标长期不更新时故障 |

运行中如果出现电流无效、过流、转子未就绪/过期、驱动 Fault、目标超时或 SVPWM 输出越界，核心会锁存故障、清零 PI 并关闭 `MOTOR_EN`/`TIM8.MOE`。相关安全检查见 [`foc_core.cpp`](../user_lib/drivers/foc/foc_core.cpp#L668-L838) 和 [`update_safety()`](../user_lib/drivers/foc/foc_core.cpp#L880-L984)。

## 5. AS5600 反馈与 I2C 恢复链

### 5.1 反馈链

```text
foc_sensor_task（1 ms）
└── foc_core::update_bus_sensors()
    └── as5600_rotor_sensor::update_task()
        └── I2C1 读取 AS5600 寄存器 0x0C，长度 2
            ├── 解析 12 位原始角度
            ├── 跨 0/4095 展开累计 full_angle_rad
            ├── 用样本时间戳计算 mechanical_velocity_rad_s
            └── publish(rotor_sample)
```

AS5600 的读取和角度展开见 [`as5600_rotor_sensor.cpp`](../user_lib/drivers/foc/sensors/encoder/as5600_rotor_sensor.cpp#L64-L204)。ISR 侧的 `read_from_isr()` 只读取最新 Topic，不能也不应该触发 I2C 传输。

### 5.2 当前自动恢复行为

I2C 传输失败或检测到线低/外设 Busy 时，任务侧总线管理器会：

1. `HAL_I2C_DeInit()`。
2. 将 SCL/SDA 切为推挽输出，强制低电平并制造一次 STOP 前置状态。
3. 切回开漏释放模式；若 SDA 仍被从机拉低，最多输出 9 个 SCL 脉冲。
4. 生成明确 STOP，检查两根线均为高电平。
5. 复位 I2C 外设、重新 `HAL_I2C_Init()`，再进行有限次数重试。

实现见 [`i2c_bus.cpp`](../user_lib/drivers/bus/i2c_bus.cpp#L477-L727)。恢复路径使用互斥锁并限制在任务上下文，不能放进 FOC ADC ISR；这也是 AS5600 独立供电、开发板复位后发生半帧/残留状态时的自动处理边界。

## 6. 外层速度环

### 6.1 调用位置与数据流

速度环不在 10 kHz 电流控制 ISR 中运行，而在 `foc_safety_task_entry()` 的 10 ms 任务中运行：

```text
foc_safety_task_entry（10 ms）
├── peek_snapshot()
├── update_commissioning()
├── update_speed_loop()
│   ├── 读取 snapshot.rotor.full_angle_rad
│   ├── 每 20 ms 计算累计角度差 / 时间差
│   ├── 按 rotor_direction 修正速度符号
│   ├── 一阶滤波（alpha=0.50）
│   ├── 速度误差 -> Q 轴电流目标
│   └── set_current_target(0, q_current)
└── foc_core::update_safety()
```

`set_current_target()` 发布的是 `foc_control_mode::CURRENT` 目标，随后由电流 ISR 读取该目标并执行 D/Q PI。因此速度环只决定“需要多少 Q 轴电流”，并不直接生成电压或 PWM。[`update_speed_loop()`](../user_lib/devices/foc_dev.cpp#L690-L922) 是当前速度闭环的完整实现。

### 6.2 当前控制律

速度反馈计算为：

```text
raw_velocity = (full_angle_now - full_angle_previous) / elapsed_time
measured_velocity = raw_velocity × rotor_direction
filtered_velocity += 0.50 × (measured_velocity - filtered_velocity)
```

当前速度目标与 Q 轴电流请求为：

```text
speed_target = 20.0 rad/s
speed_error = speed_target - filtered_velocity

unsaturated_iq = 0.020
               + 0.0005 × speed_error
               + 0 × integral
```

之后执行 `±0.050 A` 限幅、启动阶段 `0.030 A` 的 500 ms kick，以及电流斜率限制：

```text
最大单次电流变化 = 0.5 A/s × 0.020 s = 0.010 A
```

### 6.3 速度环参数

| 参数 | 当前值 |
| --- | ---: |
| 速度目标 | `20.0 rad/s` |
| 前馈 Q 轴电流 | `20 mA` |
| 比例增益 | `0.0005 A/(rad/s)` |
| 积分增益 | `0`，当前未启用积分 |
| 积分限幅 | `0 A` |
| Q 轴电流限幅 | `±50 mA` |
| 启动 kick | `30 mA`，持续 `500 ms` |
| 电流斜率 | `0.5 A/s` |
| 速度滤波系数 | `alpha=0.50` |
| 速度估算周期 | `20 ms` |
| 状态输出周期 | `100 ms` |
| 自动测试时长 | `8 s` |
| D 轴目标 | `0 A` |

速度环超时后执行 `foc_core::disable()`，清零速度环状态并停止功率输出；它不是持续运行的通用速度控制器。

## 7. 自动投运状态机

当前上电后由 10 ms 安全任务推进以下阶段：

```text
WAIT_CALIBRATION
    ↓
WAIT_ROTOR
    ↓
OPEN_LOOP_FORWARD -> OPEN_LOOP_PAUSE -> OPEN_LOOP_REVERSE
    ↓
ALIGN_FIRST -> DIAGNOSE_PHASE_B -> DIAGNOSE_PHASE_C
    ↓
VERIFY_CURRENT_POLARITY
    ↓
ALIGN_SECOND
    ├── 根据第二次机械位移确定 rotor_direction
    └── 计算 electrical_zero_offset_rad
    ↓
VERIFY_D_AXIS_CURRENT
    ↓
VERIFY_Q_AXIS_CURRENT
    ↓
启动速度环（stage=COMPLETE，speed_loop_active=1）
```

要注意两点：

- `FOC_OPEN_LOOP_CONTROL_FROM_TASK=false`，所以独立的 `foc_open_loop` 任务不会创建；投运阶段的开环目标由安全任务发布，但实际开环电压计算仍由 ADC 控制 ISR 执行。
- 当前 `ALLOW_CURRENT_LOOP_WITHOUT_PHASE_VECTOR_CHECK=true`。因此三静态矢量检查即使 `vectors_ok=0` 也不会阻断后续对齐、电流环和速度环。这是测试放行开关，不是对采样拓扑已经完全确认的结论。

状态机实现见 [`update_commissioning()`](../user_lib/devices/foc_dev.cpp#L1223-L1455)，D/Q 验证与速度环切换见 [`foc_dev.cpp`](../user_lib/devices/foc_dev.cpp#L1102-L1214)。

## 8. v13 实机日志证据

从 `/tmp/foc_test_speed_loop_weak_v13_clean.log` 统计到：

| 项目 | 结果 |
| --- | ---: |
| `speed_active=1` 状态样本 | `76` 条 |
| 速度反馈总体范围 | `8.259–32.942 rad/s` |
| 去掉前 12 条启动过渡后的样本 | `64` 条 |
| 稳态样本平均速度 | `19.142 rad/s` |
| 稳态速度范围 | `17.902–20.087 rad/s` |
| 稳态 Q 轴目标 | 主要为 `20–21 mA` |
| 最终 FOC 状态 | `READY` |
| 最终功率输出 | `output=0` |
| 最终故障 | `fault=0x00000000` |
| I2C 连续错误计数 | `0` |

日志中的投运状态同时显示 `vectors_ok=0`，但 `motion=1`、D/Q 验证和速度环均继续执行，原因就是上一节所述的放行配置。该日志证明的是：在当前限压、限 Q 轴电流和无负载测试条件下，AS5600 反馈、速度环目标更新、内层电流环输出和安全停机链路能够连续工作；它不证明带负载、长时间热稳定性或所有相电流拓扑检查均已完成。

## 9. 当前边界与下一步建议

当前可以确认：

- TIM8 三相 PWM 与 TIM8_CH4 → ADC2 注入采样已经形成闭合硬件时序。
- ADC ISR 内的 D/Q 电流闭环实际执行过，速度环能够通过 Q 轴电流目标调节转速。
- AS5600 读取失败时已有任务侧自动 I2C 总线恢复，不依赖手动复位总线。
- 输出关闭会同时关闭 `MOTOR_EN`、清除 TIM8 `BDTR.MOE` 并恢复中性 duty。

当前不能直接宣称为最终量产控制：

- 速度目标、限流、斜率和测试时长仍是硬编码测试参数。
- 速度环积分关闭，尚未做负载阶跃、加减速、反转和长期运行调参。
- 三静态矢量检查被放行，`vectors_ok=0` 仍需结合采样波形/硬件测量继续确认。
- 目前证据主要来自固件遥测和电机实机行为，尚未替代示波器对 PWM、ADC 触发点和相电流波形的独立验收。

