# 反电势 PLL 实验（test/pll）

已完成本板上的直接无感启动：3 V 定位 → 开环电流加速 → 反电势 PLL 接管。关闭 AS5600 采样后，仍完成 10 s 的有界 PLL 反馈试验（其中全 PLL 角度阶段约 9.7 s），最大相电流约 0.92 A。当前是短时台架验证，负载、反转、多转速和长时间运行尚未验证。

`PLLRelease` 默认直接无感模式，上电只初始化、校准并待机；显式请求才启动，超时或异常关断。关键修正是两电阻采样的实际相序/极性：ADC0 → −C，ADC1 → −B，A = −B − C。

- [直接无感启动：原理、修正、实机证据与复现](docs/pll/直接无感PLL启动与实机验证.md)
- [从零学习 PLL 与原始 MATLAB / Simulink 流程](docs/pll/学习教程.md)
- [历史：3 V 启动、热复位与对齐排查](docs/pll/3V启动与编码器复位排查.md)
- [实机数据、失败现象与下一步排查](docs/pll/实机记录.md)
- [MATLAB / Simulink 脚本入口](scripts/pll/README.md)

推荐固件构建：`cmake --preset PLLRelease`、`cmake --build --preset PLLRelease`。使用采集脚本时必须配同次构建的 ELF/BIN，不要在转动时用断点暂停 CPU。
