# 反电势 PLL 实验（test/pll）

MATLAB/Simulink 仿真通过；实机旁路估算尚未可靠锁定，无感接管未执行，无编码器启动未实现。硬件已关断，默认固件短时静态对齐后待机。

- [从零学习与全部复现步骤](docs/pll/学习教程.md)
- [实机数据、失败现象与下一步排查](docs/pll/实机记录.md)
- [MATLAB / Simulink 脚本入口](scripts/pll/README.md)

推荐固件构建：`cmake --preset PLLRelease`、`cmake --build --preset PLLRelease`。使用采集脚本时必须配同次构建的 ELF/BIN，不要在转动时用断点暂停 CPU。
