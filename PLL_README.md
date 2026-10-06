# 反电势 PLL 实验（test/pll）

MATLAB/Simulink 仿真通过；实机已恢复短时编码器 FOC 转动，PLL 在部分记录中内部锁定，接管条件仍未持续达标，无感接管及无编码器启动未完成。默认固件进行约 6 s 的 3 V 正反对齐后待机，异常会关断。

- [从零学习与全部复现步骤](docs/pll/学习教程.md)
- [本轮 3 V 启动、热复位与对齐排查](docs/pll/3V启动与编码器复位排查.md)
- [实机数据、失败现象与下一步排查](docs/pll/实机记录.md)
- [MATLAB / Simulink 脚本入口](scripts/pll/README.md)

推荐固件构建：`cmake --preset PLLRelease`、`cmake --build --preset PLLRelease`。使用采集脚本时必须配同次构建的 ELF/BIN，不要在转动时用断点暂停 CPU。
