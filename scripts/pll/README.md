# 反电势 PLL 实验入口

先读 [学习教程](../../docs/pll/学习教程.md) 和 [实机记录](../../docs/pll/实机记录.md)。当前仿真通过，实机未稳定锁定，没有完成无感接管/无编码器启动。

MATLAB 中切到本目录后：

```matlab
setenv('MW_MINGW64_LOC','C:\software\mingw64')
run_pll_simulation
build_simulink_demo
open_system('sensorless_pll_learning.slx')
```

Debian 仓库根目录中：

```bash
cmake --preset PLLRelease
cmake --build --preset PLLRelease
cmake -S tests/pll -B build/tests-pll -G Ninja
cmake --build build/tests-pll
ctest --test-dir build/tests-pll --output-on-failure
```

运行 OpenOCD、烧录、USBIP、启动/停止、数据采集和 RTT 的完整步骤见教程第 8 节。采集脚本默认使用 `build/PLLRelease/project.elf`，必须匹配刚烧录的固件；不要用调试器在运行的电机上打断点。

默认固件完成既有短时静态对齐后待机，`--start-encoder` 只启动原有 0.10 A 编码器反馈电流控制；`--request-switch` 是带条件和 10 秒上限的实验接管请求。本次不满足接管条件，尚不能把它当成熟的无感 FOC 使用。

结果目录：`results/` 为仿真摘要、图和可再生成的完整波形；`../../docs/pll/results/hardware/` 为保存的实机数据。MEX、MAT、完整仿真 CSV 不进 Git，但在本机实验目录中保留。
