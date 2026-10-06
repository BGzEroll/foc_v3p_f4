# 反电势 PLL 实验入口

先读 [学习教程](../../docs/pll/学习教程.md) 和 [本轮实机记录](../../docs/pll/3V启动与编码器复位排查.md)。仿真通过，实机已恢复短时编码器 FOC，PLL 部分记录内部锁定，尚未完成无感接管/无编码器启动。

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

默认固件进行约 6 s 的 3 V 正反对齐，然后待机。`--start-encoder` 启动受限速度试验，默认机械目标 40 rad/s，q 电流上限 0.10 A，8 s 自动停机；`--request-switch` 是带条件及 10 s 上限的接管请求。尚不能当成熟的无感 FOC 使用。

`run_encoder_case.py --output build/case --speed 40` 会采集三窗并请求停车；`warm_reset_check.py` 验证编码器持续供电时三次 MCU 热复位并最终停车。`capture_board.py --read-existing --allow-partial` 只用于下载已停止写入的不完整故障缓冲，不能当作通过的运行数据。

`run_observer_bandwidth_study` 比较较大随机 ADC 噪声下 6000/3000/2000 rad/s 观测器参数；`analyze_3v_cases(folder)` 对本轮实机 CSV 生成速度、角度、电流与状态图及汇总。它们不包含真实逆变器或机械闭环模型。

结果目录：`results/` 为仿真摘要、图和可再生成的完整波形；`../../docs/pll/results/hardware/` 为保存的实机数据。MEX、MAT、完整仿真 CSV 不进 Git，但在本机实验目录中保留。
