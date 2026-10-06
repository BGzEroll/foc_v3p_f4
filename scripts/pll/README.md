# 反电势 PLL 实验入口

当前默认固件已完成直接无感启动和短时闭环，默认上电待机。先读 [直接无感启动与实机验证](../../docs/pll/直接无感PLL启动与实机验证.md)。理论和原始电气 MATLAB/Simulink 流程见 [学习教程](../../docs/pll/学习教程.md)，早期有感实验见 [历史记录](../../docs/pll/3V启动与编码器复位排查.md)。

MATLAB 中切到本目录后：

```matlab
setenv('MW_MINGW64_LOC','C:\software\mingw64')
run_direct_start_simulation    % 理想机械模型 + 同份启动/观测器 C
analyze_direct_cases('../../docs/pll/results/hardware/direct')
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

按新文档在输出关断时烧录匹配 ELF/BIN、复位初始化后：

```bash
python3 scripts/pll/board_status.py
python3 scripts/pll/run_direct_case.py --output build/direct_case --no-encoder
python3 scripts/pll/board_status.py
python3 scripts/pll/read_rtt.py --seconds 3 --output build/rtt.log
```

`run_direct_case.py` 不复位、不烧录；要求直接模式 READY、无故障且驱动关闭，显式请求启动，采集多窗并最终确认输出关闭。固件限制 PLL 反馈总计 10 s（含约 0.3 s 过渡），全部启动/运行最多 18 s；结束后需在关断状态复位，才可开始下一次。`--no-encoder` 在启动前关闭 AS5600 采样，AS5600 物理上仍连接。

`run_static_vectors.py --output build/static_vectors` 为六角度 3 V 静态诊断，每角度 0.7 s，共 4.2 s，420/512 的冻结缓冲是明确标记的诊断记录。`capture_board.py --stop` 请求停车。当前 7 项主机测试覆盖新增启动守卫与生产电流驱动的 CB 映射。

RTT 读取器从匹配 ELF 解析 `_SEGGER_RTT`，校验 SRAM 标识，停止旧轮询后重新设置精确地址。空字节日志会报错并保留原文件，不作为成功证据。

旧 `run_encoder_case.py`、`--start-encoder`、`--request-switch` 和旁路相序诊断仅供关闭 `PLL_DIRECT_SENSORLESS` 的历史实验，当前脚本拒绝直接模式下这些动作。`warm_reset_check.py` 的早期对齐结论见历史文档。`capture_board.py --read-existing --allow-partial` 只下载停止写入的不完整缓冲。不要把历史 ELF 的 RAM 地址用于当前固件。

`run_observer_bandwidth_study` 比较较大随机 ADC 噪声下 6000/3000/2000 rad/s 观测器参数；`analyze_3v_cases(folder)` 对本轮实机 CSV 生成速度、角度、电流与状态图及汇总。它们不包含真实逆变器或机械闭环模型。

结果目录：`results/` 为仿真摘要、图和可再生成的完整波形；`../../docs/pll/results/hardware/direct/` 为本轮实机 CSV/JSON、有效 RTT、图和映像 SHA256；`hardware/3v/` 为历史数据。MEX、MAT、完整仿真 CSV 不进 Git，但在本机实验目录中保留。原 Simulink 模型是观测器学习模型，未加入新增机械启动状态机。
