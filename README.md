# hyperParameterFinder

用于把 Xtepper 接入成熟控制参数整定方法。

项目重点不是重新发明 PID 优化算法，而是提供高效、可重复、可守护的硬件实验执行器。

## 当前功能

- 使用 C++17 编写，无 Python 运行时依赖
- 通过 Windows 串口连接 Xtepper 固件终端
- 自动执行 FOC 电流环 `iq` 阶跃实验
- 采集 VOFA JustFloat 电流环通道
- 对 `ckp/cki` 做基础网格实验或低预算 Bayesian Optimization，并输出 CSV 结果

当前内置 BO 是二维电流环参数的轻量实现，用于先把硬件实验闭环跑起来。长期仍建议把 C++ 程序作为实验执行器，再接 Optuna、MATLAB、LabVIEW 等成熟工具链。

## 构建

```powershell
cmake -S . -B build
cmake --build build --config Release
```

生成程序：

```text
build\Release\hyperParameterFinder.exe
```

## 电流环 PI 基础实验

```powershell
.\build\Release\hyperParameterFinder.exe --port COM7 --baud 115200 --loop current
```

常用参数：

```powershell
.\build\Release\hyperParameterFinder.exe ^
  --port COM7 ^
  --kp 0.005:0.05:0.005 ^
  --ki 0.5:5.0:0.5 ^
  --target-iq 0.2 ^
  --limit 0.3
```

使用 Bayesian Optimization：

```powershell
.\build\Release\hyperParameterFinder.exe ^
  --port COM7 ^
  --optimizer bo ^
  --kp 0.005:0.05:0.005 ^
  --ki 0.5:5.0:0.5 ^
  --bo-initial 5 ^
  --bo-iterations 20 ^
  --bo-candidates 400 ^
  --target-iq 0.2 ^
  --limit 0.3 ^
  --closed-loop-duty 0.18
```

输出会写到 `output/current_YYYYMMDD_HHMMSS/`：

- 每组参数一份响应 CSV
- `summary.csv` 按评分排序

## 电流环有效区间

当前硬件上，电流环大约从 18% 占空比后才进入可观测、可闭环的区域。

因此程序默认：

```text
--closed-loop-duty 0.18
```

评分时只使用 `|vq| >= closed_loop_duty` 后的样本来评价 PI 动态。低于这个阈值的区间主要反映采样窗口、DRV8701 SO 建立时间和低占空比测量死区，不应当算作 PID 参数整定效果。

如果有效样本过少，程序会对该参数组重罚，并在 `summary.csv` 中输出 `effective_ratio`。

## 长期路线

长期架构是：

```text
C++ 硬件实验执行器
  -> 串口命令
  -> VOFA 数据采集
  -> 安全停止
  -> CSV/JSON 结果

成熟外部调参方法
  -> Relay / step tuning baseline
  -> Bayesian Optimization
  -> 坐标搜索
  -> MATLAB / LabVIEW / Optuna 等工具链
```

C++ 程序负责“把一次真实硬件实验做准”，优化算法优先复用现成工具。

## 固件侧要求

程序默认使用当前 Xtepper 命令：

- `foc prep current`
- `foc limit X`
- `foc ckp X`
- `foc cki X`
- `foc id X`
- `foc iq X`
- `vofa current on`
- `vofa off`
- `foc on/off`

VOFA current 通道顺序需要保持：

```text
targetId,targetIq,id,iq,idRaw,iqRaw,vd,vq,va,vb,id_i,iq_i
```
