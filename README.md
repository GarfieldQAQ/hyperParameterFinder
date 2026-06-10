# hyperParameterFinder

用于自动搜索 Xtepper 控制参数。

## 当前功能

- 使用 C++17 编写，无 Python 运行时依赖
- 通过 Windows 串口连接 Xtepper 固件终端
- 自动执行 FOC 电流环 `iq` 阶跃实验
- 采集 VOFA JustFloat 电流环通道
- 对 `ckp/cki` 网格搜索并输出 CSV 结果

## 构建

```powershell
cmake -S . -B build
cmake --build build --config Release
```

生成程序：

```text
build\Release\hyperParameterFinder.exe
```

## 电流环 PI 搜索

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

输出会写到 `output/current_YYYYMMDD_HHMMSS/`：

- 每组参数一份响应 CSV
- `summary.csv` 按评分排序

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
