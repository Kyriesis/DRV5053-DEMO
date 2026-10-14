# DRV5053-DEMO

TI DRV5053 线性霍尔传感器 + 微雪 ESP32-S3-Pico 的磁体位置检测方案：500Hz ADC 采样，网页实时曲线显示电压/位置。

## 硬件

- **开发板**: 微雪 ESP32-S3-Pico（主控 ESP32-S3R2，实测 8MB Flash + 2MB OPI PSRAM）
- **传感器**: TI DRV5053 线性霍尔 IC（SOT-23），输出固定 0.2~1.8V，零点 1.0V，与供电电压无关
- **接线**:

| DRV5053 (SOT-23) | 开发板 | 物理引脚 |
|---|---|---|
| VCC (1脚) | 3V3 | 左侧排第 5 焊盘 |
| OUT (2脚) | GP1 (ADC1_CH0) | 左侧排第 17 焊盘 |
| GND (3脚) | GND | 左侧排第 18 焊盘（GP1 正下方） |

- VCC 与 GND 之间在传感器旁并一颗 ≥0.01µF 瓷片电容
- 选用 ADC1 通道是因为 ESP32-S3 的 ADC2 与 WiFi 硬件冲突
- 磁铁靠近时电压从 1.0V 向 0.2V（负斜率型号）或 1.8V（正斜率型号）线性变化

## 固件编译烧录

需要 [PlatformIO](https://platformio.org/)（VS Code 插件或 Core）：

```bash
pio run -t upload      # 编译并烧录
pio device monitor     # 串口监视 (115200, CSV: t_ms, v_raw, v_filt)
```

或 VS Code 状态栏：✔ Build → → Upload → 插头（Serial Monitor）。

## 网页实时曲线

双击 `tools/plot.bat`（自动打开浏览器访问 http://localhost:8080）：

- **滚轮** — 缩放时间轴（1~60 秒）
- **启动/停止监视** — 控制串口连接；停止后释放 COM 口，可去烧录固件
- **标定输入框** — 两点标定（远点电压/位置、近点电压/位置）自动保存于浏览器，实时换算位置 mm
- 灰线=原始电压，绿线=滤波电压，橙色虚线=位置曲线（右侧 mm 轴）

运行环境：Windows 10+（自带 USB 串口驱动），Python 来源按优先级自动查找
PlatformIO venv → 系统 PATH；缺 pyserial 时自动安装。
使用前关闭 VS Code 的串口监视器（避免 COM 口占用）。

## 技术要点

- **严格 500Hz 采样**：`esp_timer` 硬件定时器回调中直接采样（8 次过采样平均 + EMA 低通，替代模拟 RC 滤波），经队列送打印线程——采样节拍与串口打印速度完全解耦
- **双核分工**：采样/打印在核心 0，板载 RGB 指示灯闪烁在核心 1（Arduino loop）
- **ADC 配置**：ADC1_CH0 + 6dB 衰减（满量程约 2.2V，恰好覆盖 5053 的 0.2~1.8V 输出，分辨率利用率 ~82%）

## 仓库结构

```
src/main.cpp            固件源码
tools/plot.py           网页曲线服务器 (零第三方依赖, 仅需 pyserial)
tools/plot.bat          双击启动器
docs/DRV5053_datasheet.pdf  TI 官方数据手册 (SLIS153D)
platformio.ini          PlatformIO 工程配置 (8MB Flash + qio_opi PSRAM 适配)
```
