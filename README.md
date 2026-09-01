# 绳驱按摩 / 绘画平台

这是一个四电机绳驱二维平台。仓库同时包含两种控制方式：ESP32-S3 主控的手机热点网页，以及电脑通过 USB-RS485 直连 Y42 电机的本地调试网页。电脑直连模式不需要 ESP32 控制板，适合电机 ID、固件模式、几何校准和轨迹控制调试。

## 当前能力

- 150 x 150 mm 绘图区域，支持手绘轨迹、多段轨迹、TXT/JSON 轨迹导入和循环/单次绘制。
- ESP32 开机创建无密码热点，手机连接后可进入网页控制。
- 电脑可通过 USB-RS485 直接打开本地调试网页，不需要手机连接热点或 ESP32。
- Windows 图片转轨迹工具支持常见图片格式，本地快速生成轨迹预览、TXT、JSON、PNG，并可通过 WiFi 或 USB 上传到板子。
- 电机校准参数、绘制速度、轨迹间移动速度、电机运动电流等参数保存到 ESP32 NVS，重启后继续使用。
- 气压部分保留手动控制：充气、慢放气、快放气、停止，并实时显示传感器电压/换算压力。

## 电脑直连 Y42（USB-RS485）

### 适用范围

电脑直连工具直接发送 Y42 二进制 RS485 指令，默认按 X 固件协议工作。它不复用 ESP32 的文本协议，也不会控制 ESP32 GPIO，因此气泵、阀门和 GPIO 输出仍只能由 ESP32 控制。

固定电机映射如下：

| 电机 | 位置 | ID |
| --- | --- | --- |
| M1 | 左上 | 1 |
| M2 | 右上 | 2 |
| M3 | 右下 | 3 |
| M4 | 左下 | 4 |

### 硬件连接

1. USB-RS485 转换器 A/B 接到四台 Y42 的 RS485 总线，电脑端安装好转换器驱动。
2. 总线上每台电机必须有唯一 ID；改 ID 时建议只接目标电机，避免多台一起改名。
3. 电机上电后应处于 X 固件模式（绿灯闪烁）。固件切换是驱动器参数切换，不是刷写应用固件；切换后按页面提示断电重启。
4. 首次运动前确认绳路、小球、滑轮和机械限位安全，并准备硬件急停。网页的空格键也会发送四台急停并失能。

### 安装依赖

```powershell
python -m pip install pyserial
```

### 命令行工具

列出电脑上的串口：

```powershell
.\run_y42_pc_direct.bat ports
```

扫描未知 ID（默认 1–255，只读，不发送运动指令）：

```powershell
.\run_y42_pc_direct.bat scan --port COM7 --start 1 --end 20
```

读取单台状态：

```powershell
.\run_y42_pc_direct.bat status --port COM7 --id 1
```

修改 ID 前必须保证旧 ID 只对应目标电机。改完后断电重启，再重新扫描：

```powershell
.\run_y42_pc_direct.bat set-id --port COM7 --old 1 --new 5
```

单电机低速位置测试（X 固件相对位置模式）：

```powershell
.\run_y42_pc_direct.bat position --port COM7 --id 1 --degrees 5 --direction 0
.\run_y42_pc_direct.bat stop --port COM7 --id 1
.\run_y42_pc_direct.bat disable --port COM7 --id 1
```

运动、力矩和原始帧命令会要求确认；自动化测试可显式追加 `--yes`。不要在不确认机构安全时使用 `--yes`。

### 本地网页调试台

启动：

```powershell
.\run_y42_pc_web.bat
```

浏览器打开：<http://127.0.0.1:8787/>

网页功能按安全流程排列：

- 顶部实时显示 M1–M4 在线状态、电流和位置。
- 一键自动准备：扫描 ID 1–4、读取/切换 X 固件、锁定电机本体按键、最后四台失能。
- USB-RS485 串口可在页面顶部下拉选择，也可自动选择唯一可用串口。
- 单台方向确认：点击对应电机，只按当前候选方向转 45°，停止后保存方向。
- 四角几何校准：默认上下中心距 150 mm、小球直径 40 mm、主拉电流 1500 mA、其余电机张力 300 mA。
- 校准后四台使用同一个公共 `角度/mm` 斜率绝对值；正负号保留安装方向，编码器截距仍按电机独立保存。斜率微调只有在手动填写非零值时才会有意改变某一台的有效斜率。
- 手绘轨迹：蓝色表示手绘输入，绿色表示当前发送的目标轨迹；轨迹起点和终点均使用平滑接管与有限超时。
- 控制目标使用四电机 0xAA 合帧，目标时钟为 100 Hz；实际发送频率会受 USB-RS485 转换器、串口驱动和总线回包影响，页面和过程日志会记录实测值。
- 空格键急停：停止自动流程并发送四台急停/失能；硬件急停仍是最高优先级。

### 几何参数和过程数据

- `tools/y42_pc_calibration.json`：本机四角几何校准结果和公共斜率，供网页启动后读取。
- `tools/y42_pc_process_log.jsonl`：运行时产生的轨迹、位置、电流、频率和错误过程数据，默认不提交到 GitHub。
- 重新做四角校准会覆盖校准结果；校准前不需要重复做单台编码器校准。

### 故障排查

| 现象 | 检查方式 |
| --- | --- |
| 找不到串口 | 重新插拔 USB-RS485，关闭串口监视器，点击网页“刷新串口” |
| 电机 ID 不全 | 单台接入扫描，确认每台 ID 唯一，再断电重启 |
| 不是 X 固件 | 读取固件状态，按页面执行切换并断电重启，确认绿灯闪烁 |
| 轨迹起点差一点 | 新逻辑会持续重发终点；误差不超过 18° 且四台停止后平滑接管，较大误差会超时失能 |
| 运动中离线 | 先按空格或硬件急停，再检查 A/B、供电、终端电阻和转换器驱动；不要让电机在未确认状态下持续使能 |

电脑直连模式不负责气泵、阀门或 ESP32 GPIO 控制；需要这些功能时请使用 ESP32 主控网页。

## 硬件连接

### 主控和电机

- 开发板：ESP32-S3 N16R8。
- TTL 转 485 模块：
  - ESP32 GPIO17 -> 485 TX/RX 模块输入侧 TX
  - ESP32 GPIO16 -> 485 TX/RX 模块输入侧 RX
  - GND 共地
- 电机摆放方向：
  - M1：左上角 `(0, 150)`
  - M2：右上角 `(150, 150)`
  - M3：右下角 `(150, 0)`
  - M4：左下角 `(0, 0)`

ESP32 固件示例默认校准系数为：

```text
M1: -3.761 deg/mm
M2:  3.775 deg/mm
M3: -3.742 deg/mm
M4:  3.796 deg/mm
```

电脑直连网页读取的当前本机校准文件使用公共斜率：

```text
M1: -4.216363 deg/mm
M2:  4.216363 deg/mm
M3: -4.216363 deg/mm
M4:  4.216363 deg/mm
```

### 气压控制

当前代码按下面接线工作：

```text
GPIO11: 充气泵 PWM
GPIO9 : 放气阀
GPIO10: 快速放气辅助阀
GPIO4 : 模拟气压传感器输出
```

手动控制逻辑：

- 充气：GPIO11 输出 PWM。
- 慢放气：GPIO9 打开。
- 快放气：GPIO9 + GPIO10 同时打开。
- 保压/停止：三个输出全部关闭。

当前传感器按 3.3 V 供电、模拟输出计算，适配已测试的 CFSensor/XGZP6847A 类 -5~5 kPa 气压模块。网页端先直接显示电压值，方便继续标定。

## 环境准备

安装 Python 3.10+，然后安装 PlatformIO：

```powershell
python -m pip install -U platformio
```

烧录固件：

```powershell
python -m platformio run --target upload --upload-port COM52
```

串口监视：

```powershell
python -m platformio device monitor --port COM52 --baud 460800
```

如果你的板子不是 COM52，把命令里的端口改成设备管理器里显示的 `USB-Enhanced-SERIAL CH343` 对应端口。

## 手机热点控制

烧录后 ESP32 会创建热点：

```text
SSID: MassageDraw
密码: 无
地址: http://192.168.4.1/
```

手机连接热点后通常会自动跳转。如果没有跳转，手动打开 `http://192.168.4.1/`。

网页端可以：

- 在白色画布里手绘轨迹。
- 导入/上传轨迹。
- 开始、停止、清空画布、循环开关。
- 调节绘制速度、轨迹间移动速度、电机运动电流。
- 一键校准参数并显示校准系数。
- 手动充气、慢放气、快放气、停止气压输出。

## USB 调试网页

电脑插上开发板后运行：

```powershell
.\run_usb_debug_web.bat
```

浏览器会打开：

```text
http://127.0.0.1:8765/
```

这个页面通过 Web Serial/本地 Python 服务和 ESP32 通信，适合调试时不用手机热点。使用前请关闭 PlatformIO Monitor、Arduino 串口监视器等占用 COM 口的软件。

USB 调试网页包含：

- 串口自动搜索和端口选择。
- 当前气压电压、换算压力、控制状态显示。
- 手动充气、慢放气、快放气、停止，以及气泵 PWM 调节。
- TXT/JSON 轨迹上传。
- 开始绘制、停止、清空画板。
- 校准参数、读取状态。
- 轨迹速度滑块。
- 电机运动电流滑块。

## 图片转轨迹工具

运行：

```powershell
.\run_trajectory_studio.bat
```

首次运行会自动安装：

```text
opencv-python
numpy
pyserial
```

工具功能：

- 打开图片后自动转换，原图在上、轨迹预览在下，方便对比。
- 支持 JPG、PNG、BMP、WebP 等 OpenCV 可读取的常用图片格式。
- 支持多种模式，例如惊喜复刻、整图素描、照片主体线稿、深色线稿、边缘线稿等。
- 可导出轨迹 TXT、JSON 和预览 PNG。
- 可直接通过 WiFi 上传到 ESP32 热点网页。
- 可直接通过 USB 上传到开发板。

轨迹会被缩放到 150 x 150 mm 绘图区域内，输出点坐标单位为 mm。

## 轨迹文件格式

TXT 格式是一行一段轨迹：

```text
x1,y1;x2,y2;x3,y3
x1,y1;x2,y2;x3,y3
```

JSON 格式是多段轨迹数组：

```json
[
  [{"x": 10.0, "y": 20.0}, {"x": 11.0, "y": 20.5}],
  [{"x": 30.0, "y": 40.0}, {"x": 31.0, "y": 41.0}]
]
```

## 常用串口命令

```text
usbstatus                读取完整状态
current 1000             设置并保存绘图运动电流，单位 mA
speed 40                 设置绘制速度百分比
travelspeed 40           设置轨迹间移动速度百分比
tension 2.0              设置绳子预拉紧补偿，单位 mm
coef                     查看当前电机系数
coef set M1 M2 M3 M4     手动写入并保存四个电机系数
calibrate                开始自动校准
webstart                 开始绘制
webstop                  停止绘制
loopon / loopoff         开启/关闭循环
clearpath                清空板端轨迹
goto X Y                 移动到指定坐标
airbits 4 0 180          持续充气，PWM=180
airbits 1 0 0            持续慢放气
airbits 3 0 0            持续快放气
airbits 0 0 0            停止气压输出
airstatus                读取气压状态
```

## 打包 Windows EXE

GitHub 仓库建议只放源码，不提交 exe、`dist/`、`build/`、`.pio/`。

如果要给别人一键运行的 Windows 程序，执行：

```powershell
.\build_trajectory_studio_exe.bat
```

生成结果：

```text
dist\TrajectoryStudio.exe
```

把这个 exe 上传到 GitHub Releases 的附件里，不要提交进 Git 历史。

## GitHub 更新前检查

```powershell
git status --short
python -m platformio run
```

确认没有多余的未追踪文件，再提交：

```powershell
git add README.md .gitignore platformio.ini src tools run_*.bat build_trajectory_studio_exe.bat
git commit -m "Add USB trajectory upload and air/manual control tools"
git push
```
