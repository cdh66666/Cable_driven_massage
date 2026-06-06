# Cable Driven Massage / Drawing Platform

这是一个基于 ESP32-S3 N16R8 的四电机绳驱二维平台。当前版本支持手机热点控制、电脑 USB 调试控制、图片转轨迹、轨迹上传绘制、电机校准、运动电流调节，以及手动气压充放气控制。

## 当前能力

- 150 x 150 mm 绘图区域，支持手绘轨迹、多段轨迹、TXT/JSON 轨迹导入和循环/单次绘制。
- ESP32 开机创建无密码热点，手机连接后可进入网页控制。
- 电脑可通过 USB 串口打开本地调试网页，不需要手机连接热点。
- Windows 图片转轨迹工具支持常见图片格式，本地快速生成轨迹预览、TXT、JSON、PNG，并可通过 WiFi 或 USB 上传到板子。
- 电机校准参数、绘制速度、轨迹间移动速度、电机运动电流等参数保存到 ESP32 NVS，重启后继续使用。
- 气压部分保留手动控制：充气、慢放气、快放气、停止，并实时显示传感器电压/换算压力。

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

当前默认校准系数为：

```text
M1: -3.761 deg/mm
M2:  3.775 deg/mm
M3: -3.742 deg/mm
M4:  3.796 deg/mm
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
