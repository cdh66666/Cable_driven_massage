# Cable Driven Massage / Drawing Controller

ESP32-S3 N16R8 + 四电机绳驱平台工程。当前固件可以完成 150 x 150 mm 平面坐标控制、四点自动校准、手机热点绘图网页、电脑 USB 网页调试、图片转轨迹、本地轨迹上传和内置图案绘制。

当前默认开发环境是 Windows + VS Code + PlatformIO，默认串口为 `COM52`。

## 功能概览

- 四个电机分别位于绘制区域四角，工作区域为 `150 x 150 mm`。
- 支持串口命令控制电机、读位置、清零、使能、失能和编码器校准。
- 支持坐标命令：`home` 四点校准、`goto X Y` 移动到指定 mm 坐标。
- 支持手机连接 ESP32 热点 `MassageDraw`，自动打开网页绘制轨迹。
- 支持多段轨迹、循环/单次播放、绘制速度、轨迹间移动速度、清除画布。
- 支持校准参数网页端操作，校准系数和速度/扭矩参数会保存到 NVS，重启后仍可使用。
- 支持电脑 USB 本地网页调试，不需要手机连接热点。
- 支持本地 Windows 图片转轨迹工具，可导出 TXT/JSON/预览 PNG，也可直接 USB 上传到板子。
- 支持内置图案按钮 `询问机仙`，加载固件内置轨迹并绘制一次或循环绘制。

## 仓库结构

```text
.
├─ src/
│  ├─ main.cpp              # ESP32-S3 固件主程序
│  └─ rice_preset.h         # 内置图案轨迹数据
├─ tools/
│  ├─ trajectory_studio.py  # Windows 图片转轨迹工具
│  ├─ usb_debug_server.py   # 本地 USB 调试网页服务器
│  ├─ usb_debug_web.html    # 浏览器 Web Serial 调试页面
│  └─ requirements_trajectory_studio.txt
├─ platformio.ini           # PlatformIO 配置
├─ run_trajectory_studio.bat
├─ run_usb_debug_web.bat
└─ build_trajectory_studio_exe.bat
```

`build/`、`dist/`、`.pio/` 和 `*.spec` 是本地生成物，已经被 `.gitignore` 排除，不要提交到 GitHub。

## 硬件和端口

默认硬件：

- 开发板：ESP32-S3 N16R8，CH343 USB 串口。
- 默认串口：`COM52`。
- 调试串口波特率：`460800`。
- 电机总线：`HardwareSerial uart2(2)`。
- 电机总线引脚：
  - `UART2_TX = 17`
  - `UART2_RX = 16`
- 绘制区域：`150 x 150 mm`。
- 四角定义：
  - M1：左下 `(0, 0)`
  - M2：左上 `(0, 150)`
  - M3：右上 `(150, 150)`
  - M4：右下 `(150, 0)`

如果你的串口不是 `COM52`，修改 [platformio.ini](platformio.ini)：

```ini
upload_port = COM52
monitor_port = COM52
monitor_speed = 460800
```

## 安装环境

需要：

- Python 3.10 或更高版本
- VS Code
- PlatformIO
- Edge 或 Chrome 浏览器，USB 本地网页需要 Web Serial

安装 PlatformIO 后，在仓库根目录运行：

```powershell
python -m platformio run
```

能正常编译即环境 OK。

## 编译和烧录固件

编译：

```powershell
python -m platformio run
```

烧录：

```powershell
python -m platformio run --target upload
```

打开串口监视器：

```powershell
python -m platformio device monitor --port COM52 --baud 460800
```

正常启动后会看到类似输出：

```text
===== 全功能电机调试器 =====
Loaded calibration from flash.
Web path buffer ready: 200000 points, 1200000 bytes
Open WiFi drawing portal ready.
SSID: MassageDraw  Password: none  URL: http://192.168.4.1/
USBREADY baud=460800 strokes=0 points=0
```

注意：ESP32-S3 ROM 启动日志是 `115200`，应用调试口是 `460800`。刚打开串口时看到少量乱码属于正常现象，应用启动后应出现中文菜单和 `USBREADY`。

## 串口命令

固件启动后会打印帮助菜单。常用命令：

```text
M                  切换单电机/全体电机模式
p                  读位置
e                  使能
d                  失能
c                  位置清零
ca                 编码器校准
t500               设置力矩
-90                转到指定角度

1p                 读 1 号电机位置
1e                 使能 1 号电机
1-90               1 号电机转到 -90 度
1t-500             1 号电机设置反向力矩

home               四点自动校准
goto 75 75         移动到坐标，单位 mm
coef               打印当前校准系数
circles            绘制圆形测试
square             绘制方形测试
eight              绘制 8 字测试
spiral             绘制螺旋测试
shapes             运行图形测试组合
ricepreset         加载内置图案，不立即移动
wifi               打印热点信息
reboot             重启板子

speed 50           设置并保存 USB/网页绘制速度，范围 1..100
travelspeed 15     设置并保存轨迹段之间的空移速度
caltorque 80 900   设置并保存校准保持/拉动扭矩，单位 mA
clearpath          清空板载已上传轨迹
calibrate          从 USB 串口触发四点校准
setid 1 4          把当前 1 号电机改成 4 号 ID 并保存
allpos             读取全部电机位置
allenable          全部电机使能
alldisable         全部电机失能
allzero            全部电机位置清零

webstart           开始绘制当前网页/USB 轨迹
webstop            停止绘制
loopon             循环绘制
loopoff            单次绘制
usbstatus          查看 USB/网页轨迹状态
```

## 手机热点网页

烧录后 ESP32 会开启开放热点：

```text
SSID: MassageDraw
Password: none
URL: http://192.168.4.1/
```

手机连接 `MassageDraw` 后，系统通常会自动弹出网页；如果没有自动弹出，手动访问：

```text
http://192.168.4.1/
```

网页功能：

- 白色画布中绘制轨迹，黑色线为轨迹。
- 支持多段轨迹：松手后再画下一段即可。
- `开始`：开始绘制当前轨迹。
- `停止`：停止绘制并失能。
- `清除画布`：清除当前网页轨迹。
- `循环: 开/关`：切换循环绘制或单次绘制。
- `校准参数`：执行四点校准，完成后显示并保存系数。
- `导入轨迹`：导入 TXT/JSON 轨迹文件。
- `询问机仙`：加载固件内置图案轨迹。
- `绘制` 滑条：控制绘制轨迹速度。
- `移动` 滑条：控制轨迹段之间、初始移动到起点的速度。
- `保持` 滑条：校准时非当前电机保持力矩。
- `校准` 滑条：校准时当前电机拉向角点的力矩。

速度、校准扭矩和校准系数都会自动保存到 NVS，重启后仍然生效。

## 电脑 USB 网页调试

这个方式不需要手机连接热点。电脑 USB 插着开发板即可。

启动：

```powershell
.\run_usb_debug_web.bat
```

脚本会启动本地服务并打开：

```text
http://127.0.0.1:8765/
```

使用步骤：

1. 用 Edge 或 Chrome 打开本地网页。
2. 点击 `连接USB串口`。
3. 选择 `COM52 - USB-Enhanced-SERIAL CH343`。
4. 状态区应出现 `USBREADY` 或 `USBSTATUS`。
5. 可以直接在中间白色画布用鼠标画多段轨迹，也可以选择 TXT/JSON 轨迹文件。
6. 点击 `上传当前画布` 或 `上传文件轨迹`。
7. 上传成功后点击 `开始`、`停止`、`循环开/关`。

USB 调试页还提供：

- 路径参数：绘制速度、空移速度、校准保持扭矩、当前电机拉动扭矩。
- 整机调试：全部使能、全部失能、读取全部位置、全部位置清零、校准系数、读取系数、坐标移动、载入内置图案。
- 单电机调试：按 ID 读取位置、使能、失能、清零、锁键、解锁、编码器校准、发送角度、发送扭矩。
- 设置电机 ID：输入当前 ID 和新 ID 后点击 `设置`。建议一次只接入或只修改一个目标电机，设置后断电重启电机，再用新 ID 读取位置确认。
- 原始命令：直接发送固件支持的串口命令。

本地 USB 网页使用浏览器 Web Serial 直接访问串口。调试结束后请点击 `断开`，否则串口会被浏览器占用，PlatformIO 不能烧录。

## 图片转轨迹工具

启动：

```powershell
.\run_trajectory_studio.bat
```

首次运行会安装：

```text
opencv-python
numpy
pyserial
```

工具界面功能：

- `打开图片`：选择 PNG/JPG/JPEG/BMP/TIFF/WebP/GIF 首帧等常见位图。
- `处理成轨迹`：把图片转换为硬件可绘制的多段轨迹。
- `导出轨迹 TXT`：导出网页/USB 可导入的轨迹文本。
- `导出 JSON`：导出多段轨迹 JSON。
- `导出预览 PNG`：导出白底黑线预览图。
- `WiFi上传到板子`：电脑连接 `MassageDraw` 热点后上传到板子。
- `打开USB调试网页`：打开 `http://127.0.0.1:8765/`。
- `USB上传轨迹`：通过 `COM52` 直接上传轨迹到板子。

处理模式：

```text
惊喜复刻
照片点描复刻
整图素描
照片主体线稿
自动
深色线稿
黑色区域轮廓
边缘线稿
物体外轮廓
中心线(慢)
```

推荐用法：

- 照片、壁纸、复杂图：优先 `惊喜复刻` 或 `整图素描`。
- 黑白线稿：优先 `深色线稿`。
- 只要主体：用 `照片主体线稿`。
- Logo、粗黑字：用 `黑色区域轮廓`。
- 简单物体外形：用 `物体外轮廓`。

导出的轨迹会自动缩放到 `150 x 150 mm` 区域，并留出边距。

## 轨迹文件格式

TXT 格式：

```text
x,y;x,y;x,y
x,y;x,y;x,y
```

说明：

- 一行是一段轨迹。
- 点单位是 mm。
- 坐标范围是 `0..150`。
- 多行表示多段轨迹，段与段之间硬件会用 `移动` 速度连接。

JSON 格式：

```json
[
  [{"x": 20, "y": 20}, {"x": 40, "y": 40}],
  [{"x": 60, "y": 20}, {"x": 80, "y": 40}]
]
```

USB 二进制上传协议：

```text
USBBIN <point_count>\n
```

板子回复：

```text
USBBIN READY count=<point_count>
```

随后发送每个点 5 字节：

```text
uint16 little-endian x100
uint16 little-endian y100
uint8  draw
```

其中 `x100/y100` 表示 `mm * 100`，`draw=0` 表示新轨迹段起点，`draw=1` 表示继续绘制。成功后板子回复：

```text
USBBIN OK strokes=<n> raw=<n> points=<n>
```

## 校准流程

推荐用网页端校准：

1. 连接手机热点网页，或使用 USB 本地网页。
2. 设置 `保持` 扭矩，默认适合让其他电机轻微拉紧。
3. 设置 `校准` 扭矩，默认适合把当前电机拉到自己的角点。
4. 点击 `校准参数`。
5. 等待四个角点依次完成。
6. 网页会显示 M1-M4 系数，固件会自动保存。

串口也可以输入：

```text
home
```

校准结果会被检查：

- 每个电机系数绝对值应在约 `4.00..4.70 deg/mm`。
- 四个电机系数绝对值最大差值不应超过 `0.15 deg/mm`。
- 如果校准结果不合理，固件会保留旧参数，不会覆盖保存。

默认系数：

```text
M1: -4.3
M2:  4.3
M3: -4.3
M4:  4.3
```

## GitHub 源码提交策略

推荐：GitHub 仓库只放源码，不放打包 exe。

已经被 `.gitignore` 排除的内容：

```text
.pio/
build/
dist/
*.spec
*.bin
*.elf
*.map
__pycache__/
*.pyc
```

提交源码：

```powershell
git status --short
git add README.md .gitignore platformio.ini src tools run_trajectory_studio.bat run_usb_debug_web.bat build_trajectory_studio_exe.bat
git status --short
git commit -m "Add USB trajectory upload and trajectory studio"
git push
```

确认不要提交这些目录：

```powershell
git status --ignored --short
```

如果误加了大文件但还没提交，用：

```powershell
git rm --cached -r build dist .pio
git rm --cached TrajectoryStudio.spec
```

## 发布 Windows EXE

如果要给别人一键运行，不要把 exe 提交进 git 历史。正确做法是放到 GitHub Releases。

本地打包：

```powershell
.\build_trajectory_studio_exe.bat
```

输出文件：

```text
dist\TrajectoryStudio.exe
```

发布方式 A：GitHub 网页

1. 打开仓库页面。
2. 右侧点击 `Releases`。
3. 点击 `Draft a new release`。
4. 创建 tag，例如 `v1.0.0`。
5. 标题写本次版本说明。
6. 上传 `dist\TrajectoryStudio.exe` 到附件。
7. 点击发布。

发布方式 B：GitHub CLI

```powershell
git tag v1.0.0
git push origin v1.0.0
gh release create v1.0.0 dist\TrajectoryStudio.exe --title "v1.0.0" --notes "USB trajectory upload version"
```

## 常见问题

### USB 网页连接后没有反应

先确认状态区是否出现：

```text
USBREADY baud=460800
```

如果没有：

- 确认选择的是 `COM52 - USB-Enhanced-SERIAL CH343`。
- 不要选到其他蓝牙串口。
- 刷新页面后重新连接。
- 按开发板复位键。
- 用串口监视器确认 `460800` 下能看到固件菜单。

### 烧录失败，提示 COM52 不存在或不可用

通常是串口被网页、串口监视器或 Python 程序占用。

处理：

1. USB 本地网页点击 `断开`。
2. 关闭串口监视器。
3. 关闭占用串口的 Python 程序。
4. 再执行：

```powershell
python -m platformio run --target upload
```

### 网页上传轨迹超时

确认板子已经回复：

```text
USBBIN READY count=...
```

如果没有：

- 先点 `读取状态`。
- 确认能看到 `USBSTATUS`。
- 重新连接 USB 串口。

### 图片转轨迹工具很大

大的是打包 exe 和 OpenCV 依赖，不是源码。

推荐：

- GitHub 仓库只提交源码。
- `dist\TrajectoryStudio.exe` 放 GitHub Release。
- 普通用户下载 Release 附件。
- 开发者 clone 仓库后运行 `run_trajectory_studio.bat`。

### 手机不自动弹出热点网页

手动打开：

```text
http://192.168.4.1/
```

热点无密码：

```text
MassageDraw
```

### 板子一直重启

打开串口监视器看错误信息。如果出现 UART buffer 相关错误，确认 `src/main.cpp` 中调试串口缓冲为：

```cpp
Serial0.setRxBufferSize(8192);
```

不要改成过大的值。
