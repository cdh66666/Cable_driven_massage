# 轨迹图片生成器

`trajectory_studio.py` 是本地 Windows 图形工具，用来把图片转换成 ESP32 绳驱平台可以绘制的多段轨迹。

## 启动

在仓库根目录双击：

```powershell
.\run_trajectory_studio.bat
```

或者手动运行：

```powershell
python -m pip install -r tools\requirements_trajectory_studio.txt
python tools\trajectory_studio.py
```

## 依赖

```text
opencv-python
numpy
pyserial
```

## 使用流程

1. 点击 `打开图片`，选择 JPG、PNG、BMP、WebP 等常用位图。
2. 工具会自动处理，界面上方显示原图，下方显示转换后的轨迹预览。
3. 根据效果调整模式和参数，再点击 `处理成轨迹`。
4. 点击 `导出轨迹 TXT` 或 `导出 JSON` 保存轨迹。
5. 也可以选择板子的 USB 端口后点击 `USB上传轨迹`。
6. 电脑连接 ESP32 热点 `MassageDraw` 后，也可以点击 `WiFi上传到板子`。

## 输出格式

TXT 是一行一段轨迹，单位为 mm，坐标范围会映射到 `0..150`：

```text
x1,y1;x2,y2;x3,y3
x1,y1;x2,y2;x3,y3
```

JSON 是多段轨迹数组：

```json
[
  [{"x": 10.0, "y": 20.0}, {"x": 12.0, "y": 22.0}],
  [{"x": 50.0, "y": 60.0}, {"x": 52.0, "y": 63.0}]
]
```

当前默认不压缩轨迹点，导出的就是预览对应的原版轨迹。点数越多，上传和绘制越慢；需要更快绘制时可以调大 `点间距(mm)` 或降低 `最多点数`。

## 模式建议

- `惊喜复刻`：照片类图片的默认推荐模式，尽量像原图。
- `整图素描`：保留更多整体细节，适合壁纸、复杂照片。
- `照片主体线稿`：更偏主体轮廓，背景会少一些。
- `深色线稿`：适合已经是黑白线稿的图片。
- `边缘线稿`：适合轮廓清晰的图片。
- `黑色区域轮廓`：适合文字、图标、粗黑线条。

## 和板子联动

USB 上传前请关闭 PlatformIO Monitor 或其他占用串口的软件。默认端口通常是 `COM52 - USB-Enhanced-SERIAL CH343`。

上传成功后，可以在 USB 调试网页或手机热点网页里点击 `开始绘制`。

## 电脑直连 Y42（不使用 ESP32）

如果电脑使用的是 USB-RS485 转换器，可以直接运行仓库根目录的：

```powershell
py -3 -m pip install pyserial
.\run_y42_pc_direct.bat ports
.\run_y42_pc_direct.bat scan --port COM55
```

这个工具直接发送 Y42 二进制 RS485 帧，不使用 `usb_debug_server.py` 的 ESP32 文本协议。

也可以启动本地网页调试台：

```powershell
.\run_y42_pc_web.bat
```

浏览器打开 `http://127.0.0.1:8787/`。精简网页只保留自动连接、四电机状态、急停、四角自动校准和手绘轨迹，并提供 0–20 ID 扫描和单电机改 ID。ID 0 是广播地址，网页只把它作为范围起点展示，实际只扫描 ID 1–20。当前固定映射为 M1 左上、M2 右上、M3 右下、M4 左下，对应 ID 1/2/3/4。网页协议按 Y42 X 固件格式解析；每次打开网页时运动控制按钮默认锁定，需确认机构安全后解锁。校准默认主拉电流 1500mA、其余电机张力 300mA；四台校准后的角度/mm斜率使用四台拟合绝对值的平均值，正负号仍保留安装方向；四角切换时保持四台使能，只重新分配主拉/张力电流。网页还提供实际固件类型读取；四台已切换并保存为 X 类型（D5 69，00），四电机急停始终可用。

先扫描未知 ID：

```powershell
python tools\y42_pc_direct.py scan --port COM55
```

扫描到旧 ID 后，建议一次只接入一台当前使用该 ID 的电机，再设置唯一的新 ID：

```powershell
python tools\y42_pc_direct.py set-id --port COM55 --old 1 --new 2
```

改 ID 后给电机断电重启，再重新扫描。若多台电机共用同一个旧 ID，设置命令会同时改动它们，不能在四台都在线时盲目操作。

无绳长/角度校准时，只使用单电机低速测试：

```powershell
python tools\y42_pc_direct.py status --port COM55 --id 2
python tools\y42_pc_direct.py enable --port COM55 --id 2
python tools\y42_pc_direct.py position --port COM55 --id 2 --degrees 5 --direction 0
python tools\y42_pc_direct.py stop --port COM55 --id 2
python tools\y42_pc_direct.py disable --port COM55 --id 2
```

`position`、`torque`、`speed` 和 `raw` 会要求输入 `YES` 确认；自动化测试可追加 `--yes`。网页端的 XY/轨迹必须先完成四角机械几何校准。气泵、阀门和 GPIO47/48/45 仍不能由纯电脑 + RS485 代替，因为它们是 ESP32 GPIO 输出。固件类型切换是驱动器参数切换，不是刷写应用固件；切换后如指示灯未变化，请断电重启。
