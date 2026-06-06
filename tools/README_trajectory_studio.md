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
