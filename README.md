# ESP32-S3 N16R8 Step Motor Controller

VSCode + PlatformIO 工程，默认开发板串口为 `COM52`，监视器波特率为 `115200`。

## 常用命令

```powershell
python -m platformio run
python -m platformio run --target upload
python -m platformio device monitor
```

调试指令见串口启动输出。新增坐标指令：

```text
home
goto 75 75
```
