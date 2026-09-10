@echo off
rem 重新打包 Y42 调试台单文件 EXE（产物：上一级目录的 调试台.exe）
setlocal
cd /d "%~dp0"

where python >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 3.10+。
    pause
    exit /b 2
)

python -c "import PyInstaller, serial" >nul 2>&1
if errorlevel 1 (
    echo 正在安装打包依赖 pyinstaller / pyserial ...
    python -m pip install pyinstaller "pyserial==3.5"
)

python build_exe.py
if errorlevel 1 (
    echo.
    echo [失败] 打包未成功，请查看上方输出。
    pause
    exit /b 1
)

echo.
echo [完成] 可执行文件已生成。
pause
