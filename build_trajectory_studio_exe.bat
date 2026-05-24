@echo off
cd /d "%~dp0"
python -m pip install -r tools\requirements_trajectory_studio.txt pyinstaller
python -m PyInstaller --noconfirm --onefile --windowed --name TrajectoryStudio --add-data "tools\usb_debug_web.html;tools" tools\trajectory_studio.py
echo.
echo Done. EXE: dist\TrajectoryStudio.exe
pause
