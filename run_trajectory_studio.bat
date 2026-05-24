@echo off
cd /d "%~dp0"
python -m pip install -r tools\requirements_trajectory_studio.txt
python tools\trajectory_studio.py
pause
