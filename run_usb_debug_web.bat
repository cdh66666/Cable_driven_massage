@echo off
cd /d "%~dp0"
python -m pip install pyserial
python tools\usb_debug_server.py
pause
