@echo off
setlocal
where py >nul 2>&1
if not errorlevel 1 (
    py -3 "%~dp0tools\y42_pc_direct_web.py"
    set "EXITCODE=%ERRORLEVEL%"
) else (
    python --version >nul 2>&1
    if errorlevel 1 (
        echo No Python 3 launcher found. Install Python 3.10+ and pyserial first.
        echo     py -3 -m pip install pyserial
        pause
        exit /b 2
    )
    python "%~dp0tools\y42_pc_direct_web.py"
    set "EXITCODE=%ERRORLEVEL%"
)
if not "%EXITCODE%"=="0" pause
exit /b %EXITCODE%
