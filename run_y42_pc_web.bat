@echo off
setlocal
set "Y42_PYTHON_ARGS="
if exist "%~dp0.venv-hardware\Scripts\python.exe" (
    set "Y42_PYTHON=%~dp0.venv-hardware\Scripts\python.exe"
    goto check_dependency
)
where py >nul 2>&1
if not errorlevel 1 (
    set "Y42_PYTHON=py"
    set "Y42_PYTHON_ARGS=-3"
    goto check_dependency
)
python --version >nul 2>&1
if errorlevel 1 (
    echo No Python 3 interpreter found. Install Python 3.10+ first.
    pause
    exit /b 2
)
set "Y42_PYTHON=python"

:check_dependency
"%Y42_PYTHON%" %Y42_PYTHON_ARGS% -c "import serial" >nul 2>&1
if errorlevel 1 (
    echo The selected Python could not import pyserial. The web server was not started.
    echo Install its hardware dependencies with:
    echo     "%Y42_PYTHON%" %Y42_PYTHON_ARGS% -m pip install -r "%~dp0tools\requirements_y42_hardware.txt"
    pause
    exit /b 2
)
"%Y42_PYTHON%" %Y42_PYTHON_ARGS% "%~dp0tools\y42_pc_direct_web.py"
set "EXITCODE=%ERRORLEVEL%"
if not "%EXITCODE%"=="0" pause
exit /b %EXITCODE%
