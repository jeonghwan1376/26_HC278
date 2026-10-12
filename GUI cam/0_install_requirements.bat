@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "PYTHON_CMD=python"
where py >nul 2>nul
if not errorlevel 1 set "PYTHON_CMD=py"

echo Installing required packages...
%PYTHON_CMD% -m pip install -r requirements.txt

if errorlevel 1 (
    echo.
    echo [ERROR] Package installation failed.
    pause
    exit /b 1
)

echo.
echo Installation completed.
pause
endlocal
