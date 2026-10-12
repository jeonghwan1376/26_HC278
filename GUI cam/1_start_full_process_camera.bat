@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "PYTHON_CMD=python"
where py >nul 2>nul
if not errorlevel 1 set "PYTHON_CMD=py"

echo Starting full process camera...
echo Stream URL: http://127.0.0.1:5100/video
echo.

%PYTHON_CMD% full_process_camera_stream.py

if errorlevel 1 (
    echo.
    echo [ERROR] Camera stream failed.
    pause
    exit /b 1
)

endlocal
