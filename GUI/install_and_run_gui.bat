@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "VENV_DIR=C:\isaac_gui_env_v10"
set "PYTHON_EXE=%VENV_DIR%\Scripts\python.exe"

where py >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python launcher was not found.
    pause
    exit /b 1
)

if not exist "%PYTHON_EXE%" (
    echo [1/4] Creating GUI environment...
    py -m venv "%VENV_DIR%"
    if errorlevel 1 goto ERROR_EXIT
)

echo [2/4] Updating pip...
"%PYTHON_EXE%" -m pip install --upgrade pip setuptools wheel
if errorlevel 1 goto ERROR_EXIT

echo [3/4] Installing GUI packages...
"%PYTHON_EXE%" -m pip install --no-cache-dir --only-binary=:all: -r "%~dp0requirements_gui.txt"
if errorlevel 1 goto ERROR_EXIT

echo [4/4] Starting large Isaac GUI...
"%PYTHON_EXE%" "%~dp0isaac_integrated_gui_v10.py"
if errorlevel 1 goto ERROR_EXIT

goto END

:ERROR_EXIT
echo.
echo [ERROR] Installation or execution failed.
echo Delete C:\isaac_gui_env_v10 and run this BAT again.
pause
exit /b 1

:END
endlocal
