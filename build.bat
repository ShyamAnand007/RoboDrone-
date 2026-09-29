@echo off
setlocal
title Drone Flight Simulator - EXE Builder
echo ==============================================
echo   Drone Flight Simulator - one-click EXE build
echo ==============================================

where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python not found. Install Python 3.10+ from https://python.org
    echo         and tick "Add Python to PATH" during setup.
    goto fail
)

if not exist .venv (
    echo [1/4] Creating virtual environment...
    python -m venv .venv
    if errorlevel 1 goto fail
)

call .venv\Scripts\activate.bat

echo [2/4] Installing dependencies...
python -m pip install --upgrade pip
pip install -r requirements.txt
if errorlevel 1 goto fail

echo [3/4] Building executable (this takes a few minutes)...
pyinstaller --noconfirm --clean --onefile --noconsole ^
    --name DroneFlightSimulator ^
    --collect-all ursina ^
    --collect-all panda3d ^
    --collect-all panda3d_gltf ^
    --collect-all panda3d_simplepbr ^
    main.py
if errorlevel 1 goto fail

echo [4/4] Done!
echo.
echo   Your executable:  dist\DroneFlightSimulator.exe
echo.
pause
exit /b 0

:fail
echo.
echo [BUILD FAILED] See messages above.
pause
exit /b 1
