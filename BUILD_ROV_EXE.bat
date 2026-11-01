@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title ROV Control Station - EXE Builder

set "APP=ROV_Control_Station_Integrated"
set "SCRIPT=%APP%.py"
set "VENV=.rov_build_env"
set "PY=%CD%\%VENV%\Scripts\python.exe"
set "LOG=%CD%\ROV_EXE_Build.log"

echo ============================================================
echo        ROV CONTROL STATION - ONE FILE EXE BUILD
echo ============================================================
echo Working folder: %CD%
echo.
if not exist "%SCRIPT%" (
    echo ERROR: Cannot find "%SCRIPT%".
    echo Put this BAT file next to your Python file and run again.
    pause
    exit /b 1
)

if not exist "%PY%" (
    echo [1/4] Creating a clean Python virtual environment...
    py -3.12 -m venv "%VENV%" 2>nul
    if errorlevel 1 (
        py -3 -m venv "%VENV%" 2>nul
        if errorlevel 1 (
            echo ERROR: Python 3 is missing or venv could not be created.
            echo Install Python 3.12 x64 and enable the Python Launcher.
            pause
            exit /b 1
        )
    )
)
if not exist "%PY%" (
    echo ERROR: Python environment not found.
    pause
    exit /b 1
)

echo [2/4] Installing required packages...
"%PY%" -m pip install --upgrade pip setuptools wheel
if errorlevel 1 goto :failed
"%PY%" -m pip install --upgrade pyinstaller PySide6 pygame opencv-python-headless numpy vtk
if errorlevel 1 goto :failed

echo [3/4] Verifying packages...
"%PY%" -c "import PySide6, PySide6.QtSerialPort, pygame, cv2, numpy, vtk, vtkmodules; print('All core imports OK')"
if errorlevel 1 goto :failed

echo [4/4] Building single EXE - detailed output in ROV_EXE_Build.log...
set "FONT_OPTION="
if exist "fonts\" set "FONT_OPTION=--add-data=fonts;fonts"

"%PY%" -m PyInstaller --noconfirm --clean --onefile --windowed ^
    --name "%APP%" ^
    --collect-all vtkmodules ^
    --collect-submodules vtk ^
    --collect-all cv2 ^
    --collect-all pygame ^
    --hidden-import PySide6.QtSerialPort ^
    --hidden-import vtkmodules.qt.QVTKRenderWindowInteractor ^
    --exclude-module PyQt6 ^
    --exclude-module PyQt5 ^
    --exclude-module PySide2 ^
    %FONT_OPTION% ^
    "%SCRIPT%" > "%LOG%" 2>&1
if errorlevel 1 (
    echo.
    echo BUILD FAILED. Last 50 log lines:
    powershell -NoProfile -Command "Get-Content -LiteralPath '%LOG%' -Tail 50"
    echo Full log: "%LOG%"
    pause
    exit /b 1
)
if not exist "dist\%APP%.exe" (
    echo ERROR: PyInstaller finished but no EXE was found. See log: "%LOG%"
    pause
    exit /b 1
)
echo.
echo ============================================================
echo SUCCESS!
echo EXE: "%CD%\dist\%APP%.exe"
echo Build log: "%LOG%"
echo ============================================================
pause
exit /b 0

:failed
echo.
echo Build stopped. Check the errors above.
pause
exit /b 1
