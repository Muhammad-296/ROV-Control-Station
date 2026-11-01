@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title ROV Control Station - Windows Setup and Launcher
set "APP=ROV_Control_Station_Integrated.py"
set "VENV=.rov_venv_windows"
set "PY=%VENV%\Scripts\python.exe"
set "DEPS=PySide6 pygame opencv-python numpy vtk pyserial"

if not exist "%APP%" (
  echo [ERROR] %APP% not found in %CD%
  echo Place this BAT beside the Python application.
  pause
  exit /b 1
)

if not exist "%PY%" (
  echo [1/3] Setting up a private Python environment...
  where py >nul 2>&1
  if not errorlevel 1 (
    py -3.12 -c "import sys" >nul 2>&1
    if not errorlevel 1 (py -3.12 -m venv "%VENV%") else (py -3 -m venv "%VENV%")
  ) else (
    where python >nul 2>&1
    if errorlevel 1 (echo [ERROR] Install Python 3.10-3.13 from python.org first.& pause & exit /b 1)
    python -m venv "%VENV%"
  )
  if errorlevel 1 (echo [ERROR] Could not create virtual environment.& pause & exit /b 1)
)

"%PY%" -c "import PySide6.QtSerialPort, pygame, cv2, numpy, vtk, vtkmodules.qt.QVTKRenderWindowInteractor" >nul 2>&1
if errorlevel 1 (
  echo [2/3] Installing libraries - internet required on first run...
  "%PY%" -m pip install --upgrade pip
  if errorlevel 1 goto :install_error
  "%PY%" -m pip install %DEPS%
  if errorlevel 1 goto :install_error
)

"%PY%" -c "import PySide6.QtSerialPort, pygame, cv2, numpy, vtk, vtkmodules.qt.QVTKRenderWindowInteractor"
if errorlevel 1 goto :install_error

echo [3/3] Starting the ROV Control Station...
"%PY%" "%APP%"
set "EXITCODE=%ERRORLEVEL%"
if not "%EXITCODE%"=="0" (echo [ERROR] The application exited with code %EXITCODE%.)
echo.
pause
exit /b %EXITCODE%

:install_error
echo [ERROR] Could not install or load required libraries.
echo Check Python version, architecture, internet connection, and installation messages.
pause
exit /b 1
