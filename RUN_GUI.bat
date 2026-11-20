@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" "control_station.py"
) else (
  py -3.12 "control_station.py"
)
if errorlevel 1 pause
