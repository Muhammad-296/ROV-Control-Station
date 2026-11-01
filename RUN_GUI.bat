@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" "ROV_Control_Station_Integrated.py"
) else (
  py -3.12 "ROV_Control_Station_Integrated.py"
)
if errorlevel 1 pause
