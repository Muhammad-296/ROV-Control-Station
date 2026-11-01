$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$python = '.\.venv\Scripts\python.exe'
if (-not (Test-Path $python)) { throw 'Run INSTALL_WINDOWS.ps1 first to create a clean Python 3.12 environment.' }
& $python -m PyInstaller --clean --noconfirm --onefile --windowed `
    --name ROV_Control_Station_v5 `
    --exclude-module PyQt6 --exclude-module PyQt5 --exclude-module PySide2 `
    --collect-submodules vtkmodules `
    'ROV_Control_Station_Integrated.py'
if ($LASTEXITCODE -ne 0) { throw 'PyInstaller failed. Check missing libraries and Qt hooks in the build output.' }
Write-Host 'Built dist\ROV_Control_Station_v5.exe (test on the target hardware before operation).' -ForegroundColor Green
