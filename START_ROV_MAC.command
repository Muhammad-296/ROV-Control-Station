#!/bin/bash

# ============================================================
# ROV Control Station - macOS Launcher
# Supports Apple Silicon (M1/M2/M3) and Intel Macs
# ============================================================

set -u

cd "$(dirname "$0")" || exit 1

APP="ROV_Control_Station_Integrated.py"
ENV_DIR=".rov_venv_mac"
PY="$ENV_DIR/bin/python"

# ------------------------------------------------------------
# Check application file
# ------------------------------------------------------------

if [ ! -f "$APP" ]; then
    echo "[ERROR] $APP not found in $(pwd)"
    echo "Place this launcher beside the Python application."
    read -r -p "Press Return to close..." _
    exit 1
fi

# ------------------------------------------------------------
# Step 1: Create Python environment
# ------------------------------------------------------------

if [ ! -x "$PY" ]; then

    echo "[1/3] Creating a dedicated Python environment..."

    if command -v python3.12 >/dev/null 2>&1; then
        PYTHON=python3.12
    elif command -v python3 >/dev/null 2>&1; then
        PYTHON=python3
    else
        echo "[ERROR] Python 3.12 is required."
        echo "Install Python 3.12 from python.org."
        read -r -p "Press Return to close..." _
        exit 1
    fi

    if ! "$PYTHON" -m venv "$ENV_DIR"; then
        echo "[ERROR] Unable to create Python environment."
        read -r -p "Press Return to close..." _
        exit 1
    fi

fi

# ------------------------------------------------------------
# Step 2: Check and install dependencies
# ------------------------------------------------------------

echo "[2/3] Checking required libraries..."

if ! "$PY" -c 'import PySide6.QtSerialPort, pygame, cv2, numpy, vtk, vtkmodules.qt.QVTKRenderWindowInteractor' >/dev/null 2>&1; then

    echo "[INFO] Installing missing or incomplete dependencies..."

    # Upgrade pip
    if ! "$PY" -m pip install --upgrade pip; then
        echo "[ERROR] Pip upgrade failed."
        read -r -p "Press Return to close..." _
        exit 1
    fi

    # Install required libraries
    if ! "$PY" -m pip install PySide6 pygame opencv-python numpy vtk pyserial; then
        echo "[ERROR] Library installation failed."
        read -r -p "Press Return to close..." _
        exit 1
    fi

fi

# ------------------------------------------------------------
# Verify dependencies
# ------------------------------------------------------------

echo "[INFO] Verifying installed libraries..."

if ! "$PY" -c '
import PySide6.QtSerialPort
import pygame
import cv2
import numpy
import vtk
import vtkmodules.qt.QVTKRenderWindowInteractor

print("All required libraries loaded successfully.")
'; then

    echo "[ERROR] An essential library could not be loaded."
    read -r -p "Press Return to close..." _
    exit 1

fi

# ------------------------------------------------------------
# Step 3: Launch application
# ------------------------------------------------------------

echo ""
echo "============================================================"
echo "       ROV CONTROL STATION"
echo "============================================================"
echo "[3/3] Launching ROV Control Station..."
echo ""

"$PY" "$APP"

CODE=$?

echo ""
echo "============================================================"
echo "ROV Control Station exited with code $CODE."
echo "============================================================"

read -r -p "Press Return to close..." _

exit "$CODE"
