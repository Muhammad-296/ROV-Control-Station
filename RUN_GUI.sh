#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
if [ -x .venv/bin/python ]; then
    exec .venv/bin/python control_station.py
else
    exec python3 control_station.py
fi
