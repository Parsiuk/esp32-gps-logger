#!/bin/bash
# Compile gps_logger.py to .mpy and install it on the ESP32, then reboot.
# It must run precompiled: compiling on the device grows the Python heap,
# which leaves too little IDF heap for TLS uploads. main.py imports
# gps_logger at boot (hold joystick M during reset to skip).
#
# Usage: ./deploy.sh            (PORT=/dev/ttyUSB1 ./deploy.sh to override)
set -euo pipefail
cd "$(dirname "$0")"

PORT=${PORT:-/dev/ttyUSB0}
PY=.venv/bin/python
MPREMOTE=.venv/bin/mpremote

# mpy-cross must match the firmware's .mpy version (MicroPython 1.29)
if ! $PY -c "import mpy_cross" 2>/dev/null; then
    .venv/bin/pip install -q "mpy-cross==1.29.*"
fi

$PY -m mpy_cross -march=xtensawin -o gps_logger.mpy gps_logger.py

# A leftover gps_logger.py would be imported instead of the .mpy
$MPREMOTE connect "$PORT" \
    cp gps_logger.mpy main.py : \
    + exec "import os; 'gps_logger.py' in os.listdir() and os.remove('gps_logger.py')" \
    + ls : \
    + reset

echo "Deployed to $PORT"
