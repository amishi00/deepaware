#!/usr/bin/env bash
# Start the full mock stack:
#   - mock CAN publishers on vcan0 and vcan1 (background)
#   - the API + dashboard server (foreground)
#
# Prereq: run scripts/setup_vcan.sh first so vcan0/vcan1 exist.
# Ctrl-C stops everything.
set -e
cd "$(dirname "$0")/.."
export PYTHONPATH="$PWD/src"

cleanup() { echo; echo "stopping..."; kill 0; }
trap cleanup EXIT INT TERM

echo "[+] mock publisher -> vcan0"
python3 -m openarm_pipeline.can_publisher --channel vcan0 --rate 500 &
echo "[+] mock publisher -> vcan1"
python3 -m openarm_pipeline.can_publisher --channel vcan1 --rate 500 &

sleep 1
echo "[+] API + dashboard -> http://127.0.0.1:8000"
uvicorn openarm_pipeline.api:app --host 0.0.0.0 --port 8000
