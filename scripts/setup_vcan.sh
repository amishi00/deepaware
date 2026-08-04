#!/usr/bin/env bash
# Bring up two virtual CAN interfaces (vcan0, vcan1) to stand in for the
# OpenArm's can0 / can1 buses when no hardware/CAN-FD adapter is available.
#
# NOTE: virtual CAN does NOT accept bitrate/FD timing parameters, so
# `openarm-can-cli can_configure` will fail on vcan with "Operation not
# supported". That is expected — vcan has no physical bus to time. It still
# carries frames, which is all the data pipeline needs. See README (Task 1).
set -e

for dev in vcan0 vcan1; do
  if ip link show "$dev" >/dev/null 2>&1; then
    echo "[=] $dev already exists"
  else
    sudo modprobe vcan
    sudo ip link add dev "$dev" type vcan
    echo "[+] created $dev"
  fi
  sudo ip link set up "$dev"
done

echo
echo "Interfaces up:"
ip -br link show type vcan
