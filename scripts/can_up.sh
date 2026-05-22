#!/usr/bin/env bash
# Bring up CAN interfaces for the motorized shoe rig.
#   can0 = ELMO drives
#   can1 = IMUs
# can0 = 1 Mbit/s, can1 = 500 kbit/s.
#
# Usage:
#   ./scripts/can_up.sh                 # can0 at 1 Mbit/s, can1 at 500 kbit/s
#   ./scripts/can_up.sh can0            # only can0
#   CAN1_BITRATE=250000 ./scripts/can_up.sh  # override can1 bitrate

set -euo pipefail

CAN0_BITRATE="${CAN0_BITRATE:-500000}"
CAN1_BITRATE="${CAN1_BITRATE:-1000000}"

if [[ $# -gt 0 ]]; then
    ifaces=("$@")
else
    ifaces=(can0 can1)
fi

for iface in "${ifaces[@]}"; do
    case "${iface}" in
        can0) bitrate="${CAN0_BITRATE}" ;;
        can1) bitrate="${CAN1_BITRATE}" ;;
        *) bitrate="${CAN1_BITRATE}" ;;
    esac

    echo "[can_up] bringing up ${iface} at ${bitrate} bps"
    sudo ip link set "${iface}" down 2>/dev/null || true
    sudo ip link set "${iface}" up type can bitrate "${bitrate}"
done

echo "[can_up] done. Current state:"
ip -brief link show | grep -E '^can[0-9]+' || true
