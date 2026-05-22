#!/usr/bin/env bash
# Bring down CAN interfaces.
#
# Usage:
#   ./scripts/can_down.sh         # both can0 and can1
#   ./scripts/can_down.sh can0    # only can0

set -euo pipefail

if [[ $# -gt 0 ]]; then
    ifaces=("$@")
else
    ifaces=(can0 can1)
fi

for iface in "${ifaces[@]}"; do
    echo "[can_down] bringing down ${iface}"
    sudo ip link set "${iface}" down 2>/dev/null || true
done
