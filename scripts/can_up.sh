#!/usr/bin/env bash
# Bring up CAN interfaces for the motorized shoe rig.
#
# Interface names follow the HAT silkscreen regardless of kernel probe order:
#   HAT CAN_0 (spi0.0) -> can0 = ELMO drives  (1 Mbit/s)
#   HAT CAN_1 (spi0.1) -> can1 = IMUs (Teensy) (500 kbit/s)
# The kernel names the two MCP2515 channels in probe order, which on this Pi
# comes out swapped, and udev cannot swap two names. So we rename here, via a
# temporary name, before bringing anything up.
#
# The ELMO drives are hard-set to 1 Mbit/s. Bringing can0 up at any other rate
# leaves the controller in ERROR-PASSIVE with zero RX/TX and the app spamming
# "SYNC send failed ... errno=105: No buffer space available" (seen 2026-09-08).
#
# Usage:
#   ./scripts/can_up.sh                 # both interfaces
#   ./scripts/can_up.sh can0            # only can0
#   CAN1_BITRATE=250000 ./scripts/can_up.sh  # override can1 bitrate

set -euo pipefail

CAN0_BITRATE="${CAN0_BITRATE:-1000000}"
CAN1_BITRATE="${CAN1_BITRATE:-500000}"

# --- Name interfaces by HAT port --------------------------------------------
iface_on() {  # prints the net interface sitting on SPI device $1, or nothing
    ls "/sys/bus/spi/devices/$1/net" 2>/dev/null || true
}
rename_if() { # rename $1 -> $2 unless already named that
    [[ "$1" == "$2" ]] || sudo ip link set "$1" name "$2"
}

on_cs0="$(iface_on spi0.0)"
on_cs1="$(iface_on spi0.1)"

if [[ -n "${on_cs0}" && -n "${on_cs1}" ]] && \
   [[ "${on_cs0}" != "can0" || "${on_cs1}" != "can1" ]]; then
    echo "[can_up] renaming: spi0.0 '${on_cs0}' -> can0, spi0.1 '${on_cs1}' -> can1"
    sudo ip link set "${on_cs0}" down
    sudo ip link set "${on_cs1}" down
    rename_if "${on_cs0}" cantmp
    rename_if "${on_cs1}" can1
    rename_if cantmp can0
fi

# --- Bring up ----------------------------------------------------------------
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
    # txqueuelen: the default of 10 is too shallow for the 1 kHz SYNC + SDO
    # traffic on can0 and shows up as sporadic ENOBUFS on sends.
    sudo ip link set "${iface}" txqueuelen 100
    sudo ip link set "${iface}" up type can bitrate "${bitrate}"
done

echo "[can_up] done. Current state:"
ip -brief link show | grep -E '^can[0-9]+' || true
for iface in "${ifaces[@]}"; do
    state="$(ip -details link show "${iface}" 2>/dev/null | grep -o 'can state [A-Z-]*' || true)"
    echo "[can_up] ${iface}: ${state:-unknown}"
done
