#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/serial/by-path/platform-3610000.usb-usb-0:2.3:1.0-port0}"
BAUDRATE="${2:-921600}"

MicroXRCEAgent serial --dev "$DEVICE" -b "$BAUDRATE"
