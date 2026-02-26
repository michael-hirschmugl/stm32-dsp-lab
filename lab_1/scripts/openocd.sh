#!/usr/bin/env bash
set -euo pipefail

OPENOCD="${OPENOCD:-openocd}"

INTERFACE_CFG="${INTERFACE_CFG:-interface/stlink.cfg}"
TARGET_CFG="${TARGET_CFG:-target/stm32f2x.cfg}"

echo "Starte OpenOCD..."
exec "$OPENOCD" \
  -f "$INTERFACE_CFG" \
  -f "$TARGET_CFG"