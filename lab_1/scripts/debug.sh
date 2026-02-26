#!/usr/bin/env bash
set -euo pipefail

# Projektroot bestimmen (eine Ebene über scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ELF="${1:-$PROJECT_ROOT/build/stm32-lab-1.elf}"
GDB="${GDB:-arm-none-eabi-gdb}"
PORT="${PORT:-3333}"

if [[ ! -f "$ELF" ]]; then
  echo "ELF nicht gefunden: $ELF"
  echo "Usage: ./scripts/debug.sh [pfad/zur.elf]"
  exit 1
fi

exec "$GDB" "$ELF" \
  -ex "set pagination off" \
  -ex "set confirm off" \
  -ex "target extended-remote :$PORT" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "monitor reset halt" \
  -ex "break main" \
  -ex "continue"