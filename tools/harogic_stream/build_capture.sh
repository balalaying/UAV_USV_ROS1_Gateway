#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$SCRIPT_DIR/bin"

g++ -std=c++11 -O2 \
  -I/opt/htraapi/inc \
  "$SCRIPT_DIR/harogic_capture.cpp" \
  -L/opt/htraapi/lib/x86_64 \
  -lhtraapi -lliquid \
  -Wl,-rpath,/opt/htraapi/lib/x86_64 \
  -o "$SCRIPT_DIR/bin/harogic_capture"

echo "built $SCRIPT_DIR/bin/harogic_capture"
