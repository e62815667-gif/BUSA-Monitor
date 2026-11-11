#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export XDG_CONFIG_HOME="${XDG_CONFIG_HOME:-$TASK_DIR/.runtime/config}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$TASK_DIR/.runtime/cache}"
export XDG_DATA_HOME="${XDG_DATA_HOME:-$TASK_DIR/.runtime/data}"
"$TASK_DIR/tools/run_checks.sh"
kicad-cli pcb drc --schematic-parity --all-track-errors --format json --exit-code-violations -o "$TASK_DIR/output/PCB_DRC.json" "$TASK_DIR/BUSA_R1.kicad_pcb"
/usr/bin/python3 "$TASK_DIR/tools/pcb_review.py"
/usr/bin/python3 "$TASK_DIR/tools/pcb_negative_control.py"
python "$TASK_DIR/tools/production_docs.py"
# Missing real-world data is an expected release blocker, not a successful check.
python "$TASK_DIR/tools/release_check.py"
