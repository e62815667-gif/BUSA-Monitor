#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export XDG_CONFIG_HOME="${XDG_CONFIG_HOME:-$TASK_DIR/.runtime/config}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$TASK_DIR/.runtime/cache}"
export XDG_DATA_HOME="${XDG_DATA_HOME:-$TASK_DIR/.runtime/data}"
mkdir -p "$XDG_CONFIG_HOME" "$XDG_CACHE_HOME" "$XDG_DATA_HOME" "$TASK_DIR/output"
python "$TASK_DIR/tools/generate.py"
kicad-cli sch erc --format json --exit-code-violations -o "$TASK_DIR/output/ERC.json" "$TASK_DIR/BUSA_R1.kicad_sch"
kicad-cli sch export netlist --format kicadxml -o "$TASK_DIR/output/NETLIST.xml" "$TASK_DIR/BUSA_R1.kicad_sch"
python "$TASK_DIR/tools/verify.py"
kicad-cli sch export pdf -o "$TASK_DIR/output/BUSA_R1.pdf" "$TASK_DIR/BUSA_R1.kicad_sch"
python "$TASK_DIR/tools/report.py"
