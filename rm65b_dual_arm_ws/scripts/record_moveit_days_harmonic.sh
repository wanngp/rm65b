#!/usr/bin/env bash
set -euo pipefail

OUT_ROOT="${1:?output root directory required}"
WORKSPACE="${2:-/tmp/rm65b_dual_arm_ws_ascii_verify2}"
DOMAIN_START="${3:-90}"

mkdir -p "$OUT_ROOT"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -gt 3 ]; then
  days=("${@:4}")
else
  days=(day01 day02 day03 day04 day05)
fi
idx=0
for day in "${days[@]}"; do
  domain=$((DOMAIN_START + idx))
  out_dir="$OUT_ROOT/$day"
  echo "recording $day to $out_dir with ROS_DOMAIN_ID=$domain"
  bash "$SCRIPT_DIR/play_harmonic_planned_record.sh" "$out_dir" "$WORKSPACE" "$domain" "$day"
  idx=$((idx + 1))
done

echo "moveit_day_recordings_complete: $OUT_ROOT"
