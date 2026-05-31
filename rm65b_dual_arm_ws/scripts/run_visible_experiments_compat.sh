#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"

OUT_ROOT="${OUT_ROOT:-$HOME/rm65b_visible_$(date +%Y%m%d_%H%M%S)}"
DOMAIN_START="${DOMAIN_START:-211}"
CAPTURE_TIMEOUT="${CAPTURE_TIMEOUT:-420}"
RECORD_RVIZ="${RECORD_RVIZ:-0}"
GUI_START_DELAY="${GUI_START_DELAY:-12}"
GUI_READY_CHECK_DELAY="${GUI_READY_CHECK_DELAY:-2}"

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

normalize_day_id() {
  case "${1,,}" in
    d1|day1|day01) echo "day01" ;;
    d2|day2|day02) echo "day02" ;;
    d3|day3|day03) echo "day03" ;;
    d4|day4|day04) echo "day04" ;;
    d5|day5|day05) echo "day05" ;;
    all|"") echo "all" ;;
    *) echo "$1" ;;
  esac
}

patch_legacy_gz_probe() {
  local script="$SCRIPT_DIR/play_harmonic_planned_record.sh"
  [[ -f "$script" ]] || return 0
  python3 - "$script" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
updated = text.replace(
    "  (gz sim --version || gz sim --versions || gz --versions || true)\n",
    "  echo \"gz_cli=available\"\n",
)
updated = updated.replace(
    "  gz sim --versions\n",
    "  echo \"gz_cli=available\"\n",
)
if updated != text:
    path.write_text(updated, encoding="utf-8")
PY
}

check_environment() {
  [[ -f /opt/ros/humble/setup.bash ]] || {
    echo "Missing /opt/ros/humble/setup.bash" >&2
    exit 1
  }
  [[ -f "$WORKSPACE/install/setup.bash" ]] || {
    echo "Missing $WORKSPACE/install/setup.bash; build the workspace first." >&2
    exit 1
  }
  command -v gz >/dev/null 2>&1 || {
    echo "Missing gz command." >&2
    exit 1
  }
  command -v ros2 >/dev/null 2>&1 || true

  source_setup_file /opt/ros/humble/setup.bash
  source_setup_file "$WORKSPACE/install/setup.bash"

  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "WARN: DISPLAY/WAYLAND_DISPLAY is empty; Gazebo GUI cannot pop up." >&2
  fi
}

run_old_visual_day() {
  local day="$1"
  local domain="$2"
  local out_dir="$OUT_ROOT/$day"
  local backend_log="$out_dir/logs/visible_backend.log"
  local gui_log="$out_dir/logs/visible_gz_gui.log"
  local day_partition="${GZ_PARTITION:-rm65b_visible_${day}_${domain}_$$}"
  local backend_pid=""
  local gui_pid=""
  mkdir -p "$out_dir/logs"

  echo "=== $day old-compatible visual run ==="
  echo "output=$out_dir"
  echo "gz_partition=$day_partition"
  echo "backend_log=$backend_log"
  echo "gui_log=$gui_log"

  (
    export ROS_DOMAIN_ID="$domain"
    export GZ_PARTITION="$day_partition"
    export RECORD_RVIZ CAPTURE_TIMEOUT
    bash "$SCRIPT_DIR/play_harmonic_planned_record.sh" "$out_dir" "$WORKSPACE" "$domain" "$day"
  ) > "$backend_log" 2>&1 &
  backend_pid="$!"

  sleep "$GUI_START_DELAY"

  if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
    GZ_PARTITION="$day_partition" gz sim -g > "$gui_log" 2>&1 &
    gui_pid="$!"
    sleep "$GUI_READY_CHECK_DELAY"
    if ! kill -0 "$gui_pid" >/dev/null 2>&1; then
      echo "FAILED: Gazebo GUI exited immediately for $day. See $gui_log" >&2
      tail -n 80 "$gui_log" >&2 || true
      kill "$backend_pid" >/dev/null 2>&1 || true
      wait "$backend_pid" >/dev/null 2>&1 || true
      return 3
    fi
    echo "Gazebo GUI opened for $day."
  fi

  local status=0
  wait "$backend_pid" || status="$?"

  if [[ -n "$gui_pid" ]] && kill -0 "$gui_pid" >/dev/null 2>&1; then
    kill "$gui_pid" >/dev/null 2>&1 || true
  fi

  if [[ "$status" -ne 0 ]]; then
    echo "FAILED: $day backend exited with $status" >&2
    echo "Last backend log lines:" >&2
    tail -n 80 "$backend_log" >&2 || true
    return "$status"
  fi
}

run_day() {
  local day="$1"
  local domain="$2"

  if [[ -f "$SCRIPT_DIR/run_day_visual.sh" ]]; then
    echo "=== $day standard visual run ==="
    RECORD_RVIZ="$RECORD_RVIZ" CAPTURE_TIMEOUT="$CAPTURE_TIMEOUT" GUI_START_DELAY="$GUI_START_DELAY" \
      bash "$SCRIPT_DIR/run_day_visual.sh" "$day" "$domain" "$OUT_ROOT"
  else
    run_old_visual_day "$day" "$domain"
  fi
}

main() {
  patch_legacy_gz_probe
  check_environment

  local requested=("$@")
  if [[ "${#requested[@]}" -eq 0 ]]; then
    requested=(all)
  fi

  local days=()
  local item
  for item in "${requested[@]}"; do
    item="$(normalize_day_id "$item")"
    if [[ "$item" == "all" ]]; then
      days=(day01 day02 day03 day04 day05)
      break
    fi
    days+=("$item")
  done

  mkdir -p "$OUT_ROOT"
  echo "workspace=$WORKSPACE"
  echo "output_root=$OUT_ROOT"
  echo "domain_start=$DOMAIN_START"
  echo "record_rviz=$RECORD_RVIZ"

  local idx=0
  local day
  for day in "${days[@]}"; do
    run_day "$day" "$((DOMAIN_START + idx))"
    idx=$((idx + 1))
  done

  echo "visible_experiments_complete=$OUT_ROOT"
}

main "$@"
