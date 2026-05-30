#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "play_harmonic_camera_record.sh is deprecated." >&2
echo "Delegating to fixed-base dual-arm joint-planning playback." >&2

exec "$SCRIPT_DIR/play_harmonic_planned_record.sh" "$@"
