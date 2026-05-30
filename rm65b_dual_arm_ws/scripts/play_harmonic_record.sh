#!/usr/bin/env bash
set -euo pipefail

cat >&2 <<'EOF'
play_harmonic_record.sh is disabled.

Acceptance videos must be recorded through play_harmonic_planned_record.sh or
record_moveit_days_harmonic.sh, which use MoveIt planned replay plus Gazebo
contact/RuntimeLinkAttacher evidence. The old single-arm set-pose playback path
was removed to avoid accidental demo choreography.
EOF

exit 2
