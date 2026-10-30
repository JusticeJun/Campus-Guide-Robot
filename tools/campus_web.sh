#!/usr/bin/env bash
set -euo pipefail
campus_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../apps/campus_mission" && pwd)"
campus_env="$HOME/.config/campus-rover/mission.env"
if [[ -f "$campus_env" ]]; then
  set -a
  source "$campus_env"
  set +a
fi
export PYTHONPATH="$campus_root${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m campus_mission.app "$@"
