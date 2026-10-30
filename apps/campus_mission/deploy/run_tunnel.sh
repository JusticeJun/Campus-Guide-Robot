#!/usr/bin/env bash
set -euo pipefail
source "$HOME/.config/campus-rover/mission.env"
deploy_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$deploy_dir/tunnel.py" --port "${CAMPUS_WEB_PORT:-8080}"
