#!/usr/bin/env bash
set -euo pipefail
source "$HOME/.config/campus-rover/mission.env"
public_file="$HOME/.local/state/campus-rover/public-url"
if [[ -s "$public_file" ]]; then
  printf 'External control link: %s/#access=%s\n' "$(cat "$public_file")" "$CAMPUS_WEB_TOKEN"
fi
printf 'Local control link: http://%s.local:%s/#access=%s\n' "$(hostname)" "${CAMPUS_WEB_PORT:-8080}" "$CAMPUS_WEB_TOKEN"
campus_lan_ip="$(python3 - <<'PY'
import json
import subprocess
try:
    routes = json.loads(subprocess.check_output(
        ['ip', '-j', '-4', 'route', 'get', '1.1.1.1'], stderr=subprocess.DEVNULL))
    print(routes[0].get('prefsrc', ''))
except (OSError, subprocess.CalledProcessError, ValueError, IndexError):
    pass
PY
)"
if [[ -n "$campus_lan_ip" ]]; then
  printf 'Same-network IP link (DNS not required): http://%s:%s/#access=%s\n' "$campus_lan_ip" "${CAMPUS_WEB_PORT:-8080}" "$CAMPUS_WEB_TOKEN"
fi
