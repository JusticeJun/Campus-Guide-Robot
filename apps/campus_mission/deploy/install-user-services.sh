#!/usr/bin/env bash
# Install only. No navigation, arming, or mode requests are sent here.
set -euo pipefail
deploy_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/.config/systemd/user" "$HOME/.config/campus-rover"
if [[ ! -f "$HOME/.config/campus-rover/mission.env" ]]; then
  umask 077
  python3 - "$deploy_dir/../.env.example" "$HOME/.config/campus-rover/mission.env" <<'PY'
from pathlib import Path
import secrets
import sys
text = Path(sys.argv[1]).read_text()
text = text.replace('CAMPUS_WEB_TOKEN=\n', 'CAMPUS_WEB_TOKEN='+secrets.token_urlsafe(18)+'\n')
Path(sys.argv[2]).write_text(text)
PY
fi
chmod 600 "$HOME/.config/campus-rover/mission.env"
install -m 644 "$deploy_dir/campus-web.service" "$HOME/.config/systemd/user/"
install -m 644 "$deploy_dir/campus-navigation.service" "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
systemctl --user enable campus-web.service
loginctl enable-linger "$USER"
echo 'Installed boot web service. Start with: systemctl --user start campus-web'
