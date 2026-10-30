#!/usr/bin/env bash
set -euo pipefail
deploy_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 - <<'PY'
from pathlib import Path
import hashlib
import platform
import urllib.request
if platform.machine() not in ('aarch64', 'arm64'):
    raise SystemExit('This installer targets the Jetson ARM64 deployment.')
version = '2026.10.5'
digest = '72e0d05a67e09da87cf495e409a28c3cc5994dd23fc66a982eebea6a55923b7d'
path = Path.home()/'.local/share/campus-tools/cloudflared/cloudflared'
path.parent.mkdir(parents=True, exist_ok=True)
if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
    url = f'https://github.com/cloudflare/cloudflared/releases/download/{version}/cloudflared-linux-arm64'
    with urllib.request.urlopen(url, timeout=120) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != digest:
        raise SystemExit('cloudflared checksum mismatch')
    tmp = path.with_suffix('.tmp')
    tmp.write_bytes(data)
    tmp.chmod(0o755)
    tmp.replace(path)
print('Verified cloudflared '+version)
PY
install -m 644 "$deploy_dir/campus-tunnel.service" "$HOME/.config/systemd/user/"
systemctl --user daemon-reload
systemctl --user enable campus-tunnel.service
echo 'Installed. Start: systemctl --user start campus-tunnel'
