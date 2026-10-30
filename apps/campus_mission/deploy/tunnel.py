"""Keep the current temporary hostname outside Git; never embed control keys."""
import argparse
from pathlib import Path
import re
import signal
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--port', type=int, default=8080)
args = parser.parse_args()
if not 1 <= args.port <= 65535:
    parser.error('Invalid port')
state = Path.home()/'.local/state/campus-rover'
state.mkdir(parents=True, exist_ok=True)
url_file = state/'public-url'
url_file.unlink(missing_ok=True)
binary = Path.home()/'.local/share/campus-tools/cloudflared/cloudflared'
process = subprocess.Popen([str(binary), '--no-autoupdate', 'tunnel', '--protocol', 'http2',
                            '--url', f'http://127.0.0.1:{args.port}'],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def stop(*_):
    if process.poll() is None:
        process.terminate()


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
try:
    for line in process.stdout:
        match = re.search(r'https://[a-z0-9-]+\.trycloudflare\.com', line)
        if match:
            temp = url_file.with_suffix('.tmp')
            temp.write_text(match[0]+'\n')
            temp.replace(url_file)
        print(line.rstrip(), flush=True)
    code = process.wait()
finally:
    stop()
    url_file.unlink(missing_ok=True)
raise SystemExit(code)
