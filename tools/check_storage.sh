#!/usr/bin/env bash
# Accept both the original SSD workspace and a full home on SSD.
set -euo pipefail
workspace="$HOME/ros2_ws"
[[ -d "$workspace/src" && -w "$workspace" ]] || { echo 'Workspace unavailable.' >&2; exit 1; }
root_device=$(findmnt -n -o MAJ:MIN -T /)
workspace_device=$(findmnt -n -o MAJ:MIN -T "$workspace")
[[ -n "$workspace_device" && "$workspace_device" != "$root_device" ]] || {
  echo 'SSD workspace is not mounted; refusing to write to the SD fallback.' >&2; exit 1;
}
