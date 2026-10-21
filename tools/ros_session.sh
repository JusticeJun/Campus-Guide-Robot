#!/usr/bin/env bash
# Keep ROS processes alive when SSH/VNC clients disconnect. No automatic restart.
set -euo pipefail
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
tmux_bin="$HOME/.local/share/campus-tools/tmux/usr/bin/tmux"
if [[ ! -x "$tmux_bin" ]]; then tmux_bin=$(command -v tmux || true); fi
[[ -n "$tmux_bin" ]] || { echo 'tmux is not installed.' >&2; exit 1; }
tmux_cmd=("$tmux_bin" -L campus-ros -f "$task_dir/ros_session.tmux.conf")
action=${1:-status}
if [[ "$action" == status ]]; then
  "${tmux_cmd[@]}" list-panes -a -F '#{session_name}: #{?pane_dead,EXITED,RUNNING} (exit=#{pane_dead_status}) #{pane_current_command}' 2>/dev/null || echo 'No persistent ROS sessions.'
  exit 0
fi
[[ $# -ge 2 && "$2" =~ ^[a-zA-Z0-9_-]+$ ]] || {
  echo 'Usage: bash ros_session.sh start NAME COMMAND... | attach NAME | stop NAME | status' >&2; exit 2;
}
name=$2
target="=$name"
pane="$name:0.0"
case "$action" in
  start)
    shift 2
    [[ $# -gt 0 ]] || { echo 'Missing command.' >&2; exit 2; }
    if "${tmux_cmd[@]}" has-session -t "$target" 2>/dev/null; then
      dead=$("${tmux_cmd[@]}" display-message -p -t "$pane" '#{pane_dead}')
      if [[ "$dead" != 1 ]]; then
        echo "Session $name is already running; use attach $name. Nothing restarted." >&2
        exit 1
      fi
      "${tmux_cmd[@]}" kill-session -t "$target"
    fi
    # Explicit env prevents an older tmux server from reusing a stale DISPLAY/map.
    # env options must precede assignments.
    unset_args=(); set_args=()
    for key in MAP_DIR DISPLAY XAUTHORITY ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI; do
      if [[ -v "$key" ]]; then set_args+=("$key=${!key}"); else unset_args+=(-u "$key"); fi
    done
    printf -v command '%q ' env "${unset_args[@]}" "${set_args[@]}" bash --noprofile --norc "$task_dir/ros_session_exec.sh" "$@"
    "${tmux_cmd[@]}" new-session -d -s "$name" -x 160 -y 45 "$command"
    echo "Started $name in persistent tmux. Logs: bash $task_dir/ros_session.sh attach $name"
    ;;
  attach)
    exec "${tmux_cmd[@]}" attach-session -t "$target"
    ;;
  stop)
    if ! "${tmux_cmd[@]}" has-session -t "$target" 2>/dev/null; then echo "$name is not running."; exit 0; fi
    if [[ $("${tmux_cmd[@]}" display-message -p -t "$pane" '#{pane_dead}') != 1 ]]; then
      "${tmux_cmd[@]}" send-keys -t "$pane" C-c
      echo "Sent Ctrl+C to $name. Waiting for clean shutdown (up to 150 s)."
      for ((i=0; i<150; i++)); do
        if [[ $("${tmux_cmd[@]}" display-message -p -t "$pane" '#{pane_dead}') == 1 ]]; then break; fi
        sleep 1
      done
    fi
    if [[ $("${tmux_cmd[@]}" display-message -p -t "$pane" '#{pane_dead}') != 1 ]]; then
      echo "Still saving/running: $name. NOT force-killed; attach to inspect." >&2; exit 1
    fi
    "${tmux_cmd[@]}" kill-session -t "$target"
    echo "$name stopped."
    ;;
  *) echo "Unknown action: $action" >&2; exit 2 ;;
esac
