#!/bin/bash
# ground_station.sh [start|stop|status] [extra tello_race options...]
# Starts "tello_race fly --headless --set web.enabled=1" in tmux session "claude", window "drone",
# makes sure Tailscale Funnel publishes port 8080 and prints the public URL (never the token).
# Examples:  scripts/pi/ground_station.sh                 real flight session (drone Wi-Fi must be up)
#            scripts/pi/ground_station.sh start --dry-run video + telemetry only, no flight commands
#            scripts/pi/ground_station.sh stop
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
SESSION=claude
WINDOW=drone
PORT=8080
LOG=logs/ground_station.log   # tello_race output, appended (relative to the repo)

window_exists() { tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -Fxq "$WINDOW"; }

funnel_url() {
	local name
	name=$(tailscale status --json 2>/dev/null | jq -r '.Self.DNSName // empty' | sed 's/\.$//')
	[ -n "$name" ] && echo "https://$name/"
}

ensure_funnel() {
	command -v tailscale >/dev/null || { echo "tailscale not installed: page only at http://<pi>:$PORT/"; return; }
	if ! tailscale funnel status 2>/dev/null | grep -q "127.0.0.1:$PORT"; then
		sudo tailscale funnel --bg $PORT >/dev/null || echo "warning: could not enable Funnel (enable it for this node in the admin console)" >&2
	fi
}

cmd=${1:-start}
case "$cmd" in start|stop|status) shift || true ;; *) cmd=start ;; esac

case "$cmd" in
stop)
	window_exists && tmux send-keys -t "$SESSION:$WINDOW" C-c
	sleep 2; window_exists && tmux kill-window -t "$SESSION:$WINDOW"
	echo "ground station stopped"
	exit 0 ;;
status)
	window_exists && echo "tmux window $SESSION:$WINDOW exists" || echo "no tmux window $SESSION:$WINDOW"
	pgrep -af '^\./build/tello_race fly( |$)' || echo "tello_race fly not running"
	ss -ltn | grep -q ":$PORT " && echo "port $PORT listening" || echo "port $PORT not listening"
	tailscale funnel status 2>/dev/null
	exit 0 ;;
esac

[ -x "$REPO/build/tello_race" ] || { echo "build first: cmake --preset linux-release && cmake --build --preset linux-release" >&2; exit 1; }
if ss -ltn | grep -q ":$PORT "; then echo "port $PORT already in use (another tello_race running? $0 stop)" >&2; exit 1; fi
ping -c 1 -W 1 192.168.10.1 >/dev/null 2>&1 || echo "warning: drone 192.168.10.1 not reachable (scripts/pi/drone_wifi.sh up)" >&2

tmux has-session -t "$SESSION" 2>/dev/null || tmux new-session -d -s "$SESSION" -c "$REPO"
window_exists && tmux kill-window -t "$SESSION:$WINDOW"
mkdir -p "$REPO/logs"
args=""; [ $# -gt 0 ] && args=$(printf ' %q' "$@")   # no options -> nothing (printf alone would pass '')
# "-t session:" = next free index in that session ("-t session" means its window 0 and fails when it exists)
tmux new-window -d -t "$SESSION:" -n "$WINDOW" -c "$REPO" \
	"echo \"=== \$(date '+%F %T') tello_race fly$args\" >> $LOG; ./build/tello_race fly --headless --set web.enabled=1$args 2>&1 | tee -a $LOG; echo \"=== \$(date '+%F %T') exit \${PIPESTATUS[0]}\" | tee -a $LOG; echo 'tello_race exited - press Enter to close'; read" ||
	{ echo "could not create tmux window $SESSION:$WINDOW" >&2; exit 1; }
ensure_funnel

# tello_race opens the web port only after the drone answers "command" (up to 4 x 3 s)
for i in $(seq 1 30); do ss -ltn | grep -q ":$PORT " && break; sleep 0.5; done
if ! ss -ltn | grep -q ":$PORT "; then
	echo "tello_race fly is not listening on $PORT (see tmux window $SESSION:$WINDOW and $LOG)" >&2; exit 1
fi
echo "tello_race fly running in tmux window $SESSION:$WINDOW (tmux attach -t $SESSION), log $LOG"
echo "local : http://$(ip -4 -br addr show eth0 | awk '{print $3}' | cut -d/ -f1):$PORT/"
url=$(funnel_url); [ -n "$url" ] && echo "public: $url   (add ?token=watch to spectate; operator token from config/default.ini)"
