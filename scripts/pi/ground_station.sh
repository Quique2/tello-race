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
	tmux has-session -t "$SESSION:$WINDOW" 2>/dev/null && tmux send-keys -t "$SESSION:$WINDOW" C-c
	sleep 2; tmux kill-window -t "$SESSION:$WINDOW" 2>/dev/null
	echo "ground station stopped"
	exit 0 ;;
status)
	tmux has-session -t "$SESSION:$WINDOW" 2>/dev/null && echo "tmux window $SESSION:$WINDOW running" || echo "not running"
	ss -ltn | grep -q ":$PORT " && echo "port $PORT listening" || echo "port $PORT not listening"
	tailscale funnel status 2>/dev/null
	exit 0 ;;
esac

[ -x "$REPO/build/tello_race" ] || { echo "build first: cmake --preset linux-release && cmake --build --preset linux-release" >&2; exit 1; }
if ss -ltn | grep -q ":$PORT "; then echo "port $PORT already in use (another tello_race running? $0 stop)" >&2; exit 1; fi
ping -c 1 -W 1 192.168.10.1 >/dev/null 2>&1 || echo "warning: drone 192.168.10.1 not reachable (scripts/pi/drone_wifi.sh up)" >&2

tmux has-session -t "$SESSION" 2>/dev/null || tmux new-session -d -s "$SESSION" -c "$REPO"
tmux kill-window -t "$SESSION:$WINDOW" 2>/dev/null
args=$(printf ' %q' "$@")
tmux new-window -d -t "$SESSION" -n "$WINDOW" -c "$REPO" \
	"./build/tello_race fly --headless --set web.enabled=1$args; echo; echo 'tello_race exited - press Enter to close'; read"
ensure_funnel

for i in $(seq 1 20); do ss -ltn | grep -q ":$PORT " && break; sleep 0.5; done
echo "tello_race fly running in tmux window $SESSION:$WINDOW (tmux attach -t $SESSION)"
echo "local : http://$(ip -4 -br addr show eth0 | awk '{print $3}' | cut -d/ -f1):$PORT/"
url=$(funnel_url); [ -n "$url" ] && echo "public: $url   (add ?token=watch to spectate; operator token from config/default.ini)"
