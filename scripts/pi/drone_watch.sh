#!/bin/bash
# drone_watch.sh - race day: the user only powers the drone on.
# Every 5 s: when TELLO-FE18DB is visible, join it (profile "tello", ipv4.never-default so the default route,
# Tailscale and the Funnel stay on eth0), wait for 192.168.10.1 to answer, stop any simulator holding port 8080
# and start the ground station (scripts/pi/ground_station.sh -> tello_race fly in tmux window claude:drone).
# If the drone stays unreachable for DROP_AFTER seconds, the ground station is stopped and the wait starts again.
# If tello_race exits, it is logged and restarted once the drone answers again. Never sends flight commands itself.
# Logs go to the journal: journalctl -t drone-watch -f
set -u
REPO=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)
SSID=TELLO-FE18DB
PROFILE=tello
IFACE=${DRONE_IFACE:-wlan0}   # radio that joins the drone; DRONE_IFACE=wlan1 in ~/.config/drone-watch.env for a USB adapter
DRONE_IP=192.168.10.1
PORT=8080
INTERVAL=5
PING_WAIT=20     # seconds to wait for the drone to answer after joining
DROP_AFTER=20    # seconds unreachable before the ground station is stopped
RETRY_AFTER=10   # pause after tello_race exits before it may be started again
# extra tello_race options, e.g. GROUND_STATION_ARGS="--log-level debug" in ~/.config/drone-watch.env
read -r -a GS_ARGS <<< "${GROUND_STATION_ARGS:-}"

log() { logger -t drone-watch -- "$*"; echo "drone-watch: $*"; }

tello_active() { nmcli -t -f NAME con show --active 2>/dev/null | grep -Fxq "$PROFILE"; }
ssid_visible() {
	# a rescan can be refused right after another one; the cached list is good enough then
	nmcli -t -f SSID dev wifi list --rescan yes ifname "$IFACE" 2>/dev/null | grep -Fxq "$SSID" ||
		nmcli -t -f SSID dev wifi list --rescan no ifname "$IFACE" 2>/dev/null | grep -Fxq "$SSID"
}
drone_up() { ping -c 1 -W 1 "$DRONE_IP" >/dev/null 2>&1; }

port_pid() { ss -ltnpH "sport = :$PORT" 2>/dev/null | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2; }
cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null; }
fly_pid() { pgrep -u "$(id -u)" -f '^\./build/tello_race fly( |$)' | head -1; }

stop_pid() {  # $1 pid: SIGINT, then SIGTERM after 5 s
	kill -INT "$1" 2>/dev/null
	for _ in $(seq 10); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done
	kill -TERM "$1" 2>/dev/null; sleep 1
}

free_port() {  # stop whatever tello_race (simulator, replay) listens on the web port
	local pid cmd
	pid=$(port_pid); [ -n "$pid" ] || return 0
	cmd=$(cmdline "$pid")
	case "$cmd" in
		*tello_race*) log "stopping '$cmd' (pid $pid) holding port $PORT"; stop_pid "$pid" ;;
		*) log "port $PORT is held by '$cmd' (pid $pid), not a tello_race: cannot start the ground station"; return 1 ;;
	esac
	[ -z "$(port_pid)" ]
}

start_station() {
	free_port || return 1
	log "starting ground station (tello_race fly) in tmux window claude:drone"
	"$REPO/scripts/pi/ground_station.sh" start "${GS_ARGS[@]}" 2>&1 | logger -t drone-watch
	sleep 2
	if [ -n "$(fly_pid)" ]; then log "tello_race fly running (pid $(fly_pid))"; else log "tello_race fly did not start"; return 1; fi
}

stop_station() {
	log "stopping ground station: $*"
	"$REPO/scripts/pi/ground_station.sh" stop >/dev/null 2>&1
	local pid; pid=$(fly_pid); [ -n "$pid" ] && stop_pid "$pid"
}

ensure_profile() {
	if [ "$(nmcli -g ipv4.never-default con show "$PROFILE" 2>/dev/null)" != yes ]; then
		log "profile $PROFILE: setting ipv4.never-default yes"
		sudo -n nmcli con modify "$PROFILE" ipv4.never-default yes ipv6.method disabled connection.autoconnect no
	fi
}

until nmcli -t general status >/dev/null 2>&1; do sleep 2; done
nmcli -t -f NAME con show 2>/dev/null | grep -Fxq "$PROFILE" || { log "profile $PROFILE missing (see docs/HANDOFF.md)"; exit 1; }
ensure_profile
log "started: waiting for $SSID on $IFACE (repo $REPO)${GS_ARGS[*]:+, tello_race options: ${GS_ARGS[*]}}"

station=0         # 1 while we believe a ground station started by us is running
unreachable=0     # seconds the drone has not answered while joined
while true; do
	if ! tello_active; then
		[ "$station" = 1 ] && { stop_station "drone network $SSID is gone"; station=0; }
		if ssid_visible; then
			log "$SSID visible, joining (nmcli con up $PROFILE)"
			ensure_profile
			if sudo -n nmcli --wait 30 con up "$PROFILE" >/dev/null 2>&1; then
				log "joined $SSID: $(ip -4 -br addr show "$IFACE" | awk '{print $3}'), default route $(ip route show default | head -1)"
				unreachable=0
				for _ in $(seq "$PING_WAIT"); do drone_up && break; sleep 1; done
			else
				log "could not join $SSID"
			fi
		fi
	fi

	if tello_active; then
		if drone_up; then
			unreachable=0
			if [ -z "$(fly_pid)" ]; then
				if [ "$station" = 1 ]; then
					log "tello_race fly exited; restarting in ${RETRY_AFTER}s if the drone still answers"
					station=0; "$REPO/scripts/pi/ground_station.sh" stop >/dev/null 2>&1
					sleep "$RETRY_AFTER"; continue
				fi
				log "drone $DRONE_IP answers"
				start_station && station=1
			fi
		else
			unreachable=$((unreachable + INTERVAL))
			[ "$unreachable" = "$INTERVAL" ] && log "drone $DRONE_IP not answering on $SSID"
			if [ "$unreachable" -ge "$DROP_AFTER" ]; then
				[ -n "$(fly_pid)" ] && stop_station "drone unreachable for ${unreachable}s"
				station=0
				log "leaving $SSID (drone unreachable for ${unreachable}s), waiting again"
				sudo -n nmcli con down "$PROFILE" >/dev/null 2>&1
				unreachable=0
			fi
		fi
	fi
	sleep "$INTERVAL"
done
