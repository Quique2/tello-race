#!/bin/bash
# vnc-screens.sh - the screen follows the VNC connection.
#   no VNC client  -> local:  DSI-2 on, wayvnc captures DSI-2, HDMI-A-1 off
#   VNC client(s)  -> remote: HDMI-A-1 on (1920x1080), wayvnc captures it, DSI-2 off
# Every change is a separate wlr-randr call with a pause, skipped when it would change nothing
# (a failed or no-op output configuration makes wayvnc 0.9.1 segfault), and the last enabled
# output is never turned off. kanshi is not touched here: ~/.config/kanshi/config only holds
# the boot layout. Any failure ends in local with DSI-2 on.
set -u
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
SOCK=/tmp/wayvnc/wayvncctl.sock
LOCAL=DSI-2
REMOTE=HDMI-A-1
MODE=1920x1080
PAUSE=1
# 0 = keep the unused output enabled and only switch the capture (fallback if on/off still crashes)
TOGGLE_OUTPUTS=${VNC_SCREENS_TOGGLE:-1}
STATE=unknown

log() { echo "vnc-screens: $*"; }
ctl() { wayvncctl --socket="$SOCK" "$@"; }

present()  { wlr-randr --json 2>/dev/null | jq -e --arg o "$1" 'any(.[]; .name==$o)' >/dev/null; }
is_on()    { wlr-randr --json 2>/dev/null | jq -e --arg o "$1" 'any(.[]; .name==$o and .enabled)' >/dev/null; }
n_on()     { wlr-randr --json 2>/dev/null | jq '[.[] | select(.enabled)] | length'; }
enabled()  { wlr-randr --json 2>/dev/null | jq -r '[.[] | select(.enabled) | .name] | join(",")'; }
captured() { ctl --json output-list 2>/dev/null | jq -r '.[] | select(.captured) | .name'; }
clients()  { ctl --json client-list 2>/dev/null | jq 'length' 2>/dev/null || echo 0; }

randr() {
	local out
	if ! out=$(wlr-randr "$@" 2>&1); then log "wlr-randr $* failed: $out"; return 1; fi
	sleep "$PAUSE"
}

output_on() {  # $1 output [$2 mode]
	is_on "$1" && return 0
	if [ -n "${2:-}" ]; then randr --output "$1" --on --mode "$2" || return 1
	else randr --output "$1" --on || return 1; fi
	is_on "$1" || { log "$1 still off after enabling"; return 1; }
}

output_off() {  # $1 output; never switches off the last enabled output
	[ "$TOGGLE_OUTPUTS" = 1 ] || return 0
	is_on "$1" || return 0
	[ "$(n_on)" -ge 2 ] || { log "not turning off $1: it is the only enabled output"; return 1; }
	randr --output "$1" --off
}

capture() {  # $1 output
	[ "$(captured)" = "$1" ] && return 0
	ctl output-set "$1" >/dev/null 2>&1 || { log "wayvnc output-set $1 failed"; return 1; }
	sleep "$PAUSE"
}

go_local() {
	output_on "$LOCAL" || log "could not enable $LOCAL"
	capture "$LOCAL"
	is_on "$LOCAL" && output_off "$REMOTE"
	STATE=local
	log "local: enabled $(enabled), wayvnc captures $(captured)"
}

go_remote() {
	[ "$STATE" = remote ] && return 0
	if ! present "$REMOTE"; then log "$REMOTE not present, staying local"; [ "$STATE" = local ] || go_local; return; fi
	if ! output_on "$REMOTE" "$MODE"; then log "could not enable $REMOTE, staying local"; go_local; return; fi
	capture "$REMOTE" || { go_local; return; }
	output_off "$LOCAL"
	STATE=remote
	log "remote: enabled $(enabled), wayvnc captures $(captured)"
}

if [ "${1:-}" = --ensure-local ]; then  # ExecStopPost: DSI-2 on if it is off, nothing else
	output_on "$LOCAL"; exit
fi

trap 'go_local; exit 0' INT TERM

until wlr-randr >/dev/null 2>&1; do sleep 1; done
if [ "$(clients)" -gt 0 ]; then go_remote; else go_local; fi

# --reconnect keeps the stream across wayvnc restarts (wayvnc-shutdown / wayvnc-startup events)
while true; do
	ctl --json --wait --reconnect event-receive 2>/dev/null |
	jq --unbuffered -r '[.method, (.params.connection_count // "" | tostring)] | @tsv' |
	while IFS=$'\t' read -r method n; do
		case "$method" in
			client-connected) log "client connected (count $n)"; go_remote ;;
			client-disconnected) log "client disconnected (count $n)"; [ "${n:-0}" = 0 ] && go_local ;;
			wayvnc-shutdown) log "wayvnc shut down"; STATE=unknown; go_local ;;
			wayvnc-startup) [ "$(clients)" -gt 0 ] && go_remote ;;
		esac
	done
	log "event stream ended, falling back to local"
	STATE=unknown; go_local
	sleep 2
done
