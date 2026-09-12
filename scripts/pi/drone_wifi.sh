#!/bin/bash
# drone_wifi.sh up|down|status - put the Pi's built-in Wi-Fi (wlan0) on the drone's network
# while the internet (and Tailscale) keeps going through eth0 (laptop ICS, 192.168.137.1).
# Profile "tello" (created once, see docs/HANDOFF.md): open SSID TELLO-FE18DB, ipv4.never-default yes.
set -u
PROFILE=tello
DRONE_IP=192.168.10.1
IFACE=wlan0

default_dev() { ip route show default | awk '{for(i=1;i<NF;i++) if($i=="dev"){print $(i+1); exit}}'; }

status() {
	echo "wlan0 profile : $(nmcli -g GENERAL.CONNECTION dev show $IFACE)"
	echo "wlan0 address : $(ip -4 -br addr show $IFACE | awk '{print $3}')"
	echo "default routes:"; ip route show default | sed 's/^/    /'
	if ping -c 2 -W 1 $DRONE_IP >/dev/null 2>&1; then echo "drone $DRONE_IP : reachable"; else echo "drone $DRONE_IP : NOT reachable"; fi
	if ping -c 1 -W 2 1.1.1.1 >/dev/null 2>&1; then echo "internet      : ok"; else echo "internet      : NOT reachable"; fi
	command -v tailscale >/dev/null && echo "tailscale     : $(tailscale status --json 2>/dev/null | jq -r '.BackendState')"
}

case "${1:-status}" in
up)
	if [ "$(default_dev)" = "$IFACE" ]; then
		echo "warning: the only default route is on $IFACE; joining the drone will cut the internet (plug the Ethernet cable)." >&2
	fi
	sudo nmcli --wait 30 con up $PROFILE || { echo "could not join TELLO-FE18DB: is the drone powered on?" >&2; exit 1; }
	for i in $(seq 1 10); do ping -c 1 -W 1 $DRONE_IP >/dev/null 2>&1 && break; sleep 1; done
	status
	;;
down)
	sudo nmcli con down $PROFILE 2>/dev/null
	sudo nmcli dev connect $IFACE >/dev/null 2>&1 || true   # back to the home Wi-Fi (autoconnect profile)
	status
	;;
status) status ;;
*) echo "usage: $0 up|down|status" >&2; exit 2 ;;
esac
