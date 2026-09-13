#!/bin/bash
# boot_check.sh - after a reboot: fixed eth0 address, Tailscale online, Funnel -> 8080, drone-watch waiting.
# Run it over SSH from the laptop: ssh quiquepi@100.112.7.35 tello-race/scripts/pi/boot_check.sh
set -u
ok() { printf '%-4s %s\n' "$1" "$2"; }

echo "uptime: $(uptime -p) (since $(uptime -s))"
a=$(ip -4 -br addr show eth0 | awk '{print $3}')
[ "$a" = 192.168.137.10/24 ] && ok OK "eth0 $a" || ok FAIL "eth0 $a (expected 192.168.137.10/24)"
d=$(ip route show default | head -1)
case "$d" in *"dev eth0"*) ok OK "default route: $d" ;; *) ok FAIL "default route: $d" ;; esac
ts=$(tailscale status --json 2>/dev/null)
st=$(jq -r '.BackendState' <<<"$ts"); on=$(jq -r '.Self.Online' <<<"$ts"); name=$(jq -r '.Self.DNSName' <<<"$ts" | sed 's/\.$//')
[ "$st" = Running ] && [ "$on" = true ] && ok OK "tailscale $st online=$on $name $(tailscale ip -4)" || ok FAIL "tailscale $st online=$on"
if tailscale funnel status 2>/dev/null | grep -q '127.0.0.1:8080'; then ok OK "funnel https://$name/ -> 127.0.0.1:8080"
else ok FAIL "funnel not serving 8080: $(tailscale funnel status 2>&1 | head -1)"; fi
code=$(curl -s -m 15 -o /dev/null -w '%{http_code}' "https://$name/api/status?token=watch")
ok INFO "public /api/status?token=watch -> HTTP $code (502 is expected while nothing listens on 8080)"
ss -ltn | grep -q ':8080 ' && ok INFO "port 8080 listening" || ok INFO "port 8080 not listening"
w=$(systemctl --user is-active drone-watch); e=$(systemctl --user is-enabled drone-watch)
[ "$w" = active ] && ok OK "drone-watch $w/$e" || ok FAIL "drone-watch $w/$e"
nmcli -g ipv4.never-default con show tello | grep -qx yes && ok OK "tello profile ipv4.never-default yes" || ok FAIL "tello profile never-default not set"
echo "drone-watch log:"; journalctl -b -t drone-watch --no-pager -o short | tail -5 | sed 's/^/    /'
echo "other services: vnc-screens $(systemctl --user is-active vnc-screens), wayvnc $(systemctl is-active wayvnc)"
