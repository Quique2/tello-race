#!/bin/bash
# Install the "screens follow the VNC connection" pieces on the Pi (see docs/HANDOFF.md).
set -eu
D=$(cd "$(dirname "$0")" && pwd)
install -Dm755 "$D/vnc-screens.sh" ~/.local/bin/vnc-screens.sh
install -Dm644 "$D/vnc-screens.service" ~/.config/systemd/user/vnc-screens.service
install -Dm644 "$D/kanshi.config" ~/.config/kanshi/config
# The Control Centre's Screens panel rewrites both kanshi files (user and login greeter) when a layout is
# saved; shipping the local layout to both keeps DSI-2 on at boot even after that.
sudo install -Dm644 "$D/kanshi.config" /etc/xdg/labwc-greeter/config.kanshi
sudo install -Dm644 "$D/wayvnc-control-socket-acl.conf" /etc/systemd/system/wayvnc.service.d/control-socket-acl.conf
sudo systemctl daemon-reload
[ -S /tmp/wayvnc/wayvncctl.sock ] && sudo setfacl -m "u:$(id -un):rwx" /tmp/wayvnc/wayvncctl.sock
systemctl --user daemon-reload
echo "installed; enable with: systemctl --user enable --now vnc-screens"
