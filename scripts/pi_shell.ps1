# Attach to the Pi's shared terminal session (the Pi's Claude Code runs inside `tmux new -s claude`).
#   scripts\pi_shell.ps1 100.x.y.z [user]      -> read-write attach; press Ctrl+B then D to detach without stopping it
param([string]$PiAddress = "100.112.7.35", [string]$User = "quiquepi")   # defaults: the Pi over Tailscale (quiquepi); use 192.168.137.10 over the cable
ssh -t "$User@$PiAddress" "tmux attach -t claude || tmux new -s claude"
