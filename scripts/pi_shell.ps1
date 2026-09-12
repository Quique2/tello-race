# Attach to the Pi's shared terminal session (the Pi's Claude Code runs inside `tmux new -s claude`).
#   scripts\pi_shell.ps1 100.x.y.z [user]      -> read-write attach; press Ctrl+B then D to detach without stopping it
param([Parameter(Mandatory=$true)][string]$PiAddress, [string]$User = "pi")
ssh -t "$User@$PiAddress" "tmux attach -t claude || tmux new -s claude"
