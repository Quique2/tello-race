# Run the mission in the simulator with a live window (random track). Extra args are passed through,
# e.g. scripts\run_sim.ps1 --track example   or   scripts\run_sim.ps1 --episodes 20 --headless --no-realtime
& "C:\dev\build\tello-race\tello_race.exe" sim --track random --seed 1 @args
