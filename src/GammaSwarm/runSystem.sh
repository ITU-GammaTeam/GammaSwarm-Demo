roslaunch crazyswarm hover_swarm.launch &
sleep 1
roslaunch GammaSwarm allRun.launch &
sleep 30 
rosnode kill -all