# pixhawk-android-gps-follower
## GPS Connection instructions (to connect android phone gps and pixhawk gps to the same network)

1. Ensure they are on the same wifi network. Check by running `ifconfig` and checking for `inet` in `wlp2s0`.
2. `export ROS_MASTER_URI=http://146.179.193.161:11311/` and substitute `146.179.193.161` with the network address. Here the above ip address corresponds to eduroam at Imperial College
3. Launch mavros `roslaunch mavros px4.launch`
4. Launch the ROS driver app on your phone and enter the same network `http://146.179.193.161:11311/`
5. `rostopic list` and check that both mavros and your phone are connected. 

## Instructions to start ros node
1. Build workspace in catkin_ws
2. Source your devel/setup.bash (or setup.zsh)
3. `rosrun follower follower_node`