# pixhawk-android-gps-follower

## To Run
- Build: `catkin build follower`
- Test: `roslaunch follower test_kf.launch` - launches Kalman filter node and dependencies

## Running Locally
### Connect phone and pixhawk to same network
1. Ensure they are on the same wifi network. Check by running `ifconfig` and checking for `inet` in `wlp2s0`.
2. `export ROS_MASTER_URI=http://192.168.0.25:11311/` and substitute `192.168.0.25` with the network (inet) address.
3. Launch mavros `roslaunch mavros px4.launch`
4. Launch the ROS driver app on your phone and enter the same network `http://192.168.0.25:11311/`
5. `rostopic list` and check that both mavros and your phone are connected. 

### Instructions to start ros node
1. Build workspace in catkin_ws
2. Source your devel/setup.bash (or setup.zsh)
3. `rosrun follower follower_node`

## Running Remotely (using port forwarding into a webserver + rosbridgecpp)
### SSH tunnelling (port forwarding) into serveo.net
1. `ssh -R 80:localhost:9090 serveo.net` forwards traffic from localhost:9090 to <given-name>.serveo.net:80
2. Run rosbridge and mavros launch scripts
3. Have a subscriber listen to given-name.serveo.net:80 and publish that data to a local rostopic
4. Run cpp follower node as usual

## Geodetic library usage
- Requires catkin_simple to be cloned into workspace src
- Build normally, then run echoer, set_reference and conversion_node respectively

## Library dependencies
- `imu_tools` for Madgwick filter in performing GPS + IMU sensor fusion
- `sensorfusion` created package, referenced from: https://github.com/maddevsio/mad-location-manager
- `rosbridgecpp` for rosbridge over websockets implementation

## Other Library dependencies
- catkin_simple: https://github.com/catkin/catkin_simple