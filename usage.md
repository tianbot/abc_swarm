## Formation target

### long snake formation target
```bash
rosrun abc_swarm long_snake_formation_target_pub_with_tf.py  _target_frame:=tbmn_01/base_link _follower_frame:=follower _window_size:=500
```

### herringbone formation target
```bash
rosrun abc_swarm herringbone_formation_target_pub_with_tf.py  _target_frame:=tbmn_01/base_link _follower_frame:=follower _window_size:=500
```

## Path generator

```bash
roslaunch abc_swarm path_generator_with_clickpoints.launch use_rviz:=true fixed_frame:=world
```

## Path Tracking

### Path pure pursuit
```bash
rosrun abc_swarm pure_pursuit.py _robot_name:=tbmn_01/base_link _path_topic:=tbmn_01/path 
```

## Target Tracking

### Target PID tracker
```bash
rosrun abc_swarm pid_tracker.py _target_frame:=follower_1/target _follower_frame:=follower_1  _set_distance:=0.5
```

## Demo Scene

1. ### leader follower of turtlesim

```bash
roslaunch abc_swarm demo_turtlesim_leader_follower.launch
```
use `arrow` key to control the leader `turtle_1`, and watch the follower turtle follow the leader.

### Simulation

2. ### leader follower of Tianbot Mini
```bash
roslaunch abc_swarm demo_tbmn_leader_follower.launch
```

3. ### leader follower of Tianbot Mini with path tracking and herringbone formation
```bash
roslaunch abc_swarm demo_tbmn_leader_follower_path_tracking.launch
```

4. ### leader follower of Tianbot Mini with path tracking and long snake formation
```bash
rosrun abc_swarm long_snake_formation_target_pub_with_tf.py  _target_frame:=tbmn_01/base_link _follower_frame:=follower _window_size:=500
```

### Real world

- Terminal 1 for vrpn_client_node publishing `/tf`
```bash
roslaunch vrpn_client_ros sample.launch server:=192.168.0.42
```

- Terminal 2 for vrpn `tf` appending
```bash
roslaunch abc_swarm- vrpn_tf_append.launch 
```

- Terminal 3 for path generator
```bash
roslaunch abc_swarm path_generator_with_clickpoints.launch use_rviz:=true fixed_frame:=world
```

- Terminal 4 for `leader` path tracking
```bash
rosrun abc_swarm pure_pursuit.py _robot_name:=tbmn_01/base_link _path_topic:=tbmn_01/path 
```

- Terminal 5 for `followers` target publisher
```bash
rosrun abc_swarm long_snake_formation_target_pub_with_tf.py  _target_frame:=tbmn_01/base_link _follower_frame:=follower _window_size:=500
```

- Terminal 6 for `follower_1/target` tracking
```bash
roslaunch abc_swarm follower_pid_tracker.launch robot_name:=follower_1
```

- Terminal 7 for `follower_2/target` tracking
```bash
roslaunch abc_swarm follower_pid_tracker.launch robot_name:=follower_2
```
