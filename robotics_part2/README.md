# robotics_part2 - office delivery robot (ROS 2 Jazzy, Gazebo Harmonic)

## Build
```bash
source /opt/ros/jazzy/setup.bash
cd robotics_part2            # folder that contains src/
rm -rf build install log     # removes stale installs from earlier experiments
colcon build --symlink-install
source install/setup.bash
```

## Fast DDS shared-memory errors
Stop all ROS/Gazebo processes, then in every terminal before launching:
```bash
ros2 daemon stop; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix office_delivery_robot)/share/office_delivery_robot/config/fastdds_no_shm.xml
```

## Run (one terminal each, source install/setup.bash in each)
```bash
ros2 launch office_delivery_robot office_world.launch.py
ros2 launch office_delivery_robot localization.launch.py
ros2 launch office_delivery_robot navigation.launch.py planner:=astar   # or planner:=dijkstra
```
Do not run `office_drive_node` or any `/cmd_vel` relay at the same time.

## Command path
Nav2 `/cmd_vel` (Twist) -> `ros_gz_bridge` (config/office_bridge.yaml) -> Gazebo DiffDrive plugin
(urdf/office_bot.urdf.xacro). There is no ros2_control / controller_manager in this project.
```bash
ros2 topic info /cmd_vel -v     # expect a Nav2 publisher and the ros_gz_bridge subscriber
gz topic -e -t /cmd_vel -n 1    # while a goal is active
```

## One navigation goal
Map frame = world frame + 1.0 m in x (static map->odom in localization.launch.py).
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

## Obstacles (while the robot is driving)
```bash
gz service -s /world/sc4_office_strip/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'sdf: "<sdf version=\"1.9\"><model name=\"test_box\"><static>true</static><pose>3 0 0.25 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>"'
gz service -s /world/sc4_office_strip/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "test_box" type: MODEL'
```

## Delivery route optimisation
```bash
ros2 run office_delivery_robot delivery_manager_node --ros-args -p use_sim_time:=true \
  -p "rooms:=['it','hr','accounting']" -p return_to_start:=true
```
Forbidden rooms are skipped. The remaining stops are re-ordered from the current pose after every
stop; a stop that fails is retried after the others (`max_attempts`).

## A* vs Dijkstra
The two configs differ only in `planner_server.GridBased.use_astar`. For each one, start navigation with
that planner, then:
```bash
ros2 run office_delivery_robot planner_benchmark_node --ros-args -p use_sim_time:=true \
  -p label:=astar -p output:=results/astar.csv      # label:=dijkstra for the other run
```

## Tests
```bash
colcon test --packages-select office_delivery_robot && colcon test-result --verbose
```
