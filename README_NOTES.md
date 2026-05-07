# Quick start: bring up everything in one terminal

Single launch — Gazebo, all robots, per-robot SLAM, and the map merger
(uses the same spawn poses for the spawner and the merger so they can't drift):
```bash
cd ~/Desktop/SES598/FinalProject/ros2_ws
source install/setup.bash
ros2 launch multirobot_bringup multirobot.launch.py
```
Override defaults, e.g. for 3 robots:
```bash
ros2 launch multirobot_bringup multirobot.launch.py \
    robot_namespaces:='[robot1, robot2, robot3]' \
    robot_initial_poses:='[0.0, 0.0, 2.0, 0.5, -2.0, 0.5]'
```
Then in a separate terminal: teleop one robot, and rviz2 to visualize.

---

# Terminal 1: World Launch

cd ~/ros2_ws
ln -s /home/gianna/Desktop/SES598/FinalProject/multirobotSLAM-ses598-finalproject/multirobot_bringup src/multirobot_bringup
colcon build --packages-select multirobot_bringup --symlink-install

Leave the world field blank to see the martian world.
```bash
source install/setup.bash
ros2 launch multirobot_bringup simulation_launch.py world:=/opt/ros/jazzy/share/nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro 
```

#this is a built in word for the Nav2 ROS2 package

Other worlds:
Warehouse ->
```bash
ros2 launch multirobot_bringup simulation_launch.py world:=/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/warehouse.sdf
```

# Terminal 2:
```bash
ros2 launch multirobot_bringup spawn_robot.launch.py namespace:=robot1 x_pose:=0.0 y_pose:=0.0 z_pose:=0.5
```
To launch second bot manually: 
```bash
cd ~/Desktop/SES598/FinalProject/ros2_ws
source install/setup.bash
ros2 launch multirobot_bringup spawn_robot.launch.py namespace:=robot2 robot_name:=turtlebot3_waffle_2 x_pose:=2.0 y_pose:=0.5
```

# Terminal 3:
```bash
ros2 launch multirobot_bringup slam_per_robot.launch.py namespace:=robot1
```

# Terminal 4:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

# Terminal 5: 
```bash
rviz2
```
 ->Need to select "map" node to see currently mapping

# Terminal 6: Map Merger
Fuses each robot's /<ns>/map into a single /map_merged (frame: map_merged).
Spawn offsets must match the x_pose/y_pose used in spawn_robot.launch.py.
```bash
ros2 launch multirobot_bringup map_merger.launch.py \
    robot_namespaces:='[robot1, robot2]' \
    robot_initial_poses:='[0.0, 0.0, 2.0, 0.5]'
```
View in RViz: add an OccupancyGrid display on /map_merged, set Fixed Frame to `map_merged`.


------------------------------------------
# Warehouses

smaller warehouse:
cd ~/Desktop/SES598/FinalProject/ros2_ws
source install/setup.bash
ros2 launch multirobot_bringup multirobot.launch.py \
    world:=/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/depot.sdf \
    selection_strategy:=random

larger warehouse: 
cd ~/Desktop/SES598/FinalProject/ros2_ws
source install/setup.bash
ros2 launch multirobot_bringup multirobot.launch.py \
    world:=/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/warehouse.sdf \
    robot_initial_poses:='[-2.0, 0.0, -2.0, 1.0]' \
    selection_strategy:=random


