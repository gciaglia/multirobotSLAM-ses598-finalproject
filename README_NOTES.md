# Terminal 1: World Launch

cd ~/ros2_ws
ln -s /home/gianna/Desktop/SES598/FinalProject/multirobotSLAM-ses598-finalproject/multirobot_bringup src/multirobot_bringup
colcon build --packages-select multirobot_bringup --symlink-install


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

