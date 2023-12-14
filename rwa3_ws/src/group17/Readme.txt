No special Instruction needed to run the program
1. Build the package -> colcon build --packages-select group17 
2. launch the turtlebot3 gazebo maz-> ros2 launch turtlebot3_gazebo maze.launch.py use_sim_time:=TrueD
3. run the node -> ros2 run group17 my_robot_node --ros-args -p use_sim_time:=True
This is enough to see the bot running.
Additionally you can open RVIZ if you wish.