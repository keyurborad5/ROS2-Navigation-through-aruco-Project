No special Instruction needed to run the program
1. Build the package -> colcon build --packages-select group17 
2. launch the turtlebot3 gazebo maz-> ros2 launch turtlebot3_gazebo maze.launch.py use_sim_time:=TrueD
3. run the node -> ros2 run group17 my_robot_node --ros-args -p use_sim_time:=True
4. Recommend not to increase the speed of the bot as it might affect its turning because I have used controls approch to commmand turning. Moreover I have only used propotional gain.
Hence if you increase the speedof bot then it may overturn while rotating.
5. I dont know why but the advanced logical camera was detecing object below the floor and it could be visulaise by RVIZ, hence i added '-' before z coordinate 
while broadcasting part. 
This is enough to see the bot running.
Additionally you can open RVIZ if you wish.
