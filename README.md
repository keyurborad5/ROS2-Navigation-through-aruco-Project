# ROS2 Navigation thorough information in Aruco Marker
Here I have uploaded the whole workspace, instead I should have uploaded only "src" folder of my workspace.
Nevertheless, you can still clone the repository and execute my package following the below instruction.

Author: Keyur Borad (Keyurborad5@gmail.com) 
### System requirement:
1. Ubuntu 20
2. ROS2 Galactic

### Simulation Video
![Video](mazenav.gif)

### Cloning and building the repository
```bash
  # Clones the repository in your system.
  git clone git@github.com:keyurborad5/ROS2-Navigation-through-aruco-Project.git
  # Delete the build , log and istall folder of the final_ws worksapce
  cd rwa3_ws
  # check all the directories, you should see build, install, log and src
  ls
  # remove unrequired folders also delete .vscode folder
  rm -rf build log install
  # check once again and only src folder shoul be present
  ls
  # Downlaod all dependencies before building it
  rosdep install --from-paths src -y --ignore-src
  # source the underlay and build the package
  source /opt/ros/galactic/setup.bash
  colcon build
  #Now source the overlay
  source install/setup.bash

```
### Launching the Simulation
```bash
  #Set the turtle bot model
  export TURTLEBOT3_MODEL=waffle
  # Launch the gazebo environment
   ros2 launch turtlebot3_gazebo maze.launch.py use_sim_time:=True
  #Launch my node
  ros2 run group17 my_robot_node --ros-args -p use_sim_time:=True
  # Additionally you can open RVIZ if you wish.
```
### NOTE
Recommend not to increase the speed of the bot as it might affect its turning because I have used controls approch to commmand turning. Moreover I have only used propotional gain.
Hence if you increase the speedof bot then it may overturn while rotating.
### Issue Resolved
I dont know why but the advanced logical camera was detecing object below the floor and it could be visulaise by RVIZ, hence i added '-' before z coordinate 
while broadcasting part. 

