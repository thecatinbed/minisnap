

---

# Introduction

This project is based on ROS and Gazebo simulation. After the keyboard input node receives the "takeoff" command, it controls the quadrotors to take off. After takeoff, the user can input commands via the keyboard to set the goal point. The ROS nodes use A* algorithm and minisnap algorithm to generate obstacle-free paths and control the quadrotors's movement. Once the quadrotors reaches the goal point, the user can input new goal point for re-planning. To initiate the return, the user can input "takeland" via the keyboard.

---


# 1. Enviroments

Fundamental enviroment: Ubuntu20.04 + ROS noetic + gazebo11 + mavros + PX4&nbsp;
Partial dependency libraries: Eigen 3.3.7

# 2. How to use
* After starting mavros and PX4, run the "px4Ctrl.sh" script.
`sh scripts\px4ctrl.sh`
* Open a new terminal and launch the "astar.launch" file from the a_star package.`roslaunch a_star astar.launch`
* Open a new terminal and run the "keyboard_input_node" node from the a_star package.`rosrun a_star keyboard_input_node`
* Enter takeoff, takeland, and set_target commands in the keyboard input node to control the quadrotors' takeoff, landing, and goal point respectively.
## *attention*
* you should change the pcd path in "astar_node.cpp" file and compile again 
# 3. Demonstration of the effect

![本地绝对路径](../../gif/astar.gif)
