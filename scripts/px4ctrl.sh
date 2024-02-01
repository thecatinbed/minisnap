rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0 
rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0 & sleep 2

rosservice call /mavros/cmd/arming "value: true"

roslaunch px4ctrl run_node.launch & sleep 1

