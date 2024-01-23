roslaunch px4ctrl run_node.launch & sleep 5

#rostopic pub /takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1" & sleep 5

rosrun rqt_reconfigure rqt_reconfigure & sleep 1

rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0 

rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0 
