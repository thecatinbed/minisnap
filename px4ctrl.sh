roslaunch px4ctrl run_node.launch & sleep 5

#rostopic pub /takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1" & sleep 5

#rosrun rqt_reconfigure rqt_reconfigure & sleep 1

rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0 

rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0 & sleep 1

rosservice call /mavros/cmd/arming "value:true" success: True result: 0
sleep 1
rosparam set /px4ctrl/mode_bool true 
sleep 1
rosparam set /px4ctrl/cmd_bool true
