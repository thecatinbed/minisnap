#include <ros/ros.h>
#include <minisnap/planner.h>

geometry_msgs::PoseStamped px4_pose;      //actual pose data
geometry_msgs::PoseStamped rec_goal_point;//the data of the goal point provided by "3D nav goal" 
geometry_msgs::PointStamped goal_point;   //the goal point, which is similar to "rec_goal_point", but they have different data type 
bool clicked = false;
bool start_plan = false;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pt){
    px4_pose = *pt;                  //save the pose of quarotors
}
void clicked_cb(const geometry_msgs::PoseStamped::ConstPtr &point){
    clicked = true;                 //a click event has occurred
    rec_goal_point = *point;        //save the goal point given by rviz
}

int main(int argc, char** argv){
    ros::init(argc,argv,"trajec_node");
    planner myplanner;
	ros::NodeHandle nh;
    nav_msgs::Path real_path;
    geometry_msgs::PoseStamped pose;
    std::vector<quadrotor_msgs::PositionCommand> tra;
    std::vector<geometry_msgs::PoseStamped> goal_point_list;

	ros::Publisher tra_generation_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10); //control command to "px4Ctrl"
    ros::Publisher real_trajectory_pub = nh.advertise<nav_msgs::Path>("/real_trajectory", 10, true);            //visulize the real trajectory 
    ros::Publisher goal_point_pub = nh.advertise<geometry_msgs::PointStamped>("/goal_point", 10, true);         //visulize the goal points
    ros::Publisher despath_pub = nh.advertise<nav_msgs::Path>("/desire_trajectory", 10, true);                  //visulize the true trajectory
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);//subscribe the quadrotors's pose
    ros::Subscriber clickedpoint_sub = nh.subscribe<geometry_msgs::PoseStamped>("/rviz_goal_point", 1, clicked_cb); //subscribe the point given by rviz's "3d nav goal"
    ros::Rate rate(100);        

    myplanner.dot_num = 0;      //init the number of goal point

    real_path.header.frame_id = "world";
    real_path.header.stamp = ros::Time::now();
    while(ros::ok()){
        if(!start_plan && clicked){                     //the planning stopped and a click event has occured
            clicked = false;
            goal_point_list.push_back(rec_goal_point);  //push the goal point into list
            goal_point.header.frame_id = "world";
            goal_point.header.stamp = ros::Time::now();
            goal_point.point.x = rec_goal_point.pose.position.x;
            goal_point.point.y = rec_goal_point.pose.position.y;
            goal_point.point.z = rec_goal_point.pose.position.z;
            goal_point_pub.publish(goal_point);     //visulize the goal point
            if(goal_point_list.size() >= 3){        //the trajectory planning process begins when the number of set goal points is greater than or equal to 3
                start_plan = true;                  //set the flag that represent the trajectory planning has begin
                myplanner.time_everytraj.resize(goal_point_list.size());    //resize the number of time intervals
                for(int i = 0;i < goal_point_list.size() ; i++){
                    myplanner.time_everytraj( i ) = 3.0;                    //set the value of every time interval
                }
                myplanner.dot_num = goal_point_list.size() + 1;             //the number of total points equals to goal points plus one
                myplanner.route.resize(myplanner.dot_num,3);                //route stores all the points
                myplanner.route(0,0) = px4_pose.pose.position.x;            //the first point is the current position of quadrotors 
                myplanner.route(0,1) = px4_pose.pose.position.y;
                myplanner.route(0,2) = px4_pose.pose.position.z;
                for(int i=1; i<=goal_point_list.size(); i++){               //set the other points
                    myplanner.route(i, 0) = goal_point_list[i-1].pose.position.x;
                    myplanner.route(i, 1) = goal_point_list[i-1].pose.position.y;
                    myplanner.route(i, 2) = goal_point_list[i-1].pose.position.z;
                }
                myplanner.poly_coeff = myplanner.getcoeff();                //obtaining the polynomial matrix of a trajectory
                tra = myplanner.get_trajectory();                           //discretize the trajectory to obtain control commands.
                myplanner.draw_desire_trajectory(despath_pub);              //visulize the desire trajectory
            }
        }
        if(start_plan){                             //the planning process has begin
            static int i = 0;
            if( i < tra.size()){
                tra_generation_pub.publish(tra[i]);//publish the control command to "px4Ctrl"
                i++;
            }else{
                i = 0;
                start_plan = false;                 //planning process is over, reset some variables
                goal_point_list.clear();
            }
        }
        pose.pose.position.x = px4_pose.pose.position.x;    
        pose.pose.position.y = px4_pose.pose.position.y;
        pose.pose.position.z = px4_pose.pose.position.z;
        real_path.poses.push_back(pose);
        real_trajectory_pub.publish(real_path);     //visulize the really trajectory

        rate.sleep();
        ros::spinOnce();
    }
	return 0;
}
