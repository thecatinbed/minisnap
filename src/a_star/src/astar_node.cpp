#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include "a_star/Astar_searcher.h"
#include "a_star/backward.hpp"
#include "minisnap/planner.h"

using namespace std;
using namespace Eigen;


bool goal_set = false;          //true represents the goal is set 
bool start_plan = false;        //true represents the planning process has begin
bool takeoffCmd = false;        //true represents that state is "takeoff", while false represents "takeland"
string pcdPath;                 //the pcd file path
geometry_msgs::PoseStamped px4_pose;            //the true pose of quadrotors
sensor_msgs::PointCloud2 globalMap_pcd;         //the cloud map data in ROSMsg form
pcl::PointCloud<pcl::PointXYZ> cloudMap;        //the cloud map data read from pcd file
pcl::PointCloud<pcl::PointXYZ> cloudMap_vis;    //the cloud map data for visulization
AstarPathFinder *pathfinderPtr = NULL;

double _resolution, _inv_resolution, _cloud_margin;//resolution of map; inverse resolution of map; temporarily unused;

double _x_size, _y_size, _z_size;    

int _max_x_id, _max_y_id, _max_z_id;//in the world coordinate system (unit: meters), the length, width, and height of the entire point cloud map

Vector3d _start_pt,_goal_pt;//in the world coordinate system, the coordinates of the starting point 

Vector3d _map_lower, _map_upper;//in the world coordinate system, minimum and maximum coordinates of the X-axis, Y-axis, and Z-axis

string distanceType;//the method of calculate "h" value in "a star" algorithm

double _weight_a,_weight_b;//the weight of "g" and "h" in calculate "f" value

void Load_map(){
    pcl::io::loadPCDFile(pcdPath,cloudMap);//load cloud point map data from pcd file
    assert((int)cloudMap.points.size() != 0);
}

void Init_Astar_map(AstarPathFinder *pathfinderPtr){
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    pathfinderPtr -> initGridMap(_resolution,_map_lower,_map_upper,_max_x_id,_max_y_id,_max_z_id);

    pcl::PointXYZ point;
    for(int i = 0;i < (int)cloudMap.points.size();i++){
        point = cloudMap.points[i];
        pathfinderPtr->setObs(point.x,point.y,point.z);// set the obstacle in grid map
    }
}

void visualize_map(AstarPathFinder *pathfinderPtr){
    pcl::PointXYZ point;
    for(int i = 0;i < (int)cloudMap.points.size();i++){
        point = cloudMap.points[i];
        
        Vector3d cor_round = pathfinderPtr -> coordRounding(Vector3d(point.x, point.y, point.z));
        point.x = cor_round(0);
        point.y = cor_round(1);
        point.z = cor_round(2);
        cloudMap_vis.points.push_back(point);
    }
    cloudMap_vis.width = cloudMap_vis.points.size();
    cloudMap_vis.height = 1;
    cloudMap_vis.is_dense = true;

    pcl::toROSMsg(cloudMap_vis, globalMap_pcd);//translate the map's data type into ROSMsg for publishing map, visulizing the map in rviz
    globalMap_pcd.header.frame_id = "world";
}

void init_marker(visualization_msgs::Marker* marker){
    //Set the message type to LINE-STRIP
    marker->type = visualization_msgs::Marker::LINE_STRIP;  
    //Set coordinate system
    marker->header.frame_id = "world";                      
    marker->header.stamp = ros::Time::now();
    marker->ns = "";
    marker->id = 0;
    marker->action = visualization_msgs::Marker::ADD;
    marker->pose.orientation.w = 1.0;
    // Set the width of the line
    marker->scale.x = 0.02;
    marker->scale.y = 0.02;
    marker->scale.z = 0.02;
    // Set the color of the line
    marker->color.r = 0.0;
    marker->color.g = 1.0;
    marker->color.b = 1.0;
    marker->color.a = 1.0;
    // Clear the points in the list
    marker->points.clear();
}

void visGridPath( vector<Vector3d> nodes, ros::Publisher _grid_path_vis_pub)
{   
    visualization_msgs::Marker node_vis; 
    geometry_msgs::Point point;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

   
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;
    

    node_vis.scale.x = 0.2 / 5;
    node_vis.scale.y = 0.2 / 5;
    node_vis.scale.z = 0.2 / 5;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void goalPointCallback(const geometry_msgs::Point::ConstPtr &point){
    if(point->x >= _map_lower(0) && point->x <= _map_upper(0) &&
       point->y >= _map_lower(1) && point->y <= _map_upper(1) &&
       point->z >= _map_lower(2) && point->z <= _map_upper(2))
    {
        _goal_pt << point->x,
                     point->y,
                     point->z;
        goal_set = true;
    }
    else
    {
        ROS_INFO("goal_point data is illegal");
    }
    
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose){
    px4_pose = *pose;
}

void takeoffCallback(const std_msgs::Bool::ConstPtr &cmd){
    takeoffCmd = cmd -> data;
}

int main(int argc, char** argv){

    int cmd_state = 0;
    int landing_state = 0;

    ros::init(argc,argv,"astar_node");
    ros::NodeHandle nh;
    planner myplanner;

    visualization_msgs::Marker real_trajectory_marker;
    vector<quadrotor_msgs::PositionCommand> tra;

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_map",1);// visulize the cloud map 
    ros::Publisher _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("/astar_path_marker", 1);// visulize the A* path
    ros::Publisher tra_generation_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);  // publish the control command to px4Ctrl
    ros::Publisher desTrajectory_pub = nh.advertise<visualization_msgs::Marker>("/desire_trajectory",1,true);// the trajectory planned by "minisnap" algorithm
    ros::Publisher realTrajectory_pub = nh.advertise<visualization_msgs::Marker>("/real_trajectory",1,true);// the really trajectory of quadrotors 

    ros::Subscriber goalPoint_sub = nh.subscribe("/goal_point",1, goalPointCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber takeoffCmd_sub = nh.subscribe<std_msgs::Bool>("/keyboard/takeoff_cmd", 1, takeoffCallback);

    nh.param("pcd_path", pcdPath, string("/home/mm/catkin_ws/src/a_star/pcd/map.pcd"));
    nh.param("heuristic/distance",distanceType, string("euclidean"));
    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size",        _x_size, 10.0);
    nh.param("map/y_size",        _y_size, 10.0);
    nh.param("map/z_size",        _z_size, 4.0 );
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);
    nh.param("weight/a",_weight_a,1.0);
    nh.param("weight/b",_weight_b,1.0);

    pathfinderPtr = new AstarPathFinder(distanceType,_weight_a,_weight_b);
    
    Load_map();
    Init_Astar_map(pathfinderPtr);
    visualize_map(pathfinderPtr);

    while(map_pub.getNumSubscribers()<1);//waitting for rviz to subscribe
    ros::Duration(3).sleep();            //To prevent RViz from not receiving the map, a delay can be introduced.
    map_pub.publish(globalMap_pcd);      //publish the map data in ROSMsg
    init_marker(&real_trajectory_marker);

    while(ros::ok()){
        switch (cmd_state)
        {
        case 0: //waitting for takeoff                                                 
            if(!start_plan && takeoffCmd){
                myplanner.dot_num = 2;
                myplanner.time_everytraj.resize(1);
                myplanner.time_everytraj(0) = 10;
                myplanner.route.resize(2, 3);
                myplanner.route(0,0) = px4_pose.pose.position.x;
                myplanner.route(0,1) = px4_pose.pose.position.y;
                myplanner.route(0,2) = px4_pose.pose.position.z;

                myplanner.route(1,0) = px4_pose.pose.position.x;
                myplanner.route(1,1) = px4_pose.pose.position.y;
                myplanner.route(1,2) = 2;
                myplanner.poly_coeff = myplanner.getcoeff();
                cout << myplanner.poly_coeff << endl;
                tra = myplanner.get_trajectory(); // to discretize a trajectory and obtain a series of trajectory points
                myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                start_plan = true;                  // the planning process has begin
            }
            if(start_plan){
                static int i = 0;
                if( i < (int)tra.size()){
                    tra_generation_pub.publish(tra[i]);                 // publish the control command of takeoff
                    geometry_msgs::Point temp_point;
                    temp_point.x = px4_pose.pose.position.x;
                    temp_point.y = px4_pose.pose.position.y;
                    temp_point.z = px4_pose.pose.position.z;
                    real_trajectory_marker.points.push_back(temp_point);
                    realTrajectory_pub.publish(real_trajectory_marker);
                    i++;
                }else{
                    i = 0;
                    start_plan = false;
                    std::cout<< "takeoff complete" <<endl;
                    cmd_state = 1;
                }
            }
            break;
        
        case 1: // waitting for goal point
            if(!start_plan && !takeoffCmd){
                cmd_state = 2;  // translate the state into "2" to takeland  
                break;
            }
            if(!start_plan && goal_set){
                goal_set = false;
                if(pathfinderPtr->isReachable(_goal_pt)){
                    pathfinderPtr -> AstarGraphSearch(Vector3d(px4_pose.pose.position.x,px4_pose.pose.position.y,px4_pose.pose.position.z), _goal_pt);
                    // obtain a set of path points and a closed set using the A* algorithm
                    auto grid_path     = pathfinderPtr->getPath();
                    auto visited_nodes = pathfinderPtr->getVisitedNodes();

                    // visulize the path points
                    visGridPath(grid_path, _grid_path_vis_pub);
                    // reset the A* algorithm for easy reuse in subsequent calls
                    pathfinderPtr->resetUsedGrids();

                    // the A* algorithm has completed, and start using the Minisnap algorithm to plan the trajectory
                    myplanner.dot_num = ((int)grid_path.size()) + 1 ;
                    myplanner.time_everytraj.resize(myplanner.dot_num - 1);
                    myplanner.route.resize(myplanner.dot_num, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;
                    Vector3d node;
                    geometry_msgs::Point point;
                    for(int i = 0; i < ((int)grid_path.size()); i++){
                        node = grid_path[i];
                        myplanner.time_everytraj(i) = 1;// set the time duration for each segment of the trajectory
                        myplanner.route(i+1, 0) = node(0);
                        myplanner.route(i+1, 1) = node(1);
                        myplanner.route(i+1, 2) = node(2);
                    }
                    myplanner.poly_coeff = myplanner.getcoeff();
                    tra = myplanner.get_trajectory(); 
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    ROS_INFO("the end point is not reachable");
                }
            }
            if(start_plan){
                goal_set = false;
                static int i = 0;
                if( i < (int)tra.size()){
                    tra_generation_pub.publish(tra[i]);
                    geometry_msgs::Point temp_point;
                    temp_point.x = px4_pose.pose.position.x;
                    temp_point.y = px4_pose.pose.position.y;
                    temp_point.z = px4_pose.pose.position.z;
                    real_trajectory_marker.points.push_back(temp_point);
                    realTrajectory_pub.publish(real_trajectory_marker);// visulize the really trajectory of quadrotors
                    i++;
                }else{
                    i = 0;
                    start_plan = false;
                    std::cout<< "publish complete" <<endl;
                }
            }
            break;
        
        case 2: // quadrotors is ready to land
            switch (landing_state)
            {
            case 0: //1. ascending to a specific altitude
                if(!start_plan){
                    myplanner.dot_num = 2;
                    myplanner.time_everytraj.resize(1);
                    myplanner.time_everytraj(0) = 10;
                    myplanner.route.resize(2, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;

                    myplanner.route(1,0) = px4_pose.pose.position.x;
                    myplanner.route(1,1) = px4_pose.pose.position.y;
                    myplanner.route(1,2) = 5;
                    myplanner.poly_coeff = myplanner.getcoeff();
                    cout << myplanner.poly_coeff << endl;
                    tra = myplanner.get_trajectory(); 
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    static int i = 0;
                    if( i < (int)tra.size()){
                        tra_generation_pub.publish(tra[i]);
                        geometry_msgs::Point temp_point;
                        temp_point.x = px4_pose.pose.position.x;
                        temp_point.y = px4_pose.pose.position.y;
                        temp_point.z = px4_pose.pose.position.z;
                        real_trajectory_marker.points.push_back(temp_point);
                        realTrajectory_pub.publish(real_trajectory_marker);
                        i++;
                    }else{
                        i = 0;
                        start_plan = false;
                        std::cout<< "landing takeoff complete" <<endl;
                        landing_state = 1;
                    }
                }
                break;
            
            case 1: //2. flying horizontally back above the starting point
                if(!start_plan){
                    myplanner.dot_num = 2;
                    myplanner.time_everytraj.resize(1);
                    myplanner.time_everytraj(0) = 10;
                    myplanner.route.resize(2, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;

                    myplanner.route(1,0) = 0;
                    myplanner.route(1,1) = 0;
                    myplanner.route(1,2) = px4_pose.pose.position.z;
                    myplanner.poly_coeff = myplanner.getcoeff();
                    cout << myplanner.poly_coeff << endl;
                    tra = myplanner.get_trajectory(); 
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    static int i = 0;
                    if( i < (int)tra.size()){
                        tra_generation_pub.publish(tra[i]);
                        geometry_msgs::Point temp_point;
                        temp_point.x = px4_pose.pose.position.x;
                        temp_point.y = px4_pose.pose.position.y;
                        temp_point.z = px4_pose.pose.position.z;
                        real_trajectory_marker.points.push_back(temp_point);
                        realTrajectory_pub.publish(real_trajectory_marker);
                        i++;
                    }else{
                        i = 0;
                        start_plan = false;
                        std::cout<< "landing back complete" <<endl;
                        landing_state = 2;
                    }
                }
                break;
            case 2: //3. descending and landing at the starting point
                if(!start_plan){
                    myplanner.dot_num = 2;
                    myplanner.time_everytraj.resize(1);
                    myplanner.time_everytraj(0) = 10;
                    myplanner.route.resize(2, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;

                    myplanner.route(1,0) = 0;
                    myplanner.route(1,1) = 0;
                    myplanner.route(1,2) = 0;
                    myplanner.poly_coeff = myplanner.getcoeff();
                    cout << myplanner.poly_coeff << endl;
                    tra = myplanner.get_trajectory(); 
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    static int i = 0;
                    if( i < (int)tra.size()){
                        tra_generation_pub.publish(tra[i]);
                        geometry_msgs::Point temp_point;
                        temp_point.x = px4_pose.pose.position.x;
                        temp_point.y = px4_pose.pose.position.y;
                        temp_point.z = px4_pose.pose.position.z;
                        real_trajectory_marker.points.push_back(temp_point);
                        realTrajectory_pub.publish(real_trajectory_marker);
                        i++;
                    }else{
                        i = 0;
                        start_plan = false;
                        landing_state = 0;
                        cmd_state = 0;
                        goal_set = false;
                        std::cout<< "landing desending complete" <<endl;
                    }
                }
                break;
            }
            
            break;
        }
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    delete pathfinderPtr;
    return 0;
}