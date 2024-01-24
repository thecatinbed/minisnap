#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv){
    int point_list[] = {0.3,-0.1,0.9,0.5,-0.1,0.7,0.7,-0.3,0.5,0.9,-0.3,0.3};
    geometry_msgs::Point point;
    ros::init(argc,argv,"pub_point_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/astar_path_point",10);

    for(int i=0;i<4;i++){
        point.x = point_list[3*i];
        point.y = point_list[3*i+1];
        point.z = point_list[3*i+2];
        pub.publish(point);
        ros::Rate(10).sleep();
    }

    return 0;
}