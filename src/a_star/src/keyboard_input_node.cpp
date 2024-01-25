#include <ros/ros.h>
#include <iostream>
#include <regex>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv)
{
  setlocale(LC_ALL,"");
  // 初始化ROS节点
  ros::init(argc, argv, "input_recognition_node");
  ros::NodeHandle nh;
  ros::Publisher goalPoint_pub = nh.advertise<geometry_msgs::Point>("/goal_point",1,true);
  // 定义正则表达式模式
  std::regex takeoff_pattern("takeoff");
  std::regex takeland_pattern("takeland");
  std::regex quit_pattern("q");
  std::regex coordinate_pattern("([+-]?[0-9]+\\.?[0-9]*),([+-]?[0-9]+\\.?[0-9]*),([+-]?[0-9]+\\.?[0-9]*)");
  
  while(ros::ok()){
    // 获取终端输入字符串
    std::string input;
    std::cout << "请输入命令或坐标点 x,y,z(输入q以退出):";
    std::getline(std::cin, input);
    
    // 匹配输入内容
    std::smatch match;
    if (std::regex_match(input, match, takeoff_pattern))
    {
      ROS_INFO("识别到 takeoff 命令");
    }
    else if (std::regex_match(input, match, takeland_pattern))
    {
      ROS_INFO("识别到 takeland 命令");
    }
    else if (std::regex_match(input, match, coordinate_pattern))
    {
      
      geometry_msgs::Point goalPoint;
      float x = std::stof(match[1].str());
      float y = std::stof(match[2].str());
      float z = std::stof(match[3].str());
      goalPoint.x = x;
      goalPoint.y = y;
      goalPoint.z = z;
      ROS_INFO("识别到坐标点： x = %.2f, y = %.2f, z = %.2f", x, y, z);
      goalPoint_pub.publish(goalPoint);
    }
    else if(std::regex_match(input, match, quit_pattern))
    {
      break;
    }
    else
    {
      ROS_INFO("非法输入");
    }
  }
  
  return 0;
}