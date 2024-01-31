#include "minisnap/planner.h"

geometry_msgs::PoseStamped px4_pose;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pt)
{
    px4_pose = *pt;
    // std::cout << "px4_pose:  " << px4_pose << std::endl;
}
/*
*获取配置文件参数
*/
void planner::getparam(void)
{
    geometry_msgs::PointStamped point;
    ros::Rate rate(10);
    dot_num = 0;
    XmlRpc::XmlRpcValue param_list;    //严格限定数据类型
    XmlRpc::XmlRpcValue time_list;
    n.getParam("pose", param_list);   //提取约束点参数
    n.getParam("ts", time_list);    //提取时间参数
    ros::Publisher point_pub = n.advertise<geometry_msgs::PointStamped>("/pose_point",20);
    dot_num = param_list.size() / 3;
    route.resize (dot_num, 3  );                           //不resize 会报错
    for (int i = 0 ; i < param_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = param_list[i];
        route(  i % dot_num , i / dot_num) = double(value);
    }
    std::cout << "route:"<< std::endl<< route << std::endl;
    
    n.getParam("ts", time_list);    //提取时间参数
    time_everytraj.resize(time_list.size());      //不resize 会报错
    for ( int i = 0 ; i < time_list.size() ; i++ )
    {
        XmlRpc::XmlRpcValue value = time_list[i];
        time_everytraj( i ) = double(value);
    }
    for(int i = 0; i < dot_num; i++){
        point.header.stamp = ros::Time::now();
        point.header.frame_id = "world";
        point.point.x = route(i, 0);
        point.point.y = route(i, 1);
        point.point.z = route(i, 2);
        point_pub.publish(point);
        rate.sleep();
    }
    std::cout<<"route:"<<endl<<route<<endl;
    std::cout<<"time:"<<endl<<time_everytraj<<endl;
    std::cout<<"dot_num:"<<endl<<dot_num<<endl;
}

/*
*获取路径参数矩阵
*/
Eigen::MatrixXd planner::getcoeff(void)
{
    static Eigen::MatrixXd polycoeff;
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
    Minisnap TrajectoryGeneratorTool;
    polycoeff = TrajectoryGeneratorTool.SolveQPClosedForm(mode, route, vel, acc, time_everytraj);
    return polycoeff; 
}


/*!
 * 求解第k个轨迹段t时刻对应的位置
 * @param polyCoeff 多项式系数矩阵
 * @param k 轨迹段序号
 * @param t 时刻
 * @return [x,y,z]^T
 */
Eigen::Vector3d planner::getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t) 
{
    Eigen::Vector3d pt;
    poly_coeff_num= 2 * mode;
    // std::cout << "poly_coeff_num:" << poly_coeff_num << std::endl;             //正确

    for (int dim = 0; dim < 3; dim++) 
    {

        Eigen::VectorXd coeff;
        coeff.resize(poly_coeff_num);
        
        coeff = (polyCoeff.row(k)).segment(dim * poly_coeff_num,  poly_coeff_num);
        Eigen::VectorXd times = Eigen::VectorXd::Zero(poly_coeff_num);
        for (int j = 0; j < poly_coeff_num; j++)
            if (j == 0)
                times(j) = 1.0;
            else
                times(j) = pow(t, j);
        double temp_pose = 0.0;
        for (int i = 0; i < times.rows(); ++i) 
        {
            temp_pose = temp_pose + coeff(i) * times(times.rows() - i - 1);
        }
        pt(dim) = temp_pose;
    }

    //std::cout << "pose:" << pt << std::endl;       //获取位置成功
    return pt;
}
/*!
 * 求解第k个轨迹段t时刻对应的速度
 * @param polyCoeff 多项式系数矩阵
 * @param k 轨迹段序号
 * @param t 时刻
 * @return [vx,vy,vz]^T
 */
Eigen::Vector3d planner::getVelPoly(Eigen::MatrixXd polyCoeff, int k, double t) 
{
    Eigen::Vector3d vt;
    poly_coeff_num= 2 * mode;
    // std::cout << "poly_coeff_num:" << poly_coeff_num << std::endl;             //正确

    for (int dim = 0; dim < 3; dim++) 
    {

        Eigen::VectorXd coeff;
        coeff.resize(poly_coeff_num);
        
        coeff = (polyCoeff.row(k)).segment(dim * poly_coeff_num, poly_coeff_num);//k行代表第k段轨迹，
        Eigen::VectorXd times = Eigen::VectorXd::Zero(poly_coeff_num);
        for (int j = 0; j < poly_coeff_num; j++)
            if (j == 0)
                times(j) = 0;
            else if(j == 1)
                times(j) = 1;
            else
                times(j) = j * pow(t, j-1);
        double temp_vel = 0.0;
        for (int i = 0; i < times.rows(); ++i) 
        {
            temp_vel = temp_vel + coeff(i) * times(times.rows() - i - 1);
        }
        vt(dim) = temp_vel;
    }

    //std::cout << "vel:" << vt << std::endl;       //获取速度成功
    return vt;
}
/*!
 * 求解第k个轨迹段t时刻对应的加速度
 * @param polyCoeff 多项式系数矩阵
 * @param k 轨迹段序号
 * @param t 时刻
 * @return [ax,ay,az]^T
 */
Eigen::Vector3d planner::getAccPoly(Eigen::MatrixXd polyCoeff, int k, double t) 
{
    Eigen::Vector3d acct;
    poly_coeff_num= 2 * mode;
    // std::cout << "poly_coeff_num:" << poly_coeff_num << std::endl;             //正确

    for (int dim = 0; dim < 3; dim++) 
    {

        Eigen::VectorXd coeff;
        coeff.resize(poly_coeff_num);
        
        coeff = (polyCoeff.row(k)).segment(dim * poly_coeff_num, poly_coeff_num);//k行代表第k段轨迹，
        Eigen::VectorXd times = Eigen::VectorXd::Zero(poly_coeff_num);
        for (int j = 0; j < poly_coeff_num; j++)
            if (j == 0 || j == 1)
                times(j) = 0;
            else if(j == 2)
                times(j) = 2;
            else
                times(j) = j * (j-1) * pow(t, j-2);
        double temp_acc = 0.0;
        for (int i = 0; i < times.rows(); ++i) 
        {
            temp_acc = temp_acc + coeff(i) * times(times.rows() - i - 1);
        }
        acct(dim) = temp_acc;
    }

    //std::cout << "acc:" << acct << std::endl;       //获取速度成功
    return acct;
}
/*!
 * 求解第k个轨迹段t时刻对应的jerk
 * @param polyCoeff 多项式系数矩阵
 * @param k 轨迹段序号
 * @param t 时刻
 * @return [jx,jy,jz]^T
 */
Eigen::Vector3d planner::getJerkPoly(Eigen::MatrixXd polyCoeff, int k, double t) 
{
    Eigen::Vector3d jerkt;
    poly_coeff_num= 2 * mode;
    // std::cout << "poly_coeff_num:" << poly_coeff_num << std::endl;             //正确

    for (int dim = 0; dim < 3; dim++) 
    {

        Eigen::VectorXd coeff;
        coeff.resize(poly_coeff_num);
        
        coeff = (polyCoeff.row(k)).segment(dim * poly_coeff_num, poly_coeff_num);//k行代表第k段轨迹，
        Eigen::VectorXd times = Eigen::VectorXd::Zero(poly_coeff_num);
        for (int j = 0; j < poly_coeff_num; j++)
            if (j == 0 || j == 1 || j == 2) 
                times(j) = 0;
            else if(j == 3)
                times(j) = 6;
            else
                times(j) = j * (j-1) * (j-2) * pow(t, j-3);
        double temp_jerk = 0.0;
        for (int i = 0; i < times.rows(); ++i) 
        {
            temp_jerk = temp_jerk + coeff(i) * times(times.rows() - i - 1);
        }
        jerkt(dim) = temp_jerk;
    }

    //std::cout << "jerk:" << jerkt << std::endl;       //获取速度成功
    return jerkt;
}

std::vector<quadrotor_msgs::PositionCommand> planner::get_trajectory(void)
{
    std::vector<quadrotor_msgs::PositionCommand> trajectory_vector;
    quadrotor_msgs::PositionCommand trajectory;
    ros::Publisher tra_generation_pub = n.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    //ros::Publisher realpath_pub = n.advertise<nav_msgs::Path>("/real_trajectory", 10);

    Vector3d pos_;             //用于获取getPosPoly返回的向量，转化为pose信息
    Vector3d vel_;
    Vector3d acc_;
    Vector3d jerk_;
    ros::Duration offset(2.0);
    ros::Time start_time = ros::Time::now();
    
    trajectory.header.frame_id = "world";
    trajectory.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    trajectory.kx[0] = 0;
    trajectory.kx[1] = 0;
    trajectory.kx[2] = 0;
    trajectory.kv[0] = 0;
    trajectory.kv[1] = 0;
    trajectory.kv[2] = 0;
    ros::Rate rate(100);
    ros::Time cur_time = ros::Time::now();
    trajectory.header.stamp = cur_time;
    //cout << "time_size:" << time_everytraj.size()<< endl;
    for (int i = 0; i < time_everytraj.size(); i++) 
    {
        
        offset.fromSec(time_everytraj(i));
        //cout << "time(i):" << time_everytraj(i)<< endl;
        for (double t = 0.0; t < time_everytraj(i); t += 0.01) 
        {
            
            //生成控制命令
            pos_ = getPosPoly(poly_coeff, i, t);
            vel_ = getVelPoly(poly_coeff, i, t);
            acc_ = getAccPoly(poly_coeff, i ,t);
            jerk_ = getJerkPoly(poly_coeff, i ,t);
            trajectory.position.x = pos_(0);
            trajectory.position.y = pos_(1);
            trajectory.position.z = pos_(2);

            trajectory.velocity.x = vel_(0);
            trajectory.velocity.y = vel_(1);
            trajectory.velocity.z = vel_(2);

            trajectory.acceleration.x = acc_(0);
            trajectory.acceleration.y = acc_(1);
            trajectory.acceleration.z = acc_(2);

            trajectory.jerk.x = jerk_(0);
            trajectory.jerk.y = jerk_(1);
            trajectory.jerk.z = jerk_(2);

            trajectory.yaw = 0;
            trajectory.yaw_dot = 0;
            //将轨迹压入到容器当中
            //tra_generation_pub.publish(trajectory);
            //cout << trajectory << endl;
            // cout << "positon:" << trajectory.position << endl;
            // cout << "velocity:" << trajectory.velocity << endl;
            trajectory_vector.push_back(trajectory);
        }
        trajectory.header.stamp = trajectory.header.stamp + offset;
        trajectory.trajectory_id = trajectory.trajectory_id + 1;   
    }   
    
    //ros::Duration(1).sleep();     
    // cout << "trajectory:" << trajectory << endl;
    //return trajectory;              //  返回产生的 path 信息
    return trajectory_vector;
}
void planner::draw_desire_trajectory(ros::Publisher despath_pub){
    static nav_msgs::Path desire_path;
    geometry_msgs::PoseStamped pose;
    Vector3d pos_;
    ROS_INFO("Drawing desire trajectory.");
    //ros::Publisher despath_pub = n.advertise<nav_msgs::Path>("/desire_trajectory", 10, true);
    ros::Duration(2.0).sleep();
    
    desire_path.header.frame_id = "world";
    desire_path.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    desire_path.poses.clear();
    for (int i = 0; i < time_everytraj.size(); i++) 
    {
        for (double t = 0.0; t < time_everytraj(i); t += 0.01) 
        {
            pos_ = getPosPoly(poly_coeff, i, t);
            pose.pose.position.x = pos_(0);
            pose.pose.position.y = pos_(1);
            pose.pose.position.z = pos_(2);
            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time::now();

            desire_path.poses.push_back(pose); 
        }  
    }
    //std::cout<< desire_path << endl;
    despath_pub.publish(desire_path); 
    
    ROS_INFO("Desire trajectory was drawn.");
}
void planner::draw_desire_trajectory_marker(ros::Publisher despath_pub){
    Vector3d pos_;
    geometry_msgs::Point point;
    ROS_INFO("Drawing desire trajectory.");
     // 创建visualization_msgs::Marker消息
    static visualization_msgs::Marker marker;
    // 设置消息类型为LINE_STRIP
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // 设置坐标系
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    // 设置线条的宽度
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    // 设置线条的颜色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.clear();
    for (int i = 0; i < time_everytraj.size(); i++) 
    {
        for (double t = 0.0; t < time_everytraj(i); t += 0.01) 
        {
            pos_ = getPosPoly(poly_coeff, i, t);
            point.x = pos_(0);
            point.y = pos_(1);
            point.z = pos_(2);
            marker.points.push_back(point);
        }  
    }
    despath_pub.publish(marker);
    ROS_INFO("Desire trajectory was drawn.");
}
/*
*轨迹发布
*/

void planner::tra_publish(void)
{
    ros::Publisher tra_generation_pub = n.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10); 
    ros::Publisher real_trajectory_pub = n.advertise<nav_msgs::Path>("/real_trajectory", 10, true); 
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);

    nav_msgs::Path real_path;
    geometry_msgs::PoseStamped pose;

    std::vector<quadrotor_msgs::PositionCommand> tra;
    tra = get_trajectory();   // 获取轨迹path
    ros::Rate rate(100);  //设置发布频率
    real_path.header.frame_id = "world";
    real_path.header.stamp = ros::Time::now();
    real_path.poses.clear();

    for(std::vector<quadrotor_msgs::PositionCommand>::iterator it = tra.begin(); it != tra.end(); ++it){
        tra_generation_pub.publish(*it);
        pose.pose.position.x = px4_pose.pose.position.x;
        pose.pose.position.y = px4_pose.pose.position.y;
        pose.pose.position.z = px4_pose.pose.position.z;
        real_path.poses.push_back(pose);
        real_trajectory_pub.publish(real_path);
        rate.sleep();
        ros::spinOnce();
    }
}


