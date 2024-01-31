#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
class Minisnap 
{
private:
    int Factorial(int x);   //求阶乘

public:
    Minisnap() = default;

    ~Minisnap() = default;

    /*求解闭式解，系数矩阵*/
    Eigen::MatrixXd SolveQPClosedForm(
            int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

};

#endif