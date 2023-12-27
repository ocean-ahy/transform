
/*

    基本用法示例


*/

#include "transform3d.hpp"
#include <iostream>
using namespace std;



int main()
{
    Eigen::Matrix3d R; // 矩阵3*3
    Eigen::Vector3d rpy; // 欧拉角 rx ry rz 
    Eigen::Vector3d axis; // 轴角 、 旋转矢量，  模长为角度
    Eigen::Quaterniond Q; // 四元数
    Eigen::Matrix4d H; // 齐次矩阵 4*4 


    //欧拉转矩阵
    R = tf3d::Euler2Mat(10, 20, 6); // 默认 zyx方向 旋转 6 、20、 10 度
    tf3d::printMat(R, "R = ");
    rpy = tf3d::Mat2Euler(R);
    tf3d::printRPY(rpy, "rpy = "); //

    // 四元数
    Q = tf3d::Mat2Quat(R); // 矩阵转四元数
    tf3d::printQuaternion(Q, "R---Q\t");
    Q = tf3d::Euler2Quat(rpy); // 欧拉转四元数
    tf3d::printQuaternion(Q, "rpy---Q\t");

    // 轴角 -旋转矢量
    axis = tf3d::Mat2AxisAngle(R);
    tf3d::printVec(axis, "R---ax\t");
    axis = tf3d::Quat2AxisAngle(Q);
    tf3d::printVec(axis, "Q---ax\t");
    axis = tf3d::Euler2AxisAngle(rpy);
    tf3d::printVec(axis, "rpy---ax\t");

    // 齐次矩阵
    H = tf3d::Mat2H(R);
    tf3d::printMat(H, "R --H");
    H = tf3d::ComposeH(R, Eigen::Vector3d(2,3,4));
    tf3d::printMat(H, "H =");
    tf3d::printMat(tf3d::GetRotFromHomo(H), "H---R ");

    // 矩阵 转为  x y z rx ry rz
    Eigen::VectorXd pose = tf3d::H2PoseRPY(H);
    tf3d::printVec(pose, "pose ");
    Eigen::VectorXd axispose = tf3d::H2PoseAxis(H);
    Eigen::VectorXd quatpose  = tf3d::H2PoseQuat(H);
    tf3d::printVec(quatpose, "quatpose ");  //x y z  qw qx qy qz




    return 0;

}

