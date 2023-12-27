/*

测试 Mat2AxisAngle  基本推导方法 错误定位


double ang = acos((R.trace() - 1.0) / 2);
double k = 1.0 / (2 * sin(ang));
Eigen::Vector3d axis;
axis[0] = k * (R(2, 1) - R(1, 2));
axis[1] = k * (R(0, 2) - R(2, 0));
axis[2] = k * (R(1, 0) - R(0, 1));

对于特殊情况的R  计算结果不对

*/


#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "../transform3d.hpp"

using namespace std;
double PI_ = 3.1415926535897932384626433832795;
const double TO_DEGREE = 180.0 / PI_; // 转为度，乘的系数
const double TO_RADIAN = PI_ / 180.0; // 转为弧度，乘的系数

// 角度类型枚举
enum AngType
{
    DEGREE = 0, // 度
    RADIAN,     // 弧度
};

Eigen::Vector3d Mat2AxisAngle(Eigen::Matrix3d R)
{
    double ang = acos((R.trace() - 1.0) / 2);
    double k = 1.0 / (2 * sin(ang));
    Eigen::Vector3d axis;
    axis[0] = k * (R(2, 1) - R(1, 2));
    axis[1] = k * (R(0, 2) - R(2, 0));
    axis[2] = k * (R(1, 0) - R(0, 1));
    // ang *= TO_DEGREE; // 转为°
    axis *= ang;

    return axis;
}

Eigen::Vector3d Mat2AxisAngle2(Eigen::Matrix3d R)
{
    Eigen::Vector3d axis;
    Eigen::AngleAxisd V(R);
    axis = V.axis() * V.angle();
    return axis;
}

Eigen::Vector3d Mat2AxisAngle3(Eigen::Matrix3d R)
{
    Eigen::Quaterniond Q(R);
    Eigen::Vector3d axis;
    double t = 2 * acos(Q.w());
    axis << Q.x(), Q.y(), Q.z();
    axis = axis / sin(t / 2) * t;
    return axis;
}

Eigen::Quaterniond Mat2Quat2(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q;
    q.w() = sqrt(R.trace() + 1.0) / 2;
    q.x() = (R(2, 1) - R(1, 2)) / (4 * q.w());
    q.y() = (R(0, 2) - R(2, 0)) / (4 * q.w());
    q.z() = (R(1, 0) - R(0, 1)) / (4 * q.w());
    return q;
}

Eigen::Quaterniond Euler2Quat(Eigen::Vector3d rpy)
{
    Eigen::Vector3d tmp = rpy * TO_RADIAN;

    Eigen::Quaterniond qz(cos(0.5 * tmp.z()), 0, 0, sin(0.5 * tmp.z()));
    Eigen::Quaterniond qy(cos(0.5 * tmp.y()), 0, sin(0.5 * tmp.y()), 0);
    Eigen::Quaterniond qx(cos(0.5 * tmp.x()), sin(0.5 * tmp.x()), 0, 0);

    Eigen::Quaterniond res1, res2;
    res1 = qz * qy * qx;

    double x = tmp.x() / 2;
    double y = tmp.y() / 2;
    double z = tmp.z() / 2;

    res2.w() = cos(x)*cos(y)*cos(z) + sin(x)*sin(y)*sin(z);
    res2.x() = sin(x)*cos(y)*cos(z) - cos(x)*sin(y)*sin(z);
    res2.y() = cos(x)*sin(y)*cos(z) + sin(x)*cos(y)*sin(z);
    res2.z() = cos(x)*cos(y)*sin(z) - sin(x)*sin(y)*cos(z);

    tf3d::printQuaternion(res1, "res1 ");
    tf3d::printQuaternion(res2, "res2 ");



    return res1;
}

int main()
{
    Eigen::Quaterniond Q1(0, 0.707, -0.707, 0); // wxyz
    Eigen::Matrix3d M = Q1.normalized().toRotationMatrix();

    Eigen::Vector3d rpy(180, -0, 180);
    // M = tf3d::Euler2Mat(rpy);
    // M = tf3d::Euler2Mat(10, 20, 30);

    // M << 0,1,0,
    //      1,0,0,
    //      0,0,-1;

    tf3d::printRPY(tf3d::Mat2Euler(M), "M=");

    tf3d::printQuaternion(Eigen::Quaterniond(M), " a---");
    tf3d::printQuaternion(Euler2Quat(tf3d::Mat2Euler(M)), " b---");
    tf3d::printQuaternion(Mat2Quat2(M), " c---");

    tf3d::printMat(M, "0----");

    Eigen::Vector3d v1, v2, v3, v4;
    v1 = Mat2AxisAngle(M);
    v2 = Mat2AxisAngle2(M);
    v3 = Mat2AxisAngle3(M);

    tf3d::printVec(v1, "1");
    tf3d::printVec(v2, "2");
    tf3d::printVec(v3, "3");

    Eigen::Matrix3d M2;
    M2 = tf3d::AxisAngle2Mat(v1, tf3d::RADIAN);
    tf3d::printMat(M2, "1----");
    M2 = tf3d::AxisAngle2Mat(v2, tf3d::RADIAN);
    tf3d::printMat(M2, "2----");
    M2 = tf3d::AxisAngle2Mat(v3, tf3d::RADIAN);
    tf3d::printMat(M2, "3----");

    return 0;
}