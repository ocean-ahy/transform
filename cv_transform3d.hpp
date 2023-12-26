/*
 * @文件名: transform3d.hpp
 * @文件描述:  基于Eigen3 ，空间矩阵变换库
 * @版本: V1.0
 * @作者: AHY
 * @日期: 2022-02
 * @其他说明:
 *    旋转向量即是轴角  RotVec = AxisAngle ,含义相通
 *    所有欧拉角、旋转矢量(轴角) 默认输出为°，
 *    输入以flag判定类型， 没有flag标志位 ，则默认为°
 *    Mat 为3*3旋转矩阵， H为 4*4齐次矩阵
 */

#ifndef TRANSFORM3D_HPP
#define TRANSFORM3D_HPP

#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cmath>

// using namespace std;
// using namespace Eigen;

using std::cout;
using std::endl;

namespace tf3d
{

    double PI_ = 3.1415926535897932384626433832795;
    const double TO_DEGREE = 180.0 / PI_; // 转为度，乘的系数
    const double TO_RADIAN = PI_ / 180.0; // 转为弧度，乘的系数

    // 角度类型枚举
    enum AngType
    {
        DEGREE = 0, // 度
        RADIAN,     // 弧度
    };

    /**
     * @函数功能: 弧度/度 转换
     * @参数:
     * @返回值:
     */
    template <typename T>
    inline T deg2rad(T x) { return (x * TO_RADIAN); }
    template <typename T>
    inline T rad2deg(T x) { return (x * TO_DEGREE); }

    /**
     * @函数功能: 绕X旋转
     * @参数: 单位 ：度
     * @返回值:
     */
    inline Eigen::Matrix3d RotX(double rx)
    {
        double ct = cos(deg2rad(rx));
        double st = sin(deg2rad(rx));
        Eigen::Matrix3d mat;
        mat << 1.0, 0.0, 0.0,
            0.0, ct, -st,
            0.0, st, ct;

        // Eigen::AngleAxisd V(deg2rad(rx), Eigen::Vector3d::UnitX());
        // cout << V.toRotationMatrix() << endl;
        return mat;
    }

    /**
     * @函数功能: 绕Y旋转
     * @参数: 单位 ：度
     * @返回值:
     */
    inline Eigen::Matrix3d RotY(double ry)
    {
        double ct = cos(deg2rad(ry));
        double st = sin(deg2rad(ry));
        Eigen::Matrix3d mat;
        mat << ct, 0.0, st,
            0.0, 1.0, 0.0,
            -st, 0.0, ct;
        return mat;
    }

    /**
     * @函数功能: 绕Z旋转
     * @参数: 单位 ：度
     * @返回值:
     */
    inline Eigen::Matrix3d RotZ(double rz)
    {
        double ct = cos(deg2rad(rz));
        double st = sin(deg2rad(rz));
        Eigen::Matrix3d mat;
        mat << ct, -st, 0.0,
            st, ct, 0.0,
            0.0, 0.0, 1.0;
        return mat;
    }
    /**
     * @函数功能: 欧拉 转 旋转矩阵 z-y-x
     * @参数:
     * @param {double} rx
     * @param {double} ry
     * @param {double} rz
     * @param {AngType} flag
     * @返回值:
     * @版本:
     * @作者:
     * @日期:
     */
    inline Eigen::Matrix3d Euler2Mat(double rx, double ry, double rz, AngType flag = DEGREE)
    {

#if 1
        if (flag != DEGREE)
        {
            rx = rad2deg(rx);
            ry = rad2deg(ry);
            rz = rad2deg(rz);
        }
        Eigen::Matrix3d rotation_matrix = RotZ(rz) * RotY(ry) * RotX(rx);
        // cout << "rotation_matrix=\n" << rotation_matrix << endl;
#else
        if (flag == DEGREE)
        {
            rx = deg2rad(rx);
            ry = deg2rad(ry);
            rz = deg2rad(rz);
        }
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()));
        rotation_matrix = yawAngle * pitchAngle * rollAngle;

        cout << "rotation_matrix=\n"
             << rotation_matrix << endl;
#endif
        return rotation_matrix;
    }

    /**
     * @函数功能:欧拉 转 旋转矩阵 z-y-x
     * @参数:
     * @param {Vector3d} eularAngle
     * @param {AngType} flag
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix3d Euler2Mat(Eigen::Vector3d eularAngle, AngType flag = DEGREE)
    {
        double rx, ry, rz;
        rx = (flag == DEGREE) ? eularAngle.x() : rad2deg(eularAngle.x());
        ry = (flag == DEGREE) ? eularAngle.y() : rad2deg(eularAngle.y());
        rz = (flag == DEGREE) ? eularAngle.z() : rad2deg(eularAngle.z());
        Eigen::Matrix3d rotation_matrix = RotZ(rz) * RotY(ry) * RotX(rx);
        return rotation_matrix;
    }

    /**
     * @函数功能: 是否为旋转矩阵
     * @参数:
     * @param {Matrix3d} R 3x3
     * @返回值: bool
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline bool isRotatedMatrix(Eigen::Matrix3d R)
    {
        double err = 1e-6;
        Eigen::Matrix3d shouldIdenity;
        shouldIdenity = R * R.transpose();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        return (shouldIdenity - I).norm() < err;
    }

    /**
     * @函数功能: 旋转矩阵转为欧拉角  zyx
     * @参数:
     * @param {Matrix3d} &R
     * @返回值: 欧拉角，角度制
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Vector3d Mat2Euler(const Eigen::Matrix3d &R)
    {
#if 0
        Eigen::Vector3d tmp = R.eulerAngles(2, 1, 0) * TO_DEGREE; // ZYX顺序，yaw,pitch,roll
        Eigen::Vector3d euler(tmp[2], tmp[1], tmp[0]);
        return euler;

#else
        double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
        bool singular = sy < 1e-6; // If

        double x, y, z;
        if (!singular)
        {
            x = atan2(R(2, 1), R(2, 2));
            y = atan2(-R(2, 0), sy);
            z = atan2(R(1, 0), R(0, 0));
        }
        else
        {
            x = atan2(-R(1, 2), R(1, 1));
            y = atan2(-R(2, 0), sy);
            z = 0;
        }
        Eigen::Vector3d euler(x, y, z);
        return (euler * TO_DEGREE);
#endif
    }

    /**
     * @函数功能: 生成反对称矩阵
     * @参数:
     * @param {Vector3d} k ：三维单位向量
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix3d SkewMat(Eigen::Vector3d k)
    {
        Eigen::Matrix3d K;
        K << 0.0, -k.z(), k.y(),
            k.z(), 0.0, -k.x(),
            -k.y(), k.x(), 0.0;
        return K;
    }

    /**
     * @函数功能: 罗德里格斯变换
     * @参数:
     * @param {Vector3d} rot_vec 旋转矢量
     * @param {AngType} flag 角度/ 弧度
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix3d Rodrigues(Eigen::Vector3d rot_vec, AngType flag = DEGREE)
    {
        double t = rot_vec.norm();
        if (flag == DEGREE)
            t = deg2rad(t);

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d K = SkewMat(rot_vec.normalized());
        Eigen::Matrix3d R = I + (1 - cos(t)) * (K * K) + sin(t) * K;
        return R;
    }

    /**
     * @函数功能:  轴角 - 旋转矩阵 ， 原理同 Rodrigues
     * @参数: 轴角向量， 向量模长：旋转角度
     * @param {double} x
     * @param {double} y
     * @param {double} z
     * @param {AngType} flag  弧度、度
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix3d AxisAngle2Mat(double x, double y, double z, AngType flag = DEGREE)
    {
        // 计算角度
        double t = sqrt(x * x + y * y + z * z);
        // 单位化
        double kx = x / t;
        double ky = y / t;
        double kz = z / t;

        if (flag == DEGREE)
            t = deg2rad(t);

        double ct = cos(t);
        double st = sin(t);
        double vt = 1.0 - ct;

        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        R << kx * kx * vt + ct, kx * ky * vt - kz * st, kx * kz * vt + ky * st,
            kx * ky * vt + kz * st, ky * ky * vt + ct, ky * kz * vt - kx * st,
            kx * kz * vt - ky * st, ky * kz * vt + kx * st, kz * kz * vt + ct;

        return R;
    }

    /**
     * @函数功能: 轴角 - 旋转矩阵  原理同 Rodrigues
     * @参数:  轴角向量， 向量模长：旋转角度
     * @param {Vector3d} axis
     * @param {AngType} flag
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix3d AxisAngle2Mat(Eigen::Vector3d axis, AngType flag = DEGREE)
    {
        double ang = axis.norm(); // 角度 = 向量模长
        axis = axis.normalized();

        if (flag == DEGREE)
            ang = deg2rad(ang);

        return Eigen::AngleAxisd(ang, axis).toRotationMatrix();
    }

    /**
     * @函数功能: 轴角 - 旋转矩阵
     * @参数:
     * @param {Vector3d} axis  轴向量，
     * @param {double} ang  绕axis 旋转的角度
     * @param {AngType} flag  弧度、度
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix3d AxisAngle2Mat(Eigen::Vector3d axis, double ang, AngType flag = DEGREE)
    {
        axis = axis.normalized();
        if (flag == DEGREE)
            ang = deg2rad(ang);

        return Eigen::AngleAxisd(ang, axis).toRotationMatrix();
    }

    /**
     * @函数功能:  旋转向量-矩阵 ： 同AxisAngle2Mat
     * @参数:
     */
    inline Eigen::Matrix3d RotVector2Mat(double rx, double ry, double rz, AngType flag = DEGREE)
    {
        // 计算角度
        double t = sqrt(rx * rx + ry * ry + rz * rz);

        // 单位化
        double kx = rx / t;
        double ky = ry / t;
        double kz = rz / t;

        if (flag == DEGREE)
            t = deg2rad(t);

        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        R = Eigen::AngleAxisd(t, Eigen::Vector3d(kx, ky, kz)).toRotationMatrix();

        return R;
    }

    /**
     * @函数功能: 旋转矩阵- 轴角
     * @参数:
     * @param {Matrix3d} R  入参 矩阵
     * @param {Vector3d} &axis  出参 单位向量
     * @param {double} &ang  出参 角度，单位度
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline bool Mat2AxisAngle(Eigen::Matrix3d R, Eigen::Vector3d &axis, double &ang)
    {
        if (!isRotatedMatrix(R))
            return false;

        ang = acos((R.trace() - 1.0) / 2);
        double k = 1.0 / (2 * sin(ang));
        axis[0] = k * (R(2, 1) - R(1, 2));
        axis[1] = k * (R(0, 2) - R(2, 0));
        axis[2] = k * (R(1, 0) - R(0, 1));

        ang *= TO_DEGREE; // 转为°
        return true;
    }

    /**
     * @函数功能: 旋转矩阵- 轴角
     * @参数:
     * @param {Matrix3d} R
     * @param {Vector3d} &axis  轴角，模长为角度，单位度
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline bool Mat2AxisAngle(Eigen::Matrix3d R, Eigen::Vector3d &axis)
    {
        if (!isRotatedMatrix(R))
            return false;

        double ang = acos((R.trace() - 1.0) / 2);
        double k = 1.0 / (2 * sin(ang));
        axis[0] = k * (R(2, 1) - R(1, 2));
        axis[1] = k * (R(0, 2) - R(2, 0));
        axis[2] = k * (R(1, 0) - R(0, 1));

        ang *= TO_DEGREE; // 转为°
        axis *= ang;
        return true;
    }

    /**
     * @函数功能:
     * @参数:
     * @param {Matrix3d} R
     * @返回值:  axis 轴角， 默认为度
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Vector3d Mat2AxisAngle(Eigen::Matrix3d R)
    {
        double ang = acos((R.trace() - 1.0) / 2);
        double k = 1.0 / (2 * sin(ang));
        Eigen::Vector3d axis;
        axis[0] = k * (R(2, 1) - R(1, 2));
        axis[1] = k * (R(0, 2) - R(2, 0));
        axis[2] = k * (R(1, 0) - R(0, 1));
        ang *= TO_DEGREE; // 转为°
        axis *= ang;

        // Eigen::AngleAxisd V(R);
        // axis = V.axis() * V.angle() * TO_DEGREE;
        return axis;
    }

    /**
     * @函数功能: 旋转 、平移  合成 4x4齐次矩阵Homogeneous
     * @参数:
     * @param {Matrix3d} R
     * @param {Vector3d} P
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix4d ComposeH(Eigen::Matrix3d R, Eigen::Vector3d P)
    {
        Eigen::Matrix4d H = Eigen::Matrix4d::Identity(); // 齐次矩阵H
        H.block<3, 3>(0, 0) = R;
        H(0, 3) = P(0);
        H(1, 3) = P(1);
        H(2, 3) = P(2);
        return H;
    }

    /**
     * @函数功能: 3x3 变为 4x4矩阵
     * @参数:
     * @param {Matrix3d} R
     * @返回值:
     */
    inline Eigen::Matrix4d Mat2H(Eigen::Matrix3d R)
    {
        return ComposeH(R, Eigen::Vector3d(0.0, 0.0, 0.0));
    }

    /**
     * @函数功能: 获取3x3 矩阵R
     * @参数: 4x4齐次矩阵
     * @返回值:
     */
    inline Eigen::Matrix3d GetRotFromHomo(Eigen::Matrix4d H)
    {
        Eigen::Matrix3d R;
        R = H.block<3, 3>(0, 0);
        return R;
    }

    /**
     * @函数功能: 获取 平移矢量
     * @参数: 4x4齐次矩阵
     * @返回值:
     */
    inline Eigen::Vector3d GetPosFromHomo(Eigen::Matrix4d H)
    {
        Eigen::Vector3d P;
        P = H.block<3, 1>(0, 3).col(0);
        return P;
    }

    /**
     * @函数功能: 齐次矩阵 分解 旋转 + 平移
     * @参数:
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix3d HDecompose(Eigen::Matrix4d H,
                                      Eigen::Matrix3d &R,
                                      Eigen::Vector3d &P)
    {
        R = GetRotFromHomo(H);
        P = GetPosFromHomo(H);
        return R;
    }

    /**
     * @函数功能: 6自由度欧拉数组  转为 4x4矩阵
     * @参数:
     * @param {double} pose
     * @param {AngType} flag  欧拉角  ：度？弧度？
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix4d PoseRPY2H(double pose[6], AngType flag = DEGREE)
    {
        Eigen::Vector3d p(pose[0], pose[1], pose[2]);
        Eigen::Vector3d eular(pose[3], pose[4], pose[5]);
        return ComposeH(Euler2Mat(eular, flag), p);
    }

    inline Eigen::Matrix4d PoseRPY2H(Eigen::VectorXd pose, AngType flag = DEGREE)
    {
        if (pose.size() == 6)
        {
            Eigen::Vector3d p(pose(0), pose(1), pose(2));
            Eigen::Vector3d eular(pose(3), pose(4), pose(5));
            return ComposeH(Euler2Mat(eular, flag), p);
        }

        return Eigen::Matrix4d::Zero();
    }

    inline Eigen::Matrix4d PoseRPY2H(double x,
                                     double y,
                                     double z,
                                     double rx,
                                     double ry,
                                     double rz,
                                     AngType flag = DEGREE)
    {
        Eigen::Vector3d p(x, y, z);
        Eigen::Vector3d eular(rx, ry, rz);
        return ComposeH(Euler2Mat(eular, flag), p);
    }

    /**
     * @函数功能: 6自由度数组， 轴角 转为4x4矩阵
     * @参数:
     * @param {double} pose  分别为x y z ， 轴角 rx ry rz
     * @param {AngType} flag
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix4d PoseAxis2H(double pose[6], AngType flag = DEGREE)
    {
        Eigen::Vector3d p(pose[0], pose[1], pose[2]);
        Eigen::Vector3d axis(pose[3], pose[4], pose[5]);
        return ComposeH(AxisAngle2Mat(axis, flag), p);
    }

    /**
     * @函数功能:
     * @参数:
     * @param {VectorXd} pose 6维向量
     * @param {AngType} flag
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix4d PoseAxis2H(Eigen::VectorXd pose, AngType flag = DEGREE)
    {
        if (pose.size() == 6)
        {
            Eigen::Vector3d p(pose(0), pose(1), pose(2));
            Eigen::Vector3d axis(pose(3), pose(4), pose(5));
            return ComposeH(AxisAngle2Mat(axis, flag), p);
        }
        return Eigen::Matrix4d::Zero();
    }

    inline Eigen::Matrix4d PoseAxis2H(double x,
                                      double y,
                                      double z,
                                      double rx,
                                      double ry,
                                      double rz,
                                      AngType flag = DEGREE)
    {
        Eigen::Vector3d p(x, y, z);
        Eigen::Vector3d axis(rx, ry, rz);
        return ComposeH(AxisAngle2Mat(axis, flag), p);
    }

    /**
     * @函数功能: xyz + 四元数 转为 4x4矩阵
     * @参数:
     * @param {VectorXd} pose： 7维矢量 ，后四个数为四元数
     * @param {string} seq 四元数，数据序列
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Matrix4d PoseQuat2H(Eigen::VectorXd pose, std::string seq = "wxyz")
    {
        if (pose.size() == 7)
        {
            Eigen::Vector3d p(pose(0), pose(1), pose(2));

            Eigen::Quaterniond q(0, 0, 0, 1); // w x y z顺序初始化
            if (seq == "wxyz")
            {
                q.w() = pose(3);
                q.x() = pose(4);
                q.y() = pose(5);
                q.z() = pose(6);
            }
            else if (seq == "xyzw")
            {
                q.x() = pose(3);
                q.y() = pose(4);
                q.z() = pose(5);
                q.w() = pose(6);
            }

            return ComposeH(q.toRotationMatrix(), p);
        }
        return Eigen::Matrix4d::Zero();
    }

    /**
     * @函数功能: 4x4齐次矩阵  转为 6维矢量 ： 欧拉
     * @参数:
     * @param {Matrix4d} H
     * @param {AngType} flag
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::VectorXd H2PoseRPY(Eigen::Matrix4d H, AngType flag = DEGREE)
    {
        Eigen::VectorXd pose = Eigen::VectorXd(6);
        Eigen::Vector3d p = GetPosFromHomo(H);
        pose(0, 0) = p.x();
        pose(1, 0) = p.y();
        pose(2, 0) = p.z();
        Eigen::Vector3d rpy = Mat2Euler(GetRotFromHomo(H));
        if (flag == RADIAN)
            rpy *= TO_RADIAN;

        pose(3, 0) = rpy.x();
        pose(4, 0) = rpy.y();
        pose(5, 0) = rpy.z();

        return pose;
    }

    inline bool H2PoseRPY(Eigen::Matrix4d H, Eigen::VectorXd &pose, AngType flag = DEGREE)
    {
        if (pose.size() != 6)
            return false;

        Eigen::Vector3d p = GetPosFromHomo(H);
        pose(0) = p.x();
        pose(1) = p.y();
        pose(2) = p.z();
        Eigen::Vector3d rpy = Mat2Euler(GetRotFromHomo(H));
        if (flag == RADIAN)
            rpy *= TO_RADIAN;

        pose(3) = rpy.x();
        pose(4) = rpy.y();
        pose(5) = rpy.z();

        return true;
    }

    /**
     * @函数功能: 4x4齐次矩阵 转为 6自由度 轴角
     * @参数:
     * @param {Matrix4d} H
     * @param {AngType} flag
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::VectorXd H2PoseAxis(Eigen::Matrix4d H, AngType flag = DEGREE)
    {
        Eigen::VectorXd pose = Eigen::VectorXd(6);
        Eigen::Vector3d p = GetPosFromHomo(H);
        pose(0, 0) = p.x();
        pose(1, 0) = p.y();
        pose(2, 0) = p.z();
        Eigen::Vector3d axis = Mat2AxisAngle(GetRotFromHomo(H));
        if (flag == RADIAN)
            axis *= TO_RADIAN;

        pose(3, 0) = axis.x();
        pose(4, 0) = axis.y();
        pose(5, 0) = axis.z();

        return pose;
    }

    inline bool H2PoseAxis(Eigen::Matrix4d H, Eigen::VectorXd &pose, AngType flag = DEGREE)
    {
        if (pose.size() != 6)
            return false;

        Eigen::Vector3d p = GetPosFromHomo(H);
        pose(0) = p.x();
        pose(1) = p.y();
        pose(2) = p.z();
        Eigen::Vector3d axis = Mat2AxisAngle(GetRotFromHomo(H));
        if (flag == RADIAN)
            axis *= TO_RADIAN;

        pose(3) = axis.x();
        pose(4) = axis.y();
        pose(5) = axis.z();

        return true;
    }

    /**
     * @函数功能:  欧拉角  转为 旋转矢量/轴角
     * @参数:
     * @param {Vector3d} euler 欧拉角 rx ry rz， 默认顺序zyx
     * @param {AngType} flag 角度类型。 输出类型与输入一致
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Vector3d Euler2RotVec(Eigen::Vector3d euler, AngType flag = DEGREE)
    {
        Eigen::Matrix3d R = Euler2Mat(euler, flag);
        Eigen::Vector3d rotVec = Mat2AxisAngle(R);

        return ((flag == RADIAN) ? rotVec * TO_RADIAN : rotVec);
    }

    inline Eigen::Vector3d RotVec2Euler(Eigen::Vector3d rot_vec, AngType flag = DEGREE)
    {
        Eigen::Matrix3d R = AxisAngle2Mat(rot_vec, flag);
        Eigen::Vector3d euler = Mat2Euler(R);
        return ((flag == RADIAN) ? euler * TO_RADIAN : euler);
    }

    /*-------------------------------四元数------------------------------------*/
    inline Eigen::Matrix3d Quat2Mat(Eigen::Quaterniond q) { return q.matrix(); }
    inline Eigen::Vector3d Quat2Eular(Eigen::Quaterniond q) { return Mat2Euler(q.matrix()); }
    inline Eigen::Vector3d Quat2Axis(Eigen::Quaterniond q) { return Mat2AxisAngle(q.matrix()); }

    inline Eigen::Quaterniond Eular2Quat(Eigen::Vector3d rpy) { return Eigen::Quaterniond(Euler2Mat(rpy)); }
    inline Eigen::Quaterniond AxisAngle2Quat(Eigen::Vector3d axis) { return Eigen::Quaterniond(AxisAngle2Mat(axis)); }
    inline Eigen::Quaterniond Mat2Quat(Eigen::Matrix3d mat) { return Eigen::Quaterniond(mat); }

    /**
     * @函数功能: 轴角转四元数
     * @参数:
     * @param {Vector3d} axis
     * @param {double} ang
     * @param {AngType} flag
     * @返回值:
     * @版本: V1.0
     * @作者: AHY
     * @日期:
     */
    inline Eigen::Quaterniond AxisAngle2Quat(Eigen::Vector3d axis, double ang, AngType flag = DEGREE)
    {
        if (flag == DEGREE)
            ang *= TO_RADIAN;

        axis = axis.normalized();
        Eigen::Quaterniond q;
        q.w() = cos(ang / 2);
        q.x() = axis.x() * sin(ang / 2);
        q.y() = axis.y() * sin(ang / 2);
        q.z() = axis.z() * sin(ang / 2);
        return q;
    }

    inline void printQuaternion(const Eigen::Quaterniond &q, const std::string &info)
    {
        std::cout << info << " x y z w: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl; // 两种元素访问方式均可
    }

    inline void printRPY(const Eigen::Vector3d &rpy, const std::string &info)
    {

        std::cout << info << " r p y: " << (rpy(0)) << ", " << (rpy(1)) << ", " << (rpy(2)) << std::endl;
    }

    inline void printAngleAxisd(const Eigen::AngleAxisd &x, const std::string &info)
    {
        // std::cout << "quaternion: " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << std::endl;  // x,y,z,w
        std::cout << info << "  axis: " << x.axis().transpose() << "  "
                  << " ang:" << (x.angle()) << std::endl; // 两种元素访问方式均可
    }

    template <typename T>
    inline void printMat(const T &m, const std::string &info)
    {
        std::cout << info << "\n"
                  << m << endl;
    }

    template <typename T>
    inline void printVec(const T &v, const std::string &info)
    {
        // cout.precision(6);
        // cout.setf(ios::showpoint);
        std::cout << info << "[ ";

        for (int i = 0; i < v.size(); i++)
        {
            std::cout << v(i);
            if (i < v.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << " ]" << std::endl;
    }

/*  算法*/
#if 0
    inline void PointCloudRegistrationSVD(Eigen::MatrixXd &source, Eigen::MatrixXd &target, Eigen::Matrix3d &rotationMat, Eigen::Vector3d &translationVec)
    {
        if (source.rows() != target.rows())
        {
            std::cout << "要求两个点集中点的个数相等" << std::endl;
            exit(1);
        }
        // 1、计算质心
        Eigen::RowVector3d meanVector1 = source.colwise().mean(); // 每列求均值
        Eigen::RowVector3d meanVector2 = target.colwise().mean();
        // 2、去质心
        source.rowwise() -= meanVector1; // 去质心坐标
        target.rowwise() -= meanVector2;
        // 3、构建协方差矩阵
        Eigen::Matrix3d covMat;
        covMat = (source.transpose() * target) / int(source.rows());
        // 4、SVD分解求旋转矩阵
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Matrix3d U = svd.matrixU();

        // 5、 计算旋转
        rotationMat = V * U.transpose(); // 这样写才是对的，网上很多代码都不对******************

        if (rotationMat.determinant() < 0)
        {
            rotationMat *= -1;
        };

        // 【5】计算旋转 和平移矩阵 R  和 t,  R= V *M* UT  =================================
        // double det = (U*V.transpose()).determinant();

        // cout << "det = " << det << endl;
        // Eigen::Matrix3d M;
        // M << 1, 0, 0, 0, 1, 0, 0, 0, det;

        // Eigen::Matrix3d R_ = V * M* (U.transpose());

        // cout << "R -------\n" << R_ << endl;

        // 6、求平移向量t =  p' - R * p
        translationVec = meanVector2 - (rotationMat * meanVector1.transpose()).transpose();
        // 或 translationVec = meanVector2 - meanVector1* rotationMat.transpose();
    }
#endif

}

#endif
