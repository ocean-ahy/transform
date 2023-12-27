#ifndef CV_TRANSFORM3D_HPP
#define CV_TRANSFORM3D_HPP


#include <iostream>
#include <string>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

using namespace std;

namespace cv_tf3d
{
    const double TO_DEGREE = 180.0 / CV_PI; // 转为度，乘的系数
    const double TO_RADIAN = CV_PI / 180.0; // 转为弧度，乘的系数

    enum AngType
    {
        DEGREE = 0,
        RADIAN,
    };

    template <typename T>
    inline T deg2rad(T x) { return (x * TO_RADIAN); }
    template <typename T>
    inline T rad2deg(T x) { return (x * TO_DEGREE); }

    inline cv::Mat RotX(double rx)
    {
        double ct = cos(deg2rad(rx));
        double st = sin(deg2rad(rx));
        cv::Mat_<double> mat(3, 3);
        mat << 1.0, 0.0, 0.0,
            0.0, ct, -st,
            0.0, st, ct;
        return mat;
    }

    inline cv::Mat RotY(double ry)
    {
        double ct = cos(deg2rad(ry));
        double st = sin(deg2rad(ry));
        cv::Mat_<double> mat(3, 3);
        mat << ct, 0.0, st,
            0.0, 1.0, 0.0,
            -st, 0.0, ct;
        return mat;
    }

    inline cv::Mat RotZ(double rz)
    {
        double ct = cos(deg2rad(rz));
        double st = sin(deg2rad(rz));
        cv::Mat_<double> mat(3, 3);
        mat << ct, -st, 0.0,
            st, ct, 0.0,
            0.0, 0.0, 1.0;
        return mat;
    }

    inline cv::Mat Euler2Mat(double rx, double ry, double rz, AngType flag = DEGREE)
    {
        if (flag != DEGREE)
        {
            rx = rad2deg(rx);
            ry = rad2deg(ry);
            rz = rad2deg(rz);
        }
        cv::Mat rotation_matrix = RotZ(rz) * RotY(ry) * RotX(rx);

        return rotation_matrix;
    }

    inline cv::Mat Euler2Mat(cv::Vec3d eularAngle, AngType flag = DEGREE)
    {
        double rx, ry, rz;
        rx = (flag == DEGREE) ? eularAngle[0] : rad2deg(eularAngle[0]);
        ry = (flag == DEGREE) ? eularAngle[1] : rad2deg(eularAngle[1]);
        rz = (flag == DEGREE) ? eularAngle[2] : rad2deg(eularAngle[2]);
        cv::Mat rotation_matrix = RotZ(rz) * RotY(ry) * RotX(rx);
        return rotation_matrix;
    }

    inline bool isRotatedMatrix(const cv::Mat &R)
    {
        cv::Mat tmp33 = R({0, 0, 3, 3});
        cv::Mat shouldBeIdentity;
        shouldBeIdentity = tmp33.t() * tmp33; // 旋转矩阵的转置矩阵是它的逆矩阵，逆矩阵 * 矩阵 = 单位矩阵
        cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
        return cv::norm(I, shouldBeIdentity) < 1e-6;
    }

    inline cv::Vec3d Mat2Euler(const cv::Mat &R)
    {
        double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
        bool singular = sy < 1e-6; // If

        double x, y, z;
        if (!singular)
        {
            x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
            y = atan2(-R.at<double>(2, 0), sy);
            z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        }
        else
        {
            x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
            y = atan2(-R.at<double>(2, 0), sy);
            z = 0;
        }

        return (cv::Vec3d(x, y, z) * TO_DEGREE);
    }

    // 反对称矩阵
    inline cv::Mat SkewMat(cv::Vec3d k)
    {
        cv::Mat K = (cv::Mat_<double>(3, 3) << 0.0, -k[2], k[1],
                     k[2], 0.0, -k[0],
                     -k[1], k[0], 0.0);
        return K;
    }

    // 轴角
    inline cv::Mat AxisAngle2Mat(double x, double y, double z, AngType flag = DEGREE)
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

        // cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat_<double> R(3, 3);
        R << kx * kx * vt + ct, kx * ky * vt - kz * st, kx * kz * vt + ky * st,
            kx * ky * vt + kz * st, ky * ky * vt + ct, ky * kz * vt - kx * st,
            kx * kz * vt - ky * st, ky * kz * vt + kx * st, kz * kz * vt + ct;
        return R;
    }

    inline cv::Mat AxisAngle2Mat(cv::Vec3d axis, AngType flag = DEGREE)
    {
        // double ang = sqrt(axis[0]*axis[0] + axis[1]*axis[1]+axis[2]*axis[2]);
        // axis = axis/ang;

        // if (flag == DEGREE)
        //     ang = deg2rad(ang);

        cv::Mat R;
        if (flag == DEGREE)
            axis = axis * TO_RADIAN;
        cv::Rodrigues(axis, R);
        return R;
    }

    inline cv::Mat RotVector2Mat(double rx, double ry, double rz, AngType flag = DEGREE)
    {
        // 计算角度
        double t = sqrt(rx * rx + ry * ry + rz * rz);

        // 单位化
        double kx = rx / t;
        double ky = ry / t;
        double kz = rz / t;

        if (flag == DEGREE)
            t = deg2rad(t);

        cv::Mat_<double> rotv(1, 3);
        rotv.at<double>(0, 0) = kx * t;
        rotv.at<double>(0, 1) = ky * t;
        rotv.at<double>(0, 2) = kz * t;
        cv::Mat R;
        cv::Rodrigues(rotv, R);
        return R;
    }

    // axis 轴角，角度为°
    inline bool Mat2AxisAngle(cv::Mat R, cv::Vec3d &axis)
    {
        if (!isRotatedMatrix(R))
            return false;

        double ang = acos((R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2) - 1.0) / 2);
        double k = 1.0 / (2 * sin(ang));
        axis[0] = k * (R.at<double>(2, 1) - R.at<double>(1, 2));
        axis[1] = k * (R.at<double>(0, 2) - R.at<double>(2, 0));
        axis[2] = k * (R.at<double>(1, 0) - R.at<double>(0, 1));

        ang *= TO_DEGREE; // 转为°
        axis *= ang;
        return true;
    }

    inline cv::Vec3d Mat2AxisAngle(cv::Mat R)
    {
        double ang = acos((R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2) - 1.0) / 2);
        double k = 1.0 / (2 * sin(ang));
        cv::Vec3d axis;
        axis[0] = k * (R.at<double>(2, 1) - R.at<double>(1, 2));
        axis[1] = k * (R.at<double>(0, 2) - R.at<double>(2, 0));
        axis[2] = k * (R.at<double>(1, 0) - R.at<double>(0, 1));
        ang *= TO_DEGREE; // 转为°
        axis *= ang;
        return axis;
    }

    inline cv::Mat ComposeH(const cv::Mat &R, const cv::Vec3d &P)
    {
        cv::Mat_<double> H = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), P(0),
                              R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), P(1),
                              R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), P(2),
                              0, 0, 0, 1.0);

        return H;
    }

    inline cv::Mat ComposeH(const cv::Mat &R, const cv::Mat &P)
    {
        cv::Mat H;
        cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                               R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                               R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
                               0, 0, 0);
        cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << P.at<double>(0, 0),
                               P.at<double>(1, 0),
                               P.at<double>(2, 0),
                               1.0);
        cv::hconcat(R1, T1, H); // 矩阵拼接
        return H;
    }

    inline cv::Mat Mat2H(cv::Mat R)
    {
        return ComposeH(R, cv::Vec3d(0.0, 0.0, 0.0));
    }

    inline cv::Mat GetRotFromHomo(cv::Mat H)
    {
        return H({0, 0, 3, 3});
    }

    inline cv::Vec3d GetPosFromHomo(cv::Mat H)
    {
        cv::Vec3d P(H.at<double>(0, 3), H.at<double>(1, 3), H.at<double>(2, 3));
        return P;
    }

    inline cv::Mat HDecompose(cv::Mat H,
                       cv::Mat &R,
                       cv::Vec3d &P)
    {
        R = GetRotFromHomo(H);
        P = GetPosFromHomo(H);
        return R;
    }

    inline cv::Mat HDecompose(cv::Mat H,
                       cv::Mat &R,
                       cv::Mat &P)
    {
        cv::Rect R_rect(0, 0, 3, 3);
        cv::Rect T_rect(3, 0, 1, 3);
        R = H(R_rect);
        P = H(T_rect);
        return R;
    }

    inline cv::Mat PoseRPY2H(double pose[6], AngType flag = DEGREE)
    {
        cv::Vec3d p(pose[0], pose[1], pose[2]);
        cv::Vec3d eular(pose[3], pose[4], pose[5]);
        return ComposeH(Euler2Mat(eular, flag), p);
    }

    inline cv::Mat PoseRPY2H(cv::Vec6d pose, AngType flag = DEGREE)
    {

        cv::Vec3d p(pose(0), pose(1), pose(2));
        cv::Vec3d eular(pose(3), pose(4), pose(5));
        return ComposeH(Euler2Mat(eular, flag), p);
    }

    inline cv::Mat PoseRPY2H(double x,
                      double y,
                      double z,
                      double rx,
                      double ry,
                      double rz,
                      AngType flag = DEGREE)
    {
        cv::Vec3d p(x, y, z);
        cv::Vec3d eular(rx, ry, rz);
        return ComposeH(Euler2Mat(eular, flag), p);
    }

    inline cv::Mat PoseAxis2H(double pose[6], AngType flag = DEGREE)
    {
        cv::Vec3d p(pose[0], pose[1], pose[2]);
        cv::Vec3d axis(pose[3], pose[4], pose[5]);
        return ComposeH(AxisAngle2Mat(axis, flag), p);
    }

    inline cv::Mat PoseAxis2H(cv::Vec6d pose, AngType flag = DEGREE)
    {

        cv::Vec3d p(pose(0), pose(1), pose(2));
        cv::Vec3d axis(pose(3), pose(4), pose(5));
        return ComposeH(AxisAngle2Mat(axis, flag), p);
    }

    inline cv::Mat PoseAxis2H(double x,
                       double y,
                       double z,
                       double rx,
                       double ry,
                       double rz,
                       AngType flag = DEGREE)
    {
        cv::Vec3d p(x, y, z);
        cv::Vec3d axis(rx, ry, rz);

        return ComposeH(AxisAngle2Mat(axis, flag), p);
    }

    inline cv::Vec6d H2PoseRPY(const cv::Mat &H, AngType flag = DEGREE)
    {
        cv::Vec6d pose;
        cv::Vec3d p = GetPosFromHomo(H);
        pose(0) = p[0];
        pose(1) = p[1];
        pose(2) = p[2];
        cv::Vec3d rpy = Mat2Euler(GetRotFromHomo(H));
        if (flag == RADIAN)
            rpy *= TO_RADIAN;
        pose(3) = rpy[0];
        pose(4) = rpy[1];
        pose(5) = rpy[2];

        return pose;
    }

    inline bool H2PoseRPY(const cv::Mat &H, cv::Vec6d &pose, AngType flag = DEGREE)
    {
        cv::Vec3d p = GetPosFromHomo(H);
        pose(0) = p[0];
        pose(1) = p[1];
        pose(2) = p[2];
        cv::Vec3d rpy = Mat2Euler(GetRotFromHomo(H));
        if (flag == RADIAN)
            rpy *= TO_RADIAN;
        pose(3) = rpy[0];
        pose(4) = rpy[1];
        pose(5) = rpy[2];

        return true;
    }

    inline cv::Vec6d H2PoseAxis(const cv::Mat &H, AngType flag = DEGREE)
    {
        cv::Vec6d pose;
        cv::Vec3d p = GetPosFromHomo(H);
        pose(0) = p[0];
        pose(1) = p[1];
        pose(2) = p[2];
        cv::Vec3d axis = Mat2AxisAngle(GetRotFromHomo(H));
        if (flag == RADIAN)
            axis *= TO_RADIAN;
        pose(3) = axis[0];
        pose(4) = axis[1];
        pose(5) = axis[2];

        return pose;
    }

    inline bool H2PoseAxis(const cv::Mat &H, cv::Vec6d &pose, AngType flag = DEGREE)
    {
        cv::Vec3d p = GetPosFromHomo(H);
        pose(0) = p[0];
        pose(1) = p[1];
        pose(2) = p[2];
        cv::Vec3d axis = Mat2AxisAngle(GetRotFromHomo(H));
        if (flag == RADIAN)
            axis *= TO_RADIAN;
        pose(3) = axis[0];
        pose(4) = axis[1];
        pose(5) = axis[2];

        return true;
    }

    inline cv::Vec3d Euler2RotVec(cv::Vec3d euler, AngType flag = DEGREE)
    {
        cv::Mat R = Euler2Mat(euler, flag);
        cv::Vec3d rotVec = Mat2AxisAngle(R);

        return ((flag == RADIAN) ? rotVec * TO_RADIAN : rotVec);
    }

    inline cv::Vec3d RotVec2Euler(cv::Vec3d rot_vec, AngType flag = DEGREE)
    {
        cv::Mat R = AxisAngle2Mat(rot_vec, flag);
        cv::Vec3d euler = Mat2Euler(R);
        return ((flag == RADIAN) ? euler * TO_RADIAN : euler);
    }

    // 四元数--------------

    inline cv::Mat Quat2Mat(const cv::Vec4d &q)
    {
        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        double q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
        double q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
        double q1q2 = q1 * q2, q1q3 = q1 * q3;
        double q2q3 = q2 * q3;
        // JPL 左手系，与右手系互为转至
        //  cv::Mat mat = (cv::Mat_<double>(3, 3) << (q0q0 + q1q1 - q2q2 - q3q3), 2 * (q1q2 + q0q3), 2 * (q1q3 - q0q2),
        //      2 * (q1q2 - q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2 * (q2q3 + q0q1),
        //      2 * (q1q3 + q0q2), 2 * (q2q3 - q0q1), (q0q0 - q1q1 - q2q2 + q3q3));

        // Hamilton 右手系
        cv::Mat mat = (cv::Mat_<double>(3, 3) << (1 - 2 * (q2q2 + q3q3)), 2 * (q1q2 - q0q3), 2 * (q1q3 + q0q2),
                       2 * (q1q2 + q0q3), 1 - 2 * (q1q1 + q3q3), 2 * (q2q3 - q0q1),
                       2 * (q1q3 - q0q2), 2 * (q2q3 + q0q1), (1 - 2 * (q1q1 + q2q2)));

        return mat;
    }

    inline cv::Mat PoseQuat2H(cv::Vec<double, 7> &pose, std::string seq = "wxyz")
    {
        cv::Vec4d quaternionVec(pose(3), pose(4), pose(5), pose(6)); // 读取存储的四元数
        cv::Vec3d p(pose(0), pose(1), pose(2));
        cv::Vec4d q(0, 0, 0, 1); // w x y z顺序初始化
        if (seq == "wxyz")
        {
            q = quaternionVec;
        }
        else if (seq == "xyzw")
        {
            q[1] = quaternionVec[0];
            q[2] = quaternionVec[1];
            q[3] = quaternionVec[2];
            q[0] = quaternionVec[3];
        }

        return ComposeH(Quat2Mat(q), p);
    }

    inline cv::Mat PoseQuat2H(cv::Mat pose, std::string seq = "wxyz")
    {
        CV_Assert(pose.total() == 7);

        cv::Vec4d quaternionVec = pose({3, 0, 4, 1}); // 读取存储的四元数
        cv::Vec3d p(pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2));
        cv::Vec4d q(0, 0, 0, 1); // w x y z顺序初始化
        if (seq == "wxyz")
        {
            q = quaternionVec;
        }
        else if (seq == "xyzw")
        {
            q[1] = quaternionVec[0];
            q[2] = quaternionVec[1];
            q[3] = quaternionVec[2];
            q[0] = quaternionVec[3];
        }

        return ComposeH(Quat2Mat(q), p);
    }

    inline cv::Vec4d AxisAngle2Quat(cv::Vec3d axis, AngType flag = DEGREE)
    {
        double ang = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);

        axis = axis / ang;
        if (flag == DEGREE)
            ang *= TO_RADIAN;

        cv::Vec4d q;
        q[0] = cos(ang / 2);
        q[1] = axis[0] * sin(ang / 2);
        q[2] = axis[1] * sin(ang / 2);
        q[3] = axis[2] * sin(ang / 2);
        return q;
    }

    inline cv::Vec3d Quat2Axis(cv::Vec4d q)
    {
        // w x y z
        // normlize
        double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]+ q[3] * q[3]);
        q = q / norm;

        double theta = acos(q[0]) * 2;
        double kx = q[1] / sin(theta / 2);
        double ky = q[2] / sin(theta / 2);
        double kz = q[3] / sin(theta / 2);
        return (cv::Vec3d(kx, ky, kz) * theta* TO_DEGREE);
    }

    inline cv::Vec4d Mat2Quat(const cv::Mat &R)
    {
        cv::Vec3d axis = Mat2AxisAngle(R);
        return AxisAngle2Quat(axis);
    }

    inline cv::Vec3d Quat2Eular(cv::Vec4d q)
    {
        return Mat2Euler(Quat2Mat(q));
    }

    inline cv::Vec4d Eular2Quat(cv::Vec3d rpy) { return Mat2Quat(Euler2Mat(rpy)); }


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
        std::cout << info << std::endl;

        // cv::Vec3d vv;
        // vv.channels

        for (int i = 0; i < v.channels; i++)
        {
            std::cout << v(i) << " , ";
        }

        std::cout << std::endl;
    }

}

#endif // CV_TRANSFORM3D_HPP