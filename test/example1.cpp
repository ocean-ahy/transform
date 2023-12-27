#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../transform3d.hpp"
using namespace std;
using namespace tf3d;


// 测试四元数  轴角之间变换
void t1_quat_axis()
{
    cout << "\n  -----t1_quat_axis  ---\n" <<  endl;

    Eigen::Quaterniond  Q1(0,  0.707, -0.707, 0); // wxyz
    
    printQuaternion(Q1, "Q1= "); 
    printRPY(Quat2euler(Q1), "RPY");
    Eigen::Matrix3d M = Quat2Mat(Q1);
    printMat( M, "M=");

    Eigen::Vector3d V1, V2, V3;
    V1 = Mat2AxisAngle(M);
    V2 = Quat2AxisAngle(Q1);

    printVec(V1, "V1");
    printVec(V2, "V2");

    Eigen::AngleAxisd V(M);
    V3 = V.axis() * V.angle() * TO_DEGREE;
    printVec(V3, "V3");


    cout <<"======================\n" <<endl;
    Eigen::Matrix3d R = Euler2Mat(180, -0, -89.9827);
    R = M;

    // R << 0,1,0,
    //      1,0,0,
    //      0,0,-1;

    Eigen::Quaterniond Q6(R);
    printQuaternion(Q6,"Q6");
    Q6.normalize();
    printQuaternion(Q6,"Q66");
    
    double ang = acos((R.trace() - 1.0) / 2);

    double ang2 = 2 * acos(Q6.w());

  

    Eigen::Vector3d V4, V5;
    V = R;
    V4 = V.axis() * V.angle() * TO_DEGREE;

    cout << ang << "   ,  "<< ang2<< "   ,  " << V.angle() << " |  "  << V.axis().transpose() <<endl;
    printVec(V4, "V4");
    printVec(Mat2AxisAngle(R), "V44");

}

void t2()
{

     Eigen::Vector3d rpy(-178.548, 0.360, -89.994);

    tf3d::printRPY(rpy, "  ");
    
    

    Eigen::Matrix3d M = Euler2Mat(rpy);
    printQuaternion(Mat2Quat(M), "q= ");

    cout << Mat2Quat(M).coeffs().transpose() << endl;

    Eigen::Matrix4d H = ComposeH(M, Eigen::Vector3d(2,4,7));

    Eigen::Quaterniond q(M), q1(M);
    q1.w() += 0.03;
    q1.normalize();

    Eigen::Quaterniond q3 = q*q1.inverse();

    cout << q3.x()/q.x() << "  " << q3.y()/q.y() << "  " << q3.z()/q.z() << "  "  <<endl;

    printQuaternion(q*q1.inverse(), "  ");
    printMat(Eigen::Matrix3d(q*q1.inverse()), "  ");
}

int main()
{
   
    t1_quat_axis();



    return 0;
}