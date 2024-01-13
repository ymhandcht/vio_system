#include <src/vio_system_estimator/include/vio_system_estimator/utility/utility.h>

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();//枢纽帧重力
    Eigen::Vector3d ng2{0, 0, 1};//第0帧重力认为载体竖直向下
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();//找到从ng1到ng2的R
    double yaw = Utility::R2ypr(R0).x();//从R中分解出yaw的分量 目的保证第零帧的yaw是0
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    return R0;
}