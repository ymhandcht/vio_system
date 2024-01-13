#pragma once
#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

void readParameters(ros::NodeHandle &n);

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern int ESTIMATE_EXTRINSIC;
extern int ESTIMATE_TD;
extern std::string IMU_TOPIC;
extern double ROW, COL;
extern double TD;
extern double MIN_PARALLAX;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
const double CAMERA_HEIGHT = 0.2235;     //这个参数是相机安装高度，也认为是图像的深度信息
extern Eigen::Vector3d G;
extern Eigen::Matrix3d RIC;
extern Eigen::Vector3d TIC;
