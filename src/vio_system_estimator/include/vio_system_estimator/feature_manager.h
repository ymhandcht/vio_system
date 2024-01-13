#pragma once

#include <eigen3/Eigen/Dense>
#include <list>
#include <vio_parameters.h>
using namespace std;
using namespace Eigen;

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);//归一化坐标
        uv.x() = _point(3);
        uv.y() = _point(4);//像素坐标
        velocity.x() = _point(5); 
        velocity.y() = _point(6); //特征点速度
        cur_td = td;//时间戳
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class FeaturePerId
{
public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;//在每一帧属性 包括像素坐标、归一化坐标、特征点速度

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail; 

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
public:
    FeatureManager(Eigen::Matrix3d _Rs[]);
    void setRic(Eigen::Matrix3d _ric);
    void clearState();
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    bool addFeatureCheckParallax(int frame_count, const map<int, Eigen::Matrix<double, 7, 1>>& image, double td);

    list<FeaturePerId> feature;
    int last_track_num;

private:
    Eigen::Matrix3d ric;
};