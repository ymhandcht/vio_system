#pragma once
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <src/vio_system_estimator/include/vio_system_estimator/factor/integration_base.h>
using namespace std;
using namespace Eigen;

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, Eigen::Matrix<double, 7, 1>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };
        map<int, Eigen::Matrix<double, 7, 1>> points;
        double t;
        Matrix3d R;
        Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};

struct SFMFeature
{
    int id;
    bool state = false; //表示是否赋值深度
    double position[3];
    vector<pair<int, Vector2d>> observation;    //归一化坐标
};