#pragma once
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <opencv2/opencv.hpp>
#include <parameters.h>
using namespace std;
using namespace camodocal;

class FeatureTracker
{
public:
            FeatureTracker() {};
            void readIntrinsicParameter(const std::string &calib_file);
            void readImage(const cv::Mat & _img, double _cur_time);
            void rejectWithF();
            void setMask();
            void addPoints();
            void undistortedPoints();
            bool updateID(unsigned int i);
            camodocal::CameraPtr m_camera;
            double cur_time;
            double prev_time;
            cv::Mat cur_img, forw_img, prev_img;
            vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
            vector<cv::Point2f> prev_un_pts, cur_un_pts;
            vector<cv::Point2f> pts_velocity;
            vector<int> ids;
            vector<int> track_cnt;
            cv::Mat mask;
            vector<cv::Point2f> n_pts;//当前帧图像需要额外提取的特征点存储容器
            map<int, cv::Point2f> cur_un_pts_map;
            map<int, cv::Point2f> prev_un_pts_map;
            static int n_id;   //记录最新特征点id
            

};