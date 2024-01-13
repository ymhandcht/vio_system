#include <feature_tracker.h>

int FeatureTracker::n_id = 0;
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void FeatureTracker::rejectWithF()
{
    //基础矩阵要用8点法，至少需要8个点
    if (forw_pts.size() > 8)
    {
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());//去畸变后的特征点
            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }
        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }   
}
//setMask函数实现把特征点按照追踪次数排序,然后在mask对应位置画实心黑色圆
//之后提取特征点就不在这些区域提,保证了特征点的均匀
void FeatureTracker::setMask()
{
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    //保留被追踪次数多的特征点
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts.size(); i++)
    {
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));
    }
    sort(cnt_pts_id.begin(), cnt_pts_id.end(),[](const pair<int, pair<cv::Point2f, int>>&a, const pair<int, pair<cv::Point2f, int>>& b)
    {
         return a.first > b.first;
    }
    );
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}


void FeatureTracker::addPoints()
{
    //把新提取的特征点存进维护特征点的容器,特征点🆔id置为-1
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}


void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //这里虽然是cur_pts实际上是最新帧图像的特征点
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    }
    //计算特征点速度,为了后面优化传感器时间延时td
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                auto it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;//得到的是归一化平面的速度
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
        {
            for (unsigned int i = 0; i < cur_pts.size(); i++)
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    prev_un_pts_map = cur_un_pts_map;
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
        {
            ids[i] = n_id++;
        }
        return true;
    }
    else
        return false;
}


void FeatureTracker::readIntrinsicParameter(const string & calib_file)
{
    this->m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}
void FeatureTracker::readImage(const cv::Mat & _img, double _cur_time)
{
    cv::Mat img;
    cur_time = _cur_time;
    img = _img;
    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }
    //记录当前图像特征点的容器清空
    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        vector<uchar> status;
        vector<float> err;
        //根据前一帧图像信息进行光流追踪
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        for (int i = 0; i < forw_pts.size(); i++)
        {
            if (status[i] && !inBorder(forw_pts[i]))
            {
                status[i] = 0;  //跟踪上但是超出图像边界的特征点，状态位也置为0
            }
        }
        //对容器进行瘦身，把跟踪失败的点相关的容器从容器里面清除
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
    }
    //把追踪上的特征点跟踪次数加1
    for (auto &n : track_cnt)
        n++;
    if (PUB_THIS_FRAME)
    {
        rejectWithF();  //通过基础矩阵去除外点
        //设置mask使得提取特征点均匀化
        setMask();
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());    //额外提取的特征点数目
        if (n_max_cnt > 0)
        {
            cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
        }
        else 
            n_pts.clear();
        addPoints();
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    //去畸变并计算特征点速度
    undistortedPoints();
    prev_time = cur_time;
}