#include <parameters.h>

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int ROW;
int COL;
int FOCAL_LENGTH;
bool PUB_THIS_FRAME;
std::string CAM_NAME;

template<typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded" << name << ":" << ans);
    }
    else
    {
        ROS_ERROR_STREAM("failed to load" << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "error:wrong path to settings" << std::endl;
    }
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    CAM_NAME = config_file;
    WINDOW_SIZE = 20;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;
    if (FREQ == 0)
        FREQ = 100;
    fsSettings.release();
}
