#include <ros/ros.h>
#include <parameters.h>
#include <feature_tracker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

ros::Publisher pub_restart, pub_img, pub_match;

FeatureTracker trackerData;
bool first_image_flag = true;
double first_image_time;
double last_image_time;
int pub_count = 0;
bool init_pub = false;


void img_callback(const sensor_msgs::ImageConstPtr & img_msg)
{
    if (first_image_flag)
    {
        //第一张图像不能进行匹配
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    //检测图像流异常
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue!reset the feature tracker");
        first_image_flag = true;
        last_image_time = 0.0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    //正常图像帧处理
    last_image_time = img_msg->header.stamp.toSec();
    //控制前端向后端发布频率，降低后端优化压力
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    //先把ros消息格式转化成opencv,8UC1格式的图像数据需要进行特殊处理
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        //灰度图像，8为无符号单通道
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;   //描述数据存储格式 大端序还是小端序
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;  //可视化显示的图像

    trackerData.readImage(ptr->image, img_msg->header.stamp.toSec());
    //更新新提取的特征点id
    for (unsigned int i = 0; ; i++)
    {
        bool completed = false;
        completed |= trackerData.updateID(i);
        if (!completed)
            break;
    }
    //把光流追踪的数据发出
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        auto &un_pts = trackerData.cur_un_pts;  //当前帧去畸变后的归一化平面坐标
        auto &cur_pts = trackerData.cur_pts;
        auto &ids = trackerData.ids;
        auto &pts_velocity = trackerData.pts_velocity;

        for (unsigned int i = 0; i < ids.size(); i++)
        {
            //把每个id特征点存入发出的消息格式里面
            if (trackerData.track_cnt[i] > 1)
            {
                geometry_msgs::Point32 p;
                p.x = un_pts[i].x;
                p.y = un_pts[i].y;
                p.z = 1;

                feature_points->points.push_back(p);
                id_of_point.values.push_back(ids[i]);
                u_of_point.values.push_back(cur_pts[i].x);
                v_of_point.values.push_back(cur_pts[i].y);
                velocity_x_of_point.values.push_back(pts_velocity[i].x);
                velocity_x_of_point.values.push_back(pts_velocity[i].y);
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(v_of_point);

        if (!init_pub)
        {
            init_pub = true;
        }
        else
            pub_img.publish(feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = ptr->image;
            cv::cvtColor(show_img, img, CV_GRAY2RGB);
            //把提取和追踪的点在图像上标记出来
            for (unsigned int i = 0; i < trackerData.cur_pts.size(); i++)
            {
                double len = std::min(1.0, 1.0 * trackerData.track_cnt[i] / WINDOW_SIZE);
                //在图像上稳定的特征点是红色，不稳定的是蓝色
                cv::circle(img, trackerData.cur_pts[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len),2);
            }
            pub_match.publish(ptr->toImageMsg());
        }
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    readParameters(n);
    trackerData.readIntrinsicParameter(CAM_NAME);   //生成相机模型
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    ros::spin();
    return 0;
}