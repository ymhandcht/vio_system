#include <ros/ros.h>
#include <vio_parameters.h>
#include <condition_variable>
#include <estimator.h>
#include <thread>
#include <mutex>
#include <src/vio_system_estimator/include/vio_system_estimator/utility/visualization.h>

Estimator estimator;
std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;

int sum_of_wait = 0;

double last_imu_t = 0;
std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;


double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_imu = true;
bool init_feature = true;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = false;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};
    //利用imu高频特性更新位姿
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    //通过陀螺仪积分得到变化的角度，角度经过四元数近似转换得到旋转矩阵
    //旋转矩阵补偿到上次的姿态，得到当前时刻姿态
    //Rw1 = Rw0 * R01
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);   
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

std::vector<std::pair<std::vector<sensor_msgs::ImuPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()//为了获得对齐的imu 图像 和光纤陀螺仪数据
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    // std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuPtr>, sensor_msgs::PointCloudConstPtr>, my_custom_msgs::MyposeConstPtr>> measurements_result;
    int index = 0;  //用来记录滑动加权滤波的索引
    int imu_window_size = 7;    //imu 滤波窗口长度
    std::vector<sensor_msgs::ImuPtr> imu_window(imu_window_size);  //滑动滤波中存储数据

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;
        //Imu数据   ************
        //image数据             *    *    *     *    *
        //这个情况属于imu数据还没来
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }
        //imu数据          ************
        //image数据   *    *    *
        //这种情况扔掉一些image帧
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        //imu:    **************************
        //image:    *   *   *   *   *   *   *   *   *   *   
        //aglig:           **************************   此时需要将image数据pop掉一部分
        // if (agilelight_buf.front()->head.stamp.toSec() > feature_buf.front()->header.stamp.toSec())
        // {
        //     feature_buf.pop();
        //     continue;
        // }
        //imu:    **************************
        //image:    *   *   *   *   *   *   *   *   *   *   
        //aglig:   **************************   此时可行
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();
        std::vector<sensor_msgs::ImuPtr> filter_imus;
        // my_custom_msgs::Mypose agiledata;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            /*****************************************begin**************************************************/
            if (index < imu_window_size)    //还没填充满滑窗
            {
                sensor_msgs::ImuConstPtr tmp_const_imu_ptr = imu_buf.front();   
                sensor_msgs::ImuPtr tmp_imu_ptr = boost::make_shared<sensor_msgs::Imu> (*tmp_const_imu_ptr);
                imu_window[index] = tmp_imu_ptr;
                filter_imus.emplace_back(tmp_imu_ptr);
                imu_buf.pop();
                index++;
            }
            else
            {
                //滑窗填满后对加速度进行加权滑动滤波
                 double sum_accx = 0.0;
                 double sum_accy = 0.0;
                 double sum_accz = 0.0;
                 //对角速度也进行滑动滤波
                 double sum_angleVelocityX = 0.0;
                 double sum_angleVelocityY = 0.0;
                 double sum_angleVelocityZ = 0.0;
                for (int i = 1; i < imu_window_size; i++)
                {
                    //把新的数据滑到窗口里面并且进行累加
                    imu_window[i - 1] = imu_window[i];
                    sum_accx += imu_window[i]->linear_acceleration.x * i;
                    sum_accy += imu_window[i]->linear_acceleration.y * i;
                    sum_accz += imu_window[i]->linear_acceleration.z * i;

                    sum_angleVelocityX += imu_window[i]->angular_velocity.x * i;
                    sum_angleVelocityY += imu_window[i]->angular_velocity.y * i;
                    sum_angleVelocityZ += imu_window[i]->angular_velocity.z * i;
                }
                sensor_msgs::ImuConstPtr tmp_const_imu_ptr = imu_buf.front();   //把最前面的imuconstptr变成imuptr
                sensor_msgs::ImuPtr tmp_imu_ptr = boost::make_shared<sensor_msgs::Imu> (*tmp_const_imu_ptr);
                sum_accx += tmp_imu_ptr->linear_acceleration.x * imu_window_size;
                sum_accy += tmp_imu_ptr->linear_acceleration.y * imu_window_size;
                sum_accz += tmp_imu_ptr->linear_acceleration.z * imu_window_size;

                sum_angleVelocityX += tmp_imu_ptr->angular_velocity.x * imu_window_size;
                sum_angleVelocityY += tmp_imu_ptr->angular_velocity.y * imu_window_size;
                sum_angleVelocityZ += tmp_imu_ptr->angular_velocity.z * imu_window_size;

                //把最新的放到滑窗里面
                imu_window[imu_window_size - 1] = tmp_imu_ptr;
                double accx = sum_accx / (imu_window_size *(1 + imu_window_size) / 2);
                double accy = sum_accy / (imu_window_size *(1 + imu_window_size) / 2);
                double accz = sum_accz / (imu_window_size *(1 + imu_window_size) / 2);

                double anglevelx = sum_angleVelocityX / (imu_window_size *(1 + imu_window_size) / 2);
                double anglevely = sum_angleVelocityY / (imu_window_size *(1 + imu_window_size) / 2);
                double anglevelz = sum_angleVelocityZ / (imu_window_size *(1 + imu_window_size) / 2);
                //先把滤波的结果赋到filter_imus
                tmp_imu_ptr->linear_acceleration.x = accx;
                tmp_imu_ptr->linear_acceleration.y = accy;
                tmp_imu_ptr->linear_acceleration.z = accz;
                //添加阈值，避免噪声影响
                anglevelx = abs(anglevelx) < 0.035 ? 0.0 : anglevelx;
                anglevely = abs(anglevely) < 0.035 ? 0.0 : anglevely;
                anglevelz = abs(anglevelz) < 0.035 ? 0.0 : anglevelz;
                tmp_imu_ptr->angular_velocity.x = anglevelx;
                tmp_imu_ptr->angular_velocity.y = anglevely;
                tmp_imu_ptr->angular_velocity.z = anglevelz;

                filter_imus.emplace_back(tmp_imu_ptr);
                imu_buf.pop();
            } 
            /*****************************************end**************************************************/
        }
        //把紧挨着image后一帧的imu取出使用但是不pop掉 主要为了插值
        if (index < imu_window_size)    //还没填充满滑窗
            {
                sensor_msgs::ImuConstPtr tmp_const_imu_ptr = imu_buf.front();   
                sensor_msgs::ImuPtr tmp_imu_ptr = boost::make_shared<sensor_msgs::Imu> (*tmp_const_imu_ptr);
                filter_imus.emplace_back(tmp_imu_ptr);
            }
        else
            {
                //滑窗填满后对加速度进行加权滑动滤波
                 double sum_accx = 0.0;
                 double sum_accy = 0.0;
                 double sum_accz = 0.0;

                 double sum_angleVelocityX = 0.0;
                 double sum_angleVelocityY = 0.0;
                 double sum_angleVelocityZ = 0.0;
                for (int i = 1; i < imu_window_size; i++)
                {
                    //把新的数据滑到窗口里面并且进行累加
                    sum_accx += imu_window[i]->linear_acceleration.x * i;
                    sum_accy += imu_window[i]->linear_acceleration.y * i;
                    sum_accz += imu_window[i]->linear_acceleration.z * i;

                    sum_angleVelocityX += imu_window[i]->angular_velocity.x * i;
                    sum_angleVelocityY += imu_window[i]->angular_velocity.y * i;
                    sum_angleVelocityZ += imu_window[i]->angular_velocity.z * i;
                }
                sensor_msgs::ImuConstPtr tmp_const_imu_ptr = imu_buf.front();   //把最前面的imuconstptr变成imuptr
                sensor_msgs::ImuPtr tmp_imu_ptr = boost::make_shared<sensor_msgs::Imu> (*tmp_const_imu_ptr);
                sum_accx += tmp_imu_ptr->linear_acceleration.x * imu_window_size;
                sum_accy += tmp_imu_ptr->linear_acceleration.y * imu_window_size;
                sum_accz += tmp_imu_ptr->linear_acceleration.z * imu_window_size;

                sum_angleVelocityX += tmp_imu_ptr->angular_velocity.x * imu_window_size;
                sum_angleVelocityY += tmp_imu_ptr->angular_velocity.y * imu_window_size;
                sum_angleVelocityZ += tmp_imu_ptr->angular_velocity.z * imu_window_size;

                //把最新的放到滑窗里面
                double accx = sum_accx / (imu_window_size *(1 + imu_window_size) / 2);
                double accy = sum_accy / (imu_window_size *(1 + imu_window_size) / 2);
                double accz = sum_accz / (imu_window_size *(1 + imu_window_size) / 2);

                double anglevelx = sum_angleVelocityX / (imu_window_size *(1 + imu_window_size) / 2);
                double anglevely = sum_angleVelocityY / (imu_window_size *(1 + imu_window_size) / 2);
                double anglevelz = sum_angleVelocityZ / (imu_window_size *(1 + imu_window_size) / 2);
                //先把滤波的结果赋到filter_imus
                tmp_imu_ptr->linear_acceleration.x = accx;
                tmp_imu_ptr->linear_acceleration.y = accy;
                tmp_imu_ptr->linear_acceleration.z = accz;

                tmp_imu_ptr->angular_velocity.x = anglevelx;
                tmp_imu_ptr->angular_velocity.y = anglevely;
                tmp_imu_ptr->angular_velocity.z = anglevelz;
                filter_imus.emplace_back(tmp_imu_ptr);
            } 
        if (filter_imus.empty())
            ROS_WARN("no imu between two image");
        // measurements.emplace_back(IMUs, img_msg);
        measurements.emplace_back(filter_imus, img_msg);
        //找到和图像帧时间戳对齐的光纤陀螺仪数据
        //std::vector<std::pair<      std::pair<std::vector<sensor_msgs::ImuPtr>, 
        //sensor_msgs::PointCloudConstPtr>, my_custom_msgs::Mypose>> measurements_result;
        // my_custom_msgs::MyposeConstPtr agile_before;
        // while (agilelight_buf.front()->head.stamp.toSec() > feature_buf.front()->header.stamp.toSec())
        // {
        //     agile_before = agilelight_buf.front();
        //     agilelight_buf.pop();
        // }
        //先将image对应时刻的前一时刻的光纤陀螺仪数据作为真值，之后通过四元数插值计算更准确的姿态
        // measurements_result.emplace_back(make_pair(make_pair(filter_imus, img_msg), agile_before));
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();
    //互斥锁的自动上锁和解锁，大括号外面互斥锁失效，自动解锁
    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }

}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (init_feature)
    {
        init_feature = false;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while (!feature_buf.empty())
            feature_buf.pop();
        while (!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        last_imu_t = 0;
        current_time = -1;
    }
    return;
}

void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
        {
            return (measurements = getMeasurements()).size() != 0;
        });
        lk.unlock();

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;

            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec();
                //时间戳小于图像帧的直接进行预积分
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;

                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;

                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
                else
                {
                    //线性插值处理最后一帧imu数据
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;

                    double w1 = dt_2 / (dt_1 + dt_2);   //图像到imu最后一帧的时间差
                    double w2 = dt_1 / (dt_1 + dt_2);   //倒数第2帧和图像时间戳的时间差
                    dx = w1 * dx + w2 *imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;

                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));

                }
            }

            //处理图像  map的index是特征点id， value是特征点信息--->归一坐标 像素坐标 
            map<int, Eigen::Matrix<double, 7, 1>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int feature_id = img_msg->channels[0].values[i];
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id] = xyz_uv_velocity;
            }
            estimator.processImage(image, img_msg->header);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vio_system_estimator");
    ros::NodeHandle n("~");
    readParameters(n);
    estimator.setParameter();

    registerPub(n);
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    std::thread measurement_process{process};
    ros::spin();
    
    return 0;
}