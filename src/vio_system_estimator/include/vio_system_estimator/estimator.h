#include <eigen3/Eigen/Dense>
#include <feature_manager.h>
#include <vector>
#include <std_msgs/Header.h>
#include <src/vio_system_estimator/include/vio_system_estimator/factor/projection_factor.h>
#include <src/vio_system_estimator/include/vio_system_estimator/factor/projection_td_factor.h>
#include <src/vio_system_estimator/include/vio_system_estimator/factor/integration_base.h>
#include <src/vio_system_estimator/include/vio_system_estimator/initial/initial_alignment.h>
#include <src/vio_system_estimator/include/vio_system_estimator/factor/marginalization_factor.h>
using namespace Eigen;
using namespace std;


class Estimator
{
public:
    Estimator();
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void setParameter();
    void clearState();
    void processImage(const map<int, Eigen::Matrix<double, 7, 1>> &image, const std_msgs::Header &header);
    bool initialStructure();
    
    Eigen::Matrix3d ric; 
    Eigen::Vector3d tic;
    FeatureManager f_manager;
    Matrix3d Rs[(WINDOW_SIZE + 1)];

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Eigen::Vector3d g;
    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];
    std::vector<Vector3d> key_poses;
    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];
    bool first_imu = true;
    Vector3d acc_0, gyr_0;
    // map<double, ImageFrame> all_image_frame;
    vector<double *> last_marginalization_parameter_blocks;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    double initial_timestamp;
    bool failure_occur;
    int frame_count;
    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    IntegrationBase *tmp_pre_integration;

    map<double, ImageFrame> all_image_frame;
    MarginalizationInfo *last_marginalization_info;



};