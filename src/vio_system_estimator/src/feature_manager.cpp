#include <feature_manager.h>
#include <map>
#include <vector>

void FeatureManager::setRic(Eigen::Matrix3d _Rs)
{
    this->ric = _Rs;
}

void FeatureManager::clearState()
{
    this->feature.clear();
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //通过计算滑窗内倒数第二帧和倒数第三帧视差，判断倒数第二帧是否是关键帧
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;
    double u_j = p_j(0);
    double v_j = p_j(1);
    Vector3d p_i = frame_i.point;
    

    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    ans = max(ans, sqrt(du * du + dv * dv));

    return ans;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, Eigen::Matrix<double, 7, 1>>& image, double td)
{
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    //遍历所有特征点，并将特征点添加到feature里面
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second, td);
        int feature_id = id_pts.first;

        auto it = find_if(feature.begin(), feature.end(),[feature_id](const FeaturePerId &it)
                           {
                return it.feature_id == feature_id;
                            });
        //如果特征点容器里面没有这个特征点 把特征点添加进feature 也把对应的perframe信息添加
        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        //如果不是第一次出现的特征点，只把feature_per_frame添加
        else if(it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }
    //根据视差判断是否是关键帧
    if (frame_count < 2 || last_track_num < 20)
    {
        return true;
    }
    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
        it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
    
}