#include "Types/SystemParams.h"

namespace ORB_SLAM3
{
    SystemParams* SystemParams::mSystemParams = nullptr;

    SystemParams::SystemParams()
    {
        mSystemParams = nullptr;
    }

    SystemParams *SystemParams::GetParams()
    {
        if (mSystemParams == nullptr)
            mSystemParams = new SystemParams();
        return mSystemParams;
    }

    void SystemParams::SetParams(const std::string &strConfigFile)
    {
        std::cout << "Loading system parameters from: " << strConfigFile << std::endl;
        try
        {
            mConfig = YAML::LoadFile(strConfigFile);
        }
        catch (YAML::BadFile &e)
        {
            std::cerr << "Error loading configuration file: " << e.what() << std::endl;

            // [TODO] Wheter to exit or not?
        }

        // set parameters
        try
        {
            general.mode_of_operation = static_cast<general::ModeOfOperation>(mConfig["general"]["mode_of_operation"].as<int>());
            general.env_database = mConfig["general"]["env_database"].as<std::string>();

            markers.impact = mConfig["markers"]["impact"].as<float>();

            pointcloud.distance_thresh = std::make_pair(mConfig["pointcloud"]["distance_thresh"]["near"].as<float>(),
                                                        mConfig["pointcloud"]["distance_thresh"]["far"].as<float>());

            seg.pointclouds_thresh = mConfig["seg"]["pointclouds_thresh"].as<unsigned int>();
            seg.plane_association_thresh = mConfig["seg"]["plane_association_thresh"].as<float>();
            seg.plane_point_dist_thresh = mConfig["seg"]["plane_point_dist_thresh"].as<float>();
            seg.plane_facing_dot_thresh = mConfig["seg"]["plane_facing_dot_thresh"].as<float>();
            seg.ransac.max_planes = mConfig["seg"]["ransac"]["max_planes"].as<unsigned int>();
            seg.ransac.distance_thresh = mConfig["seg"]["ransac"]["distance_thresh"].as<float>();
            seg.ransac.max_iterations = mConfig["seg"]["ransac"]["max_iterations"].as<unsigned int>();

            geo_seg.downsample_leaf_size = mConfig["geo_seg"]["downsample_leaf_size"].as<float>();

            sem_seg.downsample_leaf_size = mConfig["sem_seg"]["downsample_leaf_size"].as<float>();
            sem_seg.prob_thresh = mConfig["sem_seg"]["prob_thresh"].as<float>();
            sem_seg.max_step_elevation = mConfig["sem_seg"]["max_step_elevation"].as<float>();
            sem_seg.max_tilt_wall = mConfig["sem_seg"]["max_tilt_wall"].as<float>();
            sem_seg.max_tilt_ground = mConfig["sem_seg"]["max_tilt_ground"].as<float>();
            sem_seg.min_votes = mConfig["sem_seg"]["min_votes"].as<float>();

            room_seg.method = static_cast<room_seg::Method>(mConfig["room_seg"]["method"].as<int>());

        }
        catch (YAML::Exception &e)
        {
            std::cerr << "Error loading system parameters. Make sure all parameters are defined properly: " << e.what() << std::endl;
        }
    }
}
