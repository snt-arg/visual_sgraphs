#include <yaml-cpp/yaml.h>
#include <iostream>

#ifndef SYSTEMPARAMS_H
#define SYSTEMPARAMS_H

namespace ORB_SLAM3
{
    class SystemParams
    {
    public:
        static SystemParams *GetParams();
        void SetParams(const std::string &strConfigFile);

        // Structs for different categories of parameters
        struct general
        {
            // enum for mode of operation
            enum ModeOfOperation
            {
                SEM_GEO = 0,
                SEM = 1,
                GEO = 2
            };
            ModeOfOperation mode_of_operation = SEM_GEO;
            std::string env_database = "";
        } general;

        struct markers
        {
            float impact = 0.1f;
        } markers;

        struct pointcloud
        {
            std::pair<float, float> distance_thresh = std::make_pair(0.2f, 2.5f);
        } pointcloud;

        struct seg
        {
            unsigned int pointclouds_thresh = 200;
            float plane_association_thresh = 0.2f;
            float plane_point_dist_thresh = 0.2f;

            struct ransac
            {
                unsigned int max_planes = 2;
                float distance_thresh = 0.04f;
                unsigned int max_iterations = 600;
            } ransac;
        } seg;

        struct geo_seg
        {
            struct pointcloud
            {
                float downsample_leaf_size = 0.09f;
                struct outlier_removal
                {
                    float std_threshold = 1.0;
                    unsigned int mean_threshold = 50;
                } outlier_removal;
            } pointcloud;
        } geo_seg;

        struct sem_seg
        {
            float min_votes = 3.5;
            float prob_thresh = 0.5f;
            float conf_thresh = 0.5f;
            float max_tilt_wall = 0.3f;
            float max_tilt_ground = 0.2f;
            float max_step_elevation = 0.2f;

            struct pointcloud
            {
                float downsample_leaf_size = 0.03f;
                struct outlier_removal
                {
                    float std_threshold = 1.0;
                    unsigned int mean_threshold = 50;
                } outlier_removal;
            } pointcloud;
        } sem_seg;

        struct room_seg
        {
            enum Method
            {
                GEOMETRIC = 0,
                FREE_SPACE = 1,
                GNN = 2
            };
            Method method = FREE_SPACE;

            float plane_facing_dot_thresh = -0.8f;
            float room_center_distance_thresh = 1.5f;
            float walls_perpendicularity_thresh = 10.0;

            unsigned int min_cluster_vertices = 5;
            float marker_wall_distance_thresh = 3.0f;
            float cluster_point_wall_distance_thresh = 0.5f;
        } room_seg;

    private:
        SystemParams();
        static SystemParams *mSystemParams;
        YAML::Node mConfig;
    };
}

#endif