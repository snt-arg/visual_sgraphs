#include <yaml-cpp/yaml.h>
#include <iostream>

#ifndef SYSTEMPARAMS_H
#define SYSTEMPARAMS_H

namespace VS_GRAPHS
{
    class SystemParams
    {
    public:
        static SystemParams *GetParams();
        void SetParams(const std::string &strConfigFile);

        // Common struct definitions
        struct Constraint
        {
            bool enabled = false;
            float information_gain = 0.1f;
        };
        struct Downsample
        {
            float leaf_size = 0.03f;
            unsigned int min_points_per_voxel = 5;
        };
        struct OutlierRemoval
        {
            float std_threshold = 1.0;
            unsigned int mean_threshold = 50;
        };

        // Structs for different modules
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

        struct optimization
        {
            bool marginalize_planes = false;
            Constraint plane_map_point;
            Constraint plane_kf;
            Constraint plane_point;
        } optimization;

        struct refine_map_points
        {
            bool enabled = false;
            float max_distance_for_delete = 0.5f;
            struct octree
            {
                float resolution = 0.1f;
                float search_radius = 0.5f;
                unsigned int min_neighbors = 2;
            } octree;
        } refine_map_points;

        struct plane_based_covisibility
        {
            bool enabled = true;
            unsigned int max_keyframes = 75;
            unsigned int score_per_plane = 60;
        } plane_based_covisibility;

        struct seg
        {
            unsigned int pointclouds_thresh = 200;
            float plane_point_dist_thresh = 0.2f;

            struct plane_association
            {
                float ominus_thresh = 0.1f;
                float distance_thresh = 0.2f;
                float centroid_thresh = 3.2f;

                struct cluster_separation
                {
                    bool enabled = false;
                    float tolerance = 2.5f;
                    Downsample downsample;
                } cluster_separation;

            } plane_association;

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
                Downsample downsample;
                OutlierRemoval outlier_removal;
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
                Downsample downsample;
                OutlierRemoval outlier_removal;
            } pointcloud;

            struct reassociate
            {
                bool enabled = false;
                float association_thresh = 0.2f;
            } reassociate;
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

            float center_distance_thresh = 1.5f;
            float plane_facing_dot_thresh = -0.8f;
            float min_wall_distance_thresh = 1.0f;
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