# Common Configuration Parameters of vS-Graphs

This guide describes the configuration parameters (independent of the `ROS` wrapper) for performing SLAM. They can be find and modified in [common_system_params.yaml](/config/common_system_params.yaml).

## General Parameters

These parameters are combined in the `general` category and cover general system configurations that are not specific to any module:

| Category  | Sub-category        | Description                                                               |
| --------- | ------------------- | ------------------------------------------------------------------------- |
| `general` | `mode_of_operation` | choose from 0 (SemSeg+GeoSeg), 1 (SemSeg), and 2 (GeoSeg)                 |
| `general` | `env_database`      | the JSON file containing the details on markers placed in the environment |

## Marker Parameters

These parameters are combined in the `markers` category and cover configurations related to aruco markers:

| Category  | Sub-category | Description                                             |
| --------- | ------------ | ------------------------------------------------------- |
| `markers` | `impact`     | defines how much the system should trust markers' poses |

## Optimization Parameters

These parameters are combined in the `optimization` category and cover configurations of the optimizer thread:

| Category       | Sub-category         | Child              | Description                                     |
| -------------- | -------------------- | ------------------ | ----------------------------------------------- |
| `optimization` | `marginalize_planes` |                    | marginalize out during the optimization?        |
| `optimization` | `plane_kf`           | `enabled`          | enables plane-to-KeyFrame factor in the graph   |
| `optimization` | `plane_kf`           | `information_gain` | sets the information gain for plane-to-KeyFrame |
| `optimization` | `plane_point`        | `enabled`          | enables plane-to-points factor in the graph     |
| `optimization` | `plane_point`        | `information_gain` | sets the information gain for plane-to-points   |

## Segmentation Parameters

These parameters are combined in the `seg` category and cover configurations for common segmentation parameters:

| Category | Sub-category               | Child             | Description                                                |
| -------- | -------------------------- | ----------------- | ---------------------------------------------------------- |
| `seg`    | `pointclouds_thresh`       |                   | minimum number of points needed to fit a plane             |
| `seg`    | `plane_association_thresh` |                   | minimum ominus threshold for considering two planes as one |
| `seg`    | `plane_point_dist_thresh`  |                   | maximum distance for point to be considered on a plane     |
| `seg`    | `plane_cutting_threshold`  |                   | maximum spatial distance between two planes to stay uncut  |
| `seg`    | `ransac`                   | `max_planes`      | maximum number of planes extracted from a pointcloud       |
| `seg`    | `ransac`                   | `distance_thresh` | maximum distance for a point to be considered as inlier    |
| `seg`    | `ransac`                   | `max_iterations`  | maximum number of RANSAC iterations                        |

## Geometric Segmentation Parameters

These parameters are combined in the `geo_seg` category and cover configurations for Geometric Segmentation (GeoSeg) parameters:

| Category  | Sub-category | Process           | Child                  | Description                                   |
| --------- | ------------ | ----------------- | ---------------------- | --------------------------------------------- |
| `geo_seg` | `pointcloud` | `downsample`      | `leaf_size`            | leaf size (same in all axes) for downsampling |
| `geo_seg` | `pointcloud` | `downsample`      | `min_points_per_voxel` | minimum number of points per voxel            |
| `geo_seg` | `pointcloud` | `outlier_removal` | `std_threshold`        | standard deviation threshold                  |
| `geo_seg` | `pointcloud` | `outlier_removal` | `mean_threshold`       | number of points considered as neighbors      |

## Semantic Segmentation Parameters

These parameters are combined in the `sem_seg` category and cover configurations for Semantic Segmentation (SemSeg) parameters:

| Category  | Sub-category         | Process             | Child                  | Description                                |
| --------- | -------------------- | ------------------- | ---------------------- | ------------------------------------------ |
| `sem_seg` | `pointcloud`         | `downsample`        | `leaf_size`            | leaf size (in all axes) for downsampling   |
| `sem_seg` | `pointcloud`         | `downsample`        | `min_points_per_voxel` | minimum number of points per voxel         |
| `sem_seg` | `pointcloud`         | `outlier_removal`   | `std_threshold`        | standard deviation threshold               |
| `sem_seg` | `pointcloud`         | `outlier_removal`   | `mean_threshold`       | number of points considered as neighbors   |
| `sem_seg` | `prob_thresh`        |                     |                        | minimum class probability for points (>.5) |
| `sem_seg` | `conf_thresh`        |                     |                        | confidence threshold for class probability |
| `sem_seg` | `max_step_elevation` |                     |                        | max. height over the main ground plane     |
| `sem_seg` | `max_tilt_wall`      |                     |                        | maximum tilt heuristic for a wall          |
| `sem_seg` | `max_tilt_ground`    |                     |                        | maximum tilt heuristic for a ground        |
| `sem_seg` | `min_votes`          |                     |                        | minimum votes for a plane to get a label   |
| `sem_seg` | `reassociate`        | `enabled`           |                        | enables re-association of semantic planes  |
| `sem_seg` | `reassociate`        | `association_thres` |                        | threshold for re-association               |

## Room Segmentation Parameters

These parameters are combined in the `room_seg` category and cover configurations for room detection and segmentation:

| Category   | Sub-category               | Child                                | Description                                                                                                                       |
| ---------- | -------------------------- | ------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------- |
| `room_seg` | `method`                   |                                      | choose from 0 (Geometric), 1 (FreeSpace), and 2 (GNN)                                                                             |
| `room_seg` | `plane_facing_dot_thresh`  |                                      | max. dot-product of plane normals to be facing                                                                                    |
| `room_seg` | `min_wall_distance_thresh` |                                      | minimum space between two given walls of a corridor/room (in meters) to be valid                                                  |
| `room_seg` | `perpendicularity_thresh`  |                                      | thresh. for walls perpendicularity to be square room (degrees)                                                                    |
| `room_seg` | `center_distance_thresh`   |                                      | maximum distance between room centroids to associate (meters)                                                                     |
| `room_seg` | `geo_based`                | `marker_wall_distance_thresh`        | maximum distance from a marker to a wall to be considered part of the room in geometrically-based closest walls room segmentation |
| `room_seg` | `skeleton_based`           | `min_cluster_vertices`               | minimum number of points needed to form a cluster in voxblox free-space room segmentation                                         |
| `room_seg` | `skeleton_based`           | `cluster_point_wall_distance_thresh` | maximum distance from a point of a cluster to a wall to be considered part of the room in voxblox free-space room segmentation    |
