# 🚀 Configure vS-Graphs

This guide outlines the core configuration parameters for running SLAM using **vS-Graphs**, independent of the `ROS`-related settings. These parameters are stored in [system_params.yaml](/config/system_params.yaml).

## ⚙️ General Parameters

These parameters are grouped under the `general` category and define system-wide configurations that are not tied to any specific module:

| Category  | Parameter           | Description                                                                                                      |
| --------- | ------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `general` | `mode_of_operation` | Operating mode: <br> `0` = SemSeg + GeoSeg <br> `1` = SemSeg only (default - recommended) <br> `2` = GeoSeg only |
| `general` | `env_database`      | Path to the `JSON` file containing information about fiducial markers in the environment                         |

## 🏷️ Marker Parameters

These parameters are grouped under the `markers` category and control configurations related to ArUco markers:

| Category  | Parameter | Description                                                 |
| --------- | --------- | ----------------------------------------------------------- |
| `markers` | `impact`  | Defines the weight or trust level assigned to marker poses. |

## 🧮 Optimization Parameters

These parameters are grouped under the `optimization` category and define configurations for the optimizer thread:

| Category       | Parameter            | Sub-parameter      | Description                                                        |
| -------------- | -------------------- | ------------------ | ------------------------------------------------------------------ |
| `optimization` | `marginalize_planes` | —                  | Whether to marginalize planes during optimization (`true`/`false`) |
| `optimization` | `plane_kf`           | `enabled`          | Enables plane-to-KeyFrame factors in the optimization graph        |
| `optimization` | `plane_kf`           | `information_gain` | Sets the information gain for plane-to-KeyFrame associations       |
| `optimization` | `plane_point`        | `enabled`          | Enables plane-to-points factors in the optimization graph          |
| `optimization` | `plane_point`        | `information_gain` | Sets the information gain for plane-to-points associations         |

## 🗺️ Map Point Refinement Parameters

These parameters are grouped under the `refine_map_points` category and configure the semantic-based refinement of map points:

| Category            | Parameter                 | Sub-parameter   | Description                                                                              |
| ------------------- | ------------------------- | --------------- | ---------------------------------------------------------------------------------------- |
| `refine_map_points` | `enabled`                 | —               | Enables or disables semantic refinement of map points (`true` / `false`)                 |
| `refine_map_points` | `max_distance_for_delete` | —               | Maximum allowed distance (in meters) from semantic constraints before a point is deleted |
| `refine_map_points` | `octree`                  | `resolution`    | Resolution of the octree used for spatial partitioning                                   |
| `refine_map_points` | `octree`                  | `search_radius` | Search radius used when finding neighboring points                                       |
| `refine_map_points` | `octree`                  | `min_neighbors` | Minimum number of neighbors required to keep a point                                     |

## 🪞 Plane-Based Covisibility Graph Parameters

These parameters are grouped under the `plane_based_covisibility` category and configure the construction of a covisibility graph using semantic planes:

| Category                   | Parameter         | Description                                                                 |
| -------------------------- | ----------------- | --------------------------------------------------------------------------- |
| `plane_based_covisibility` | `enabled`         | Enables or disables the use of plane-based covisibility (`true` / `false`)  |
| `plane_based_covisibility` | `max_keyframes`   | Maximum number of keyframes considered when building the covisibility graph |
| `plane_based_covisibility` | `score_per_plane` | The score each semantic plane contributes to the covisibility graph         |

## 🧩 Segmentation Parameters (Common)

These parameters are grouped under the `seg` category and define common settings for the segmentation process:

| Category | Parameter                  | Sub-parameter     | Description                                                              |
| -------- | -------------------------- | ----------------- | ------------------------------------------------------------------------ |
| `seg`    | `pointclouds_thresh`       | —                 | Minimum number of points required to fit a plane                         |
| `seg`    | `plane_association_thresh` | —                 | Minimum ominus threshold to consider two planes as the same              |
| `seg`    | `plane_point_dist_thresh`  | —                 | Maximum distance a point can be from a plane to be considered part of it |
| `seg`    | `plane_cutting_threshold`  | —                 | Maximum spatial separation between two planes to avoid being segmented   |
| `seg`    | `ransac`                   | `max_planes`      | Maximum number of planes to extract from a single point cloud            |
| `seg`    | `ransac`                   | `distance_thresh` | Maximum distance from a point to a plane to be considered an inlier      |
| `seg`    | `ransac`                   | `max_iterations`  | Maximum number of RANSAC iterations during plane fitting                 |

## 📐 Geometric Segmentation Parameters

These parameters fall under the `geo_seg` category and configure the behavior of the Geometric Segmentation (GeoSeg) module:

| Category  | Component    | Process           | Parameter              | Description                                                          |
| --------- | ------------ | ----------------- | ---------------------- | -------------------------------------------------------------------- |
| `geo_seg` | `pointcloud` | `downsample`      | `leaf_size`            | Leaf size (uniform across all axes) used for voxel grid downsampling |
| `geo_seg` | `pointcloud` | `downsample`      | `min_points_per_voxel` | Minimum number of points required per voxel to retain it             |
| `geo_seg` | `pointcloud` | `outlier_removal` | `std_threshold`        | Standard deviation threshold for statistical outlier removal         |
| `geo_seg` | `pointcloud` | `outlier_removal` | `mean_threshold`       | Minimum number of neighboring points for a point to be retained      |

## 🧠 Semantic Segmentation Parameters

These parameters belong to the `sem_seg` category and configure the Semantic Segmentation (SemSeg) process:

| Category  | Component     | Process           | Parameter                          | Description                                                        |
| --------- | ------------- | ----------------- | ---------------------------------- | ------------------------------------------------------------------ |
| `sem_seg` | `pointcloud`  | `downsample`      | `leaf_size`                        | Leaf size (uniform across all axes) for voxel downsampling         |
| `sem_seg` | `pointcloud`  | `downsample`      | `min_points_per_voxel`             | Minimum number of points per voxel to retain it                    |
| `sem_seg` | `pointcloud`  | `outlier_removal` | `std_threshold`                    | Standard deviation threshold for outlier removal                   |
| `sem_seg` | `pointcloud`  | `outlier_removal` | `mean_threshold`                   | Number of neighboring points required to keep a point              |
| `sem_seg` | —             | —                 | `prob_thresh`                      | Minimum class probability threshold (e.g., > 0.5)                  |
| `sem_seg` | —             | —                 | `conf_thresh`                      | Minimum confidence threshold for class probabilities               |
| `sem_seg` | —             | —                 | `max_step_elevation`               | Maximum step height over the ground plane                          |
| `sem_seg` | —             | —                 | `max_tilt_wall`                    | Maximum tilt angle for wall classification                         |
| `sem_seg` | —             | —                 | `max_tilt_ground`                  | Maximum tilt angle for ground classification                       |
| `sem_seg` | —             | —                 | `max_wall_door_distance`           | Maximum distance from wall to door for association                 |
| `sem_seg` | —             | —                 | `max_door_width`                   | Maximum width for a door to be valid                               |
| `sem_seg` | —             | —                 | `max_door_height`                  | Maximum height for a door to be valid                              |
| `sem_seg` | —             | —                 | `max_kf_passage_distance`          | Maximum distance from keyframe to doorway                          |
| `sem_seg` | —             | —                 | `passage_centroid_distance_thresh` | Maximum distance from passage association                          |
| `sem_seg` | —             | —                 | `passage_kf_window`                | Size of the KeyFrame window to check for passing through a doorway |
| `sem_seg` | —             | —                 | `min_votes`                        | Minimum number of votes needed for a plane to get a label          |
| `sem_seg` | `reassociate` | —                 | `enabled`                          | Enables semantic re-association of planes (`true/false`)           |
| `sem_seg` | `reassociate` | —                 | `association_thres`                | Threshold for considering planes in reassociation                  |

## 🚪 Room Segmentation Parameters

These parameters belong to the `room_seg` category and configure the detection and segmentation of rooms:

| Category   | Sub-category     | Parameter                                        | Description                                                                                                                 |
| ---------- | ---------------- | ------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------- |
| `room_seg` | —                | `method`                                         | Choose method: 0 (Geometric), 1 (FreeSpace), or 2 (GNN)                                                                     |
| `room_seg` | —                | `plane_facing_dot_thresh`                        | Maximum dot product of plane normals to be considered facing                                                                |
| `room_seg` | —                | `min_wall_distance_thresh`                       | Minimum valid distance (meters) between two walls of a corridor or room                                                     |
| `room_seg` | —                | `perpendicularity_thresh`                        | Threshold in degrees for walls perpendicularity                                                                             |
| `room_seg` | —                | `parallelism_thresh`                             | Threshold in degrees for walls parallelism                                                                                  |
| `room_seg` | —                | `center_distance_thresh`                         | Maximum distance (meters) between room centroids to be associated                                                           |
| `room_seg` | `geo_based`      | `marker_wall_distance_thresh`                    | Max distance from marker to wall to consider marker part of the room (geometric method)                                     |
| `room_seg` | `skeleton_based` | `min_cluster_vertices`                           | Minimum number of points to form a cluster (voxblox free-space room segmentation)                                           |
| `room_seg` | `skeleton_based` | `cluster_point_wall_distance_thresh`             | Max distance from a cluster points to a wall to be considered part of the room (voxblox free-space segmentation)            |
| `room_seg` | `skeleton_based` | `cluster_centroid_wall_centroid_distance_thresh` | Max distance from a cluster centroid to a wall centroid to be considered part of the room (voxblox free-space segmentation) |
| `room_seg` | `gnn_based`      | `gnn_version`                                    | The version of GNN-based room detector (1: the legacy used in S-Graphs, 2: the newer version)                               |
