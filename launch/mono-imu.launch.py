import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    keyframe_depth_config = os.path.join(
        get_package_share_directory("keyframe_depth_estimator"),
        "config",
        "config.yaml",
    )
    keyframe_depth_validator_config = os.path.join(
        get_package_share_directory("keyframe_depth_validator"),
        "config",
        "config.yaml",
    )
    dynamic_keypoint_tracker_config = os.path.join(
        get_package_share_directory("dynamic_keypoint_tracker"),
        "config",
        "config.yaml",
    )
    dynamic_keypoint_3d_lifter_config = os.path.join(
        get_package_share_directory("dynamic_keypoint_3d_lifter"),
        "config",
        "config.yaml",
    )
    object_motion_estimator_config = os.path.join(
        get_package_share_directory("object_motion_estimator"),
        "config",
        "config.yaml",
    )

    return LaunchDescription(
        [
            # Global arguments declarations
            DeclareLaunchArgument("offline", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=[
                    get_package_share_directory("vs_graphs"),
                    "/config/Visualization/vsgraphs_rgbd.rviz",
                ],
            ),
            DeclareLaunchArgument("colored_pointcloud", default_value="true"),
            DeclareLaunchArgument("visualize_segmented_scene", default_value="true"),
            DeclareLaunchArgument("use_aux_depth", default_value="false"),
            DeclareLaunchArgument("launch_keyframe_depth_estimator", default_value="false"), # Pipeline DA3
            DeclareLaunchArgument("launch_keyframe_depth_validator", default_value="false"), # Pipeline DA3
            DeclareLaunchArgument("launch_dynamic_keypoint_tracker", default_value="true"),
            DeclareLaunchArgument("launch_object_track_manager", default_value="true"),
            DeclareLaunchArgument("launch_track_outlier_rejector", default_value="true"),
            DeclareLaunchArgument("launch_dynamic_keypoint_3d_lifter", default_value="false"),    # Pipeline DA3 
            DeclareLaunchArgument("launch_dynamic_keypoint_interpolator", default_value="false"), # Pipeline DA3
            DeclareLaunchArgument("launch_object_motion_estimator", default_value="false"),  # Pipeline DA3
            DeclareLaunchArgument("launch_object_ba_tracker", default_value="true"), # Pipeline BA
            DeclareLaunchArgument(
                "keyframe_depth_model_path",
                default_value=(
                    "/home/marco/workspace/src/keyframe_depth_estimator/include/models/"
                    "engines/da3metric_280x504.engine"
                ),
            ),
            DeclareLaunchArgument(
                "keyframe_depth_metric_topic", default_value="/keyframe_depth/metric"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_corrected_topic", default_value="/keyframe_depth/corrected"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_publish_debug_image", default_value="false"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_publish_debug_image", default_value="false"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_queue_depth", default_value="100"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_debug_match_logging", default_value="false"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_use_offline_metric_depth", default_value="true"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_offline_sync_tolerance_ms", default_value="20.0"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_tuning_mode", default_value="false"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_tuning_history_size", default_value="200"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_tuning_visualization_period_s", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_rgbd_depth_topic",
                default_value="/camera/realsense/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_dynamic_masks_topic",
                default_value="/camera/color/instance_masks",
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_rgbd_valid_range_min_m", default_value="0.3"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_rgbd_valid_range_max_m", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "keyframe_depth_validator_rgbd_heatmap_max_abs_error_m", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_instance_masks_topic",
                default_value="/camera/color/image_instance_masks_array",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_tracks_topic",
                default_value="/dynamic_keypoint_tracks",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_object_tracks_topic",
                default_value="/object_tracks",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_filtered_object_tracks_topic",
                default_value="/object_tracks/filtered",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_publish_debug_image",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_debug_image_width",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_debug_image_height",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_3d_lifter_output_topic",
                default_value="/dynamic_object_points_3d",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_3d_lifter_low_confidence_behavior",
                default_value="lift_and_flag",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_3d_lifter_low_confidence_max_residual_m",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_3d_lifter_max_lift_distance_m",
                default_value="7.0",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_3d_lifter_publish_debug_image",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_3d_lifter_queue_depth",
                default_value="100",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_3d_lifter_debug_match_logging",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_interpolator_output_topic",
                default_value="/dynamic_object_points_3d/interpolated",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_interpolator_queue_depth",
                default_value="300",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_interpolator_gap_timeout_ms",
                default_value="2000",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_interpolator_min_residual_path_length_px",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "dynamic_keypoint_interpolator_publish_debug_image",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "object_motion_input_mode",
                default_value="keyframe_and_interpolated_weighted",
                description=(
                    "Phase 4 input mode: keyframe_only, keyframe_and_interpolated, "
                    "or keyframe_and_interpolated_weighted"
                ),
            ),
            DeclareLaunchArgument(
                "object_motion_output_topic",
                default_value="/object_motion/depth_model",
            ),
            DeclareLaunchArgument(
                "object_motion_markers_topic",
                default_value="/object_motion/depth_model/markers",
            ),
            DeclareLaunchArgument(
                "object_motion_world_frame_id",
                default_value="world",
            ),
            DeclareLaunchArgument(
                "object_motion_queue_depth",
                default_value="300",
            ),
            DeclareLaunchArgument(
                "object_motion_gap_timeout_ms",
                default_value="2000",
            ),
            DeclareLaunchArgument(
                "object_motion_trajectory_window_size",
                default_value="2147483647",
            ),
            DeclareLaunchArgument(
                "object_motion_mad_k",
                default_value="2.5",
            ),
            DeclareLaunchArgument(
                "object_motion_ema_alpha_h",
                default_value="0.4",
            ),
            DeclareLaunchArgument(
                "object_motion_publish_debug_image",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "object_motion_publish_markers",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "object_motion_arrow_scale",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "object_motion_arrow_time_scale_s",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "object_motion_arrow_lifetime_s",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "object_motion_clear_markers_on_track_lost",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "object_motion_marker_line_width_m",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "object_motion_marker_sphere_radius_m",
                default_value="0.08",
            ),
            DeclareLaunchArgument(
                "object_motion_marker_text_height_m",
                default_value="0.15",
            ),
            DeclareLaunchArgument(
                "object_ba_output_topic",
                default_value="/object_motion/ba",
            ),
            DeclareLaunchArgument(
                "object_ba_markers_topic",
                default_value="/object_motion/ba/markers",
            ),
            DeclareLaunchArgument(
                "object_ba_window_size",
                default_value="20",
            ),
            DeclareLaunchArgument(
                "object_ba_min_window_frames",
                default_value="5",
            ),
            DeclareLaunchArgument(
                "object_ba_min_point_observations",
                default_value="6",
            ),
            DeclareLaunchArgument(
                "object_ba_sigma_pixel",
                default_value="1.5",
            ),
            DeclareLaunchArgument(
                "object_ba_huber_delta_pixels",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "object_ba_num_iterations",
                default_value="10",
            ),
            DeclareLaunchArgument(
                "object_ba_time_budget_ms",
                default_value="15.0",
            ),
            DeclareLaunchArgument(
                "object_ba_min_camera_displacement_m",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "object_ba_alpha_scale",
                default_value="0.3",
            ),
            DeclareLaunchArgument(
                "object_ba_pose_topic_timeout_s",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "object_ba_pose_sync_tolerance_ms",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "object_ba_mad_k",
                default_value="2.5",
            ),
            DeclareLaunchArgument(
                "object_ba_max_landmark_anchor_norm_m",
                default_value="20.0",
            ),
            DeclareLaunchArgument(
                "object_ba_max_landmark_reprojection_error_px",
                default_value="8.0",
            ),
            DeclareLaunchArgument(
                "object_ba_min_scale_object_displacement_m",
                default_value="0.10",
            ),
            DeclareLaunchArgument("keyframe_depth_sky_handling", default_value="true"),
            DeclareLaunchArgument(
                "aux_depth_topic", default_value="/camera/depth_da3/image_rect"
            ),
            DeclareLaunchArgument(
                "semantic_scene_segmenter",
                default_value="off",
                description="The method to segment the semantic scene (if off, the baseline)",
                choices=["yoso", "pfcn", "yolo26", "off"],
            ),
            # Topics
            DeclareLaunchArgument("camera_frame", default_value="camera"),
            DeclareLaunchArgument("sensor_config", default_value="UniLu_RealSense_D435i_640"),
            DeclareLaunchArgument(
                "rgb_image_topic", default_value="/camera/realsense/color/image_raw"
            ),
            DeclareLaunchArgument(
                "rgb_camera_info_topic",
                default_value="/camera/realsense/color/camera_info",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/camera/realsense/imu",
            ),
            # VS-Graphs Node
            Node(
                name="vs_graphs",
                package="vs_graphs",
                executable="ros_mono_inertial",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("offline")},
                    {
                        "voc_file": LaunchConfiguration(
                            "voc_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/Vocabulary/ORBvoc.txt.bin",
                            ],
                        )
                    },
                    {
                        "settings_file": LaunchConfiguration(
                            "settings_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/Monocular-Inertial/",
                                LaunchConfiguration("sensor_config"),
                                ".yaml",
                            ],
                        )
                    },
                    {
                        "sys_params_file": LaunchConfiguration(
                            "sys_params_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/system_params.yaml",
                            ],
                        )
                    },
                    {"yaw": 0.0},
                    {"roll": 0.0},
                    {"pitch": 0.0},
                    {"frame_map": "map"},
                    {"frame_imu": "imu"},
                    {"frame_world": "world"},
                    {"frame_camera": "camera_color_optical_frame"},
                    {"enable_pangolin": False},
                    {"static_transform": True},
                    {"colored_pointcloud": False},
                    {"publish_pointclouds": True},
                    {"use_aux_depth": LaunchConfiguration("use_aux_depth")},
                    {"aux_depth_topic": LaunchConfiguration("aux_depth_topic")},
                ],
                remappings=[
                    ("/imu", LaunchConfiguration("imu_topic")),
                    ("/camera/image_raw", LaunchConfiguration("rgb_image_topic")),
                ],
            ),
            # Keyframe Metric Depth Estimator
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_keyframe_depth_estimator")
                ),
                package="keyframe_depth_estimator",
                executable="keyframe_depth_estimator_node",
                name="keyframe_depth_estimator",
                output="screen",
                parameters=[
                    keyframe_depth_config,
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "keyframe_created_topic": "/orbslam3/keyframe_created",
                        "camera_info_topic": LaunchConfiguration(
                            "rgb_camera_info_topic"
                        ),
                        "metric_depth_topic": LaunchConfiguration(
                            "keyframe_depth_metric_topic"
                        ),
                        "model_path": LaunchConfiguration(
                            "keyframe_depth_model_path"
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration("keyframe_depth_publish_debug_image"),
                            value_type=bool,
                        ),
                        "sky_handling": ParameterValue(
                            LaunchConfiguration("keyframe_depth_sky_handling"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            # Keyframe Depth Validator
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_keyframe_depth_validator")
                ),
                package="keyframe_depth_validator",
                executable="keyframe_depth_validator_node",
                name="keyframe_depth_validator",
                output="screen",
                parameters=[
                    keyframe_depth_validator_config,
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "metric_depth_topic": LaunchConfiguration(
                            "keyframe_depth_metric_topic"
                        ),
                        "static_correspondences_topic": (
                            "/orbslam3/keyframe_static_map_points"
                        ),
                        "corrected_depth_topic": LaunchConfiguration(
                            "keyframe_depth_corrected_topic"
                        ),
                        "queue_depth": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_queue_depth"
                            ),
                            value_type=int,
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_publish_debug_image"
                            ),
                            value_type=bool,
                        ),
                        "debug_match_logging": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_debug_match_logging"
                            ),
                            value_type=bool,
                        ),
                        "use_offline_metric_depth": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_use_offline_metric_depth"
                            ),
                            value_type=bool,
                        ),
                        "offline_metric_depth_sync_tolerance_ms": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_offline_sync_tolerance_ms"
                            ),
                            value_type=float,
                        ),
                        "tuning_mode": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_tuning_mode"
                            ),
                            value_type=bool,
                        ),
                        "tuning_history_size": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_tuning_history_size"
                            ),
                            value_type=int,
                        ),
                        "tuning_visualization_period_s": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_tuning_visualization_period_s"
                            ),
                            value_type=float,
                        ),
                        "rgbd_depth_topic": LaunchConfiguration(
                            "keyframe_depth_validator_rgbd_depth_topic"
                        ),
                        "dynamic_masks_topic": LaunchConfiguration(
                            "keyframe_depth_validator_dynamic_masks_topic"
                        ),
                        "rgbd_valid_range_min_m": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_rgbd_valid_range_min_m"
                            ),
                            value_type=float,
                        ),
                        "rgbd_valid_range_max_m": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_rgbd_valid_range_max_m"
                            ),
                            value_type=float,
                        ),
                        "rgbd_heatmap_max_abs_error_m": ParameterValue(
                            LaunchConfiguration(
                                "keyframe_depth_validator_rgbd_heatmap_max_abs_error_m"
                            ),
                            value_type=float,
                        ),
                    },
                ],
            ),
            # Dynamic Keypoint Tracker
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_dynamic_keypoint_tracker")
                ),
                package="dynamic_keypoint_tracker",
                executable="dynamic_keypoint_tracker_node",
                name="dynamic_keypoint_tracker",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "image_topic": LaunchConfiguration("rgb_image_topic"),
                        "instance_masks_topic": LaunchConfiguration(
                            "dynamic_keypoint_instance_masks_topic"
                        ),
                        "tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_tracks_topic"
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_publish_debug_image"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            # Dynamic Object Track Manager
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_object_track_manager")
                ),
                package="dynamic_keypoint_tracker",
                executable="object_track_manager_node",
                name="object_track_manager",
                output="screen",
                parameters=[
                    dynamic_keypoint_tracker_config,
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "instance_masks_topic": LaunchConfiguration(
                            "dynamic_keypoint_instance_masks_topic"
                        ),
                        "dynamic_keypoint_tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_tracks_topic"
                        ),
                        "image_topic": LaunchConfiguration("rgb_image_topic"),
                        "object_tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_object_tracks_topic"
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_publish_debug_image"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            # Dynamic Track Outlier Rejector
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_track_outlier_rejector")
                ),
                package="dynamic_keypoint_tracker",
                executable="track_outlier_rejector_node",
                name="track_outlier_rejector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "object_tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_object_tracks_topic"
                        ),
                        "filtered_object_tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_filtered_object_tracks_topic"
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_publish_debug_image"),
                            value_type=bool,
                        ),
                        "debug_image_width": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_debug_image_width"),
                            value_type=int,
                        ),
                        "debug_image_height": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_debug_image_height"),
                            value_type=int,
                        ),
                    },
                ],
            ),
            # Dynamic Keypoint 3D Lifter
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_dynamic_keypoint_3d_lifter")
                ),
                package="dynamic_keypoint_3d_lifter",
                executable="dynamic_keypoint_3d_lifter_node",
                name="dynamic_keypoint_3d_lifter",
                output="screen",
                parameters=[
                    dynamic_keypoint_3d_lifter_config,
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "object_tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_filtered_object_tracks_topic"
                        ),
                        "corrected_depth_topic": LaunchConfiguration(
                            "keyframe_depth_corrected_topic"
                        ),
                        "keyframe_pose_topic": "/vs_graphs/camera_pose",
                        "camera_info_topic": LaunchConfiguration("rgb_camera_info_topic"),
                        "output_topic": LaunchConfiguration(
                            "dynamic_keypoint_3d_lifter_output_topic"
                        ),
                        "queue_depth": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_3d_lifter_queue_depth"),
                            value_type=int,
                        ),
                        "low_confidence_behavior": LaunchConfiguration(
                            "dynamic_keypoint_3d_lifter_low_confidence_behavior"
                        ),
                        "low_confidence_max_residual_m": ParameterValue(
                            LaunchConfiguration(
                                "dynamic_keypoint_3d_lifter_low_confidence_max_residual_m"
                            ),
                            value_type=float,
                        ),
                        "max_lift_distance_m": ParameterValue(
                            LaunchConfiguration(
                                "dynamic_keypoint_3d_lifter_max_lift_distance_m"
                            ),
                            value_type=float,
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration(
                                "dynamic_keypoint_3d_lifter_publish_debug_image"
                            ),
                            value_type=bool,
                        ),
                        "debug_match_logging": ParameterValue(
                            LaunchConfiguration(
                                "dynamic_keypoint_3d_lifter_debug_match_logging"
                            ),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            # Dynamic Keypoint Interpolator
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_dynamic_keypoint_interpolator")
                ),
                package="dynamic_keypoint_3d_lifter",
                executable="dynamic_keypoint_interpolator_node",
                name="dynamic_keypoint_interpolator",
                output="screen",
                parameters=[
                    dynamic_keypoint_3d_lifter_config,
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "lifted_points_topic": LaunchConfiguration(
                            "dynamic_keypoint_3d_lifter_output_topic"
                        ),
                        "object_tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_filtered_object_tracks_topic"
                        ),
                        "camera_pose_topic": "/vs_graphs/camera_pose",
                        "camera_info_topic": LaunchConfiguration("rgb_camera_info_topic"),
                        "output_topic": LaunchConfiguration(
                            "dynamic_keypoint_interpolator_output_topic"
                        ),
                        "queue_depth": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_interpolator_queue_depth"),
                            value_type=int,
                        ),
                        "gap_timeout_ms": ParameterValue(
                            LaunchConfiguration("dynamic_keypoint_interpolator_gap_timeout_ms"),
                            value_type=int,
                        ),
                        "min_residual_path_length_px": ParameterValue(
                            LaunchConfiguration(
                                "dynamic_keypoint_interpolator_min_residual_path_length_px"
                            ),
                            value_type=float,
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration(
                                "dynamic_keypoint_interpolator_publish_debug_image"
                            ),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            # Object Motion Estimator
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_object_motion_estimator")
                ),
                package="object_motion_estimator",
                executable="object_motion_estimator_node",
                name="object_motion_estimator",
                output="screen",
                parameters=[
                    object_motion_estimator_config,
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "input_mode": LaunchConfiguration("object_motion_input_mode"),
                        "dynamic_object_points_topic": LaunchConfiguration(
                            "dynamic_keypoint_3d_lifter_output_topic"
                        ),
                        "interpolated_points_topic": LaunchConfiguration(
                            "dynamic_keypoint_interpolator_output_topic"
                        ),
                        "object_track_events_topic": "/object_tracks/events",
                        "camera_pose_topic": "/vs_graphs/camera_pose",
                        "camera_info_topic": LaunchConfiguration("rgb_camera_info_topic"),
                        "image_topic": LaunchConfiguration("rgb_image_topic"),
                        "output_topic": LaunchConfiguration("object_motion_output_topic"),
                        "markers_topic": LaunchConfiguration("object_motion_markers_topic"),
                        "world_frame_id": LaunchConfiguration("object_motion_world_frame_id"),
                        "queue_depth": ParameterValue(
                            LaunchConfiguration("object_motion_queue_depth"),
                            value_type=int,
                        ),
                        "gap_timeout_ms": ParameterValue(
                            LaunchConfiguration("object_motion_gap_timeout_ms"),
                            value_type=int,
                        ),
                        "trajectory_window_size": ParameterValue(
                            LaunchConfiguration("object_motion_trajectory_window_size"),
                            value_type=int,
                        ),
                        "mad_k": ParameterValue(
                            LaunchConfiguration("object_motion_mad_k"),
                            value_type=float,
                        ),
                        "ema_alpha_h": ParameterValue(
                            LaunchConfiguration("object_motion_ema_alpha_h"),
                            value_type=float,
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration("object_motion_publish_debug_image"),
                            value_type=bool,
                        ),
                        "publish_markers": ParameterValue(
                            LaunchConfiguration("object_motion_publish_markers"),
                            value_type=bool,
                        ),
                        "arrow_scale": ParameterValue(
                            LaunchConfiguration("object_motion_arrow_scale"),
                            value_type=float,
                        ),
                        "arrow_time_scale_s": ParameterValue(
                            LaunchConfiguration("object_motion_arrow_time_scale_s"),
                            value_type=float,
                        ),
                        "arrow_lifetime_s": ParameterValue(
                            LaunchConfiguration("object_motion_arrow_lifetime_s"),
                            value_type=float,
                        ),
                        "clear_markers_on_track_lost": ParameterValue(
                            LaunchConfiguration("object_motion_clear_markers_on_track_lost"),
                            value_type=bool,
                        ),
                        "marker_line_width_m": ParameterValue(
                            LaunchConfiguration("object_motion_marker_line_width_m"),
                            value_type=float,
                        ),
                        "marker_sphere_radius_m": ParameterValue(
                            LaunchConfiguration("object_motion_marker_sphere_radius_m"),
                            value_type=float,
                        ),
                        "marker_text_height_m": ParameterValue(
                            LaunchConfiguration("object_motion_marker_text_height_m"),
                            value_type=float,
                        ),
                    },
                ],
            ),
            # Object BA Tracker (Pipeline B)
            Node(
                condition=IfCondition(
                    LaunchConfiguration("launch_object_ba_tracker")
                ),
                package="object_motion_estimator",
                executable="object_ba_tracker_node",
                name="object_ba_tracker",
                output="screen",
                parameters=[
                    object_motion_estimator_config,
                    {
                        "use_sim_time": LaunchConfiguration("offline"),
                        "object_tracks_topic": LaunchConfiguration(
                            "dynamic_keypoint_filtered_object_tracks_topic"
                        ),
                        "camera_pose_topic": "/vs_graphs/camera_pose",
                        "camera_info_topic": LaunchConfiguration("rgb_camera_info_topic"),
                        "object_track_events_topic": "/object_tracks/events",
                        "output_topic": LaunchConfiguration("object_ba_output_topic"),
                        "markers_topic": LaunchConfiguration("object_ba_markers_topic"),
                        "world_frame_id": LaunchConfiguration("object_motion_world_frame_id"),
                        "queue_depth": ParameterValue(
                            LaunchConfiguration("object_motion_queue_depth"),
                            value_type=int,
                        ),
                        "window_size": ParameterValue(
                            LaunchConfiguration("object_ba_window_size"),
                            value_type=int,
                        ),
                        "min_window_frames": ParameterValue(
                            LaunchConfiguration("object_ba_min_window_frames"),
                            value_type=int,
                        ),
                        "min_point_observations": ParameterValue(
                            LaunchConfiguration("object_ba_min_point_observations"),
                            value_type=int,
                        ),
                        "sigma_pixel": ParameterValue(
                            LaunchConfiguration("object_ba_sigma_pixel"),
                            value_type=float,
                        ),
                        "huber_delta_pixels": ParameterValue(
                            LaunchConfiguration("object_ba_huber_delta_pixels"),
                            value_type=float,
                        ),
                        "num_iterations": ParameterValue(
                            LaunchConfiguration("object_ba_num_iterations"),
                            value_type=int,
                        ),
                        "ba_time_budget_ms": ParameterValue(
                            LaunchConfiguration("object_ba_time_budget_ms"),
                            value_type=float,
                        ),
                        "min_camera_displacement_m": ParameterValue(
                            LaunchConfiguration("object_ba_min_camera_displacement_m"),
                            value_type=float,
                        ),
                        "alpha_scale": ParameterValue(
                            LaunchConfiguration("object_ba_alpha_scale"),
                            value_type=float,
                        ),
                        "pose_topic_timeout_s": ParameterValue(
                            LaunchConfiguration("object_ba_pose_topic_timeout_s"),
                            value_type=float,
                        ),
                        "pose_sync_tolerance_ms": ParameterValue(
                            LaunchConfiguration("object_ba_pose_sync_tolerance_ms"),
                            value_type=float,
                        ),
                        "mad_k": ParameterValue(
                            LaunchConfiguration("object_ba_mad_k"),
                            value_type=float,
                        ),
                        "max_landmark_anchor_norm_m": ParameterValue(
                            LaunchConfiguration("object_ba_max_landmark_anchor_norm_m"),
                            value_type=float,
                        ),
                        "max_landmark_reprojection_error_px": ParameterValue(
                            LaunchConfiguration("object_ba_max_landmark_reprojection_error_px"),
                            value_type=float,
                        ),
                        "min_scale_object_displacement_m": ParameterValue(
                            LaunchConfiguration("object_ba_min_scale_object_displacement_m"),
                            value_type=float,
                        ),
                        "trajectory_window_size": ParameterValue(
                            LaunchConfiguration("object_motion_trajectory_window_size"),
                            value_type=int,
                        ),
                        "publish_markers": ParameterValue(
                            LaunchConfiguration("object_motion_publish_markers"),
                            value_type=bool,
                        ),
                        "arrow_time_scale_s": ParameterValue(
                            LaunchConfiguration("object_motion_arrow_time_scale_s"),
                            value_type=float,
                        ),
                        "arrow_lifetime_s": ParameterValue(
                            LaunchConfiguration("object_motion_arrow_lifetime_s"),
                            value_type=float,
                        ),
                        "clear_markers_on_track_lost": ParameterValue(
                            LaunchConfiguration("object_motion_clear_markers_on_track_lost"),
                            value_type=bool,
                        ),
                        "marker_line_width_m": ParameterValue(
                            LaunchConfiguration("object_motion_marker_line_width_m"),
                            value_type=float,
                        ),
                        "marker_sphere_radius_m": ParameterValue(
                            LaunchConfiguration("object_motion_marker_sphere_radius_m"),
                            value_type=float,
                        ),
                        "marker_text_height_m": ParameterValue(
                            LaunchConfiguration("object_motion_marker_text_height_m"),
                            value_type=float,
                        ),
                    },
                ],
            ),
            # Static Transforms
            Node(
                package="tf2_ros",
                name="map_to_map_elevated",  # For Voxblox Skeleton
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "map_elevated"],
            ),
            Node(
                name="bc_to_se",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "3", "0", "0", "0", "build_comp", "struc_elem"],
            ),
            Node(
                package="tf2_ros",
                name="world_to_bc",
                executable="static_transform_publisher",
                arguments=["0", "0", "3", "0", "0", "0", "world", "build_comp"],
            ),
            Node(
                package="tf2_ros",
                name="camera_to_imu",
                executable="static_transform_publisher",
                arguments=[
                    "-0.011739999987185001",
                    "-0.005520000122487545",
                    "0.005100000184029341",
                    "0",
                    "0",
                    "0",
                    "camera",
                    "imu",
                ],
            ),
            Node(
                package="tf2_ros",
                name="camera_to_camera_optical",
                executable="static_transform_publisher",
                arguments=[
                    "0.00011983246804447845",
                    "0.014999180100858212",
                    "0.00015637179603800178",
                    "-1.5730284322450263",
                    "0.019080586334886032",
                    "-1.574238659946616",
                    "camera",
                    "camera_color_optical_frame",
                    # RealSense: camera_color_optical_frame, OpenLoris: d400_color
                ],
            ),
            # RViz
            Node(
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_config"),
                ],
                output="screen",
            ),
            # Semantic Scene Segmenter Node (based on semantic_scene_segmenter argument)
            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("semantic_scene_segmenter"), "yoso"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_yoso.py",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")}
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yoso.yaml",
                    ],
                ],
            ),
            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("semantic_scene_segmenter"), "pfcn"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_pFCN.py",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")}
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_pFCN.yaml",
                    ],
                ],
            ),
            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("semantic_scene_segmenter"), "yolo26"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="frame_segmenter_yolo26.py",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")},
                    {
                        "params.ros_topics.raw_image_topic": LaunchConfiguration(
                            "rgb_image_topic"
                        )
                    },
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yolo26.yaml",
                    ],
                ],
            ),
            # Structural Element Detectors
            # Node(
            #     name="situational_graphs_reasoning",
            #     package="situational_graphs_reasoning",
            #     executable="situational_graphs_reasoning",
            #     output="screen",
            #     # parameters=[
            #     #     os.path.join(
            #     #         get_package_share_directory("situational_graphs_reasoning"),
            #     #         "config",
            #     #         "params.yaml",
            #     #     )
            #     # ],
            #     # remappings=[
            #     #     (
            #     #         "situational_graphs_reasoning/graphs",
            #     #         "/s_graphs/graph_structure",
            #     #     ),
            #     # ],
            # ),
        ]
    )
