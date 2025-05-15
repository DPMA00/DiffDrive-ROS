include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,

    map_frame = "map",
    tracking_frame = "base_footprint",
    published_frame = "base_footprint",
    odom_frame = "odom",

    

    provide_odom_frame = false,
    publish_frame_projected_to_2d = true,
    use_pose_extrapolator = true,        -- Added: explicitly use pose extrapolator (true by default in ROS2 Cartographer)
    use_odometry = true,
    use_nav_sat = false,
    use_landmarks = false,

    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    num_point_clouds = 0,

    lookup_transform_timeout_sec = 0.2,   -- Added: TF lookup timeout
    submap_publish_period_sec = 0.3,      -- Added: submap publishing period
    pose_publish_period_sec = 0.005,      -- Added: pose publishing period (5e-3)
    trajectory_publish_period_sec = 0.030,-- Added: trajectory publishing period (30e-3)

    rangefinder_sampling_ratio = 0.5,
    odometry_sampling_ratio = 0.5,
    fixed_frame_pose_sampling_ratio = 1.0,
    imu_sampling_ratio = 0.5,
    landmarks_sampling_ratio = 1.0,      -- Added: landmark sampling ratio (even if landmarks not used)
}

-- Use the 2D SLAM algorithm
MAP_BUILDER.use_trajectory_builder_2d = true

-- Trajectory builder 2D parameters
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.03
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

return options
