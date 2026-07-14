#pragma once

#include <mujoco/mujoco.h>

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <vector>

// Direct terrain height map (Isaac-style), not a LiDAR point-cloud sim.
//
// Yaw-aligned 17×11 downward mj_ray onto static ground → full height_scan[187]
// (no under-body masking; deploy applies that for the policy).
// Publishes rt/height_scan for deploy / debug.
class HeightMapSimulator
{
public:
    HeightMapSimulator(mjModel* model, mjData* data);

    void update(double sim_time);

    bool enabled() const { return enabled_; }
    const std::vector<float>& height_scan() const { return height_scan_; }
    float imu_yaw() const { return imu_yaw_; }
    const mjtNum* site_pos() const;

private:
    void init_publisher();
    void compute_height_map();
    void publish_height_scan();

    mjModel* mj_model_ = nullptr;
    mjData* mj_data_ = nullptr;
    int site_id_ = -1;
    int imu_quat_adr_ = -1;
    bool enabled_ = false;

    std::vector<float> height_scan_;
    float imu_yaw_ = 0.0f;

    unitree::robot::RealTimePublisher<sensor_msgs::msg::dds_::PointCloud2_> height_scan_pub_{
        "rt/height_scan"};

    double last_publish_time_ = -1.0;
    static constexpr double kPublishPeriod = 0.02;  // 50 Hz
};
