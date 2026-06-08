#pragma once

#include <mujoco/mujoco.h>

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <cstdint>
#include <vector>

// Simulates Go2 onboard LiDAR: raycast in MuJoCo, publish rt/utlidar/cloud (PointCloud2).
class UtLidarSimulator
{
public:
    UtLidarSimulator(mjModel* model, mjData* data);

    void update(double sim_time);

    bool enabled() const { return enabled_; }

private:
    struct RayDir
    {
        float x;
        float y;
        float z;
    };

    struct LidarHit
    {
        float x;
        float y;
        float z;
    };

    void init_ray_pattern();
    void init_point_cloud_template();
    void publish_cloud(const std::vector<LidarHit>& hits);

    mjModel* mj_model_;
    mjData* mj_data_;
    int site_id_ = -1;
    int body_exclude_id_ = -1;
    bool enabled_ = false;

    std::vector<RayDir> ray_dirs_;

    unitree::robot::RealTimePublisher<sensor_msgs::msg::dds_::PointCloud2_> cloud_pub_{
        "rt/utlidar/cloud"};

    double last_publish_time_ = -1.0;
    static constexpr double kPublishPeriod = 0.1;  // ~10 Hz, matches Go2 LiDAR
    static constexpr float kMaxRange = 30.0f;
    static constexpr int kHorizRays = 360;
    static constexpr int kVertRays = 16;
    static constexpr float kVertFovDeg = 30.0f;  // +/-15 deg elevation
};
