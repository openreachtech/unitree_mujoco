#pragma once

#include <mujoco/mujoco.h>

#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// MID-360 stand-in that publishes what livox_ros_driver2 does (xfer_format 0, 10 Hz), so a
// ROS 2 consumer (DLIO, elevation_mapping_cupy) cannot tell it from the real driver.
// Ported from tsubame_perception/livox_sim.py.
//
//   rt/livox/lidar -> ROS 2 /livox/lidar  PointCloud2, frame livox_frame, one per 100 ms.
//       x y z intensity (f32), tag line (u8), timestamp (f64, absolute ns), packed in
//       26 bytes. No-returns stay in as (0, 0, 0). header.stamp is the first point's time.
//   rt/livox/imu   -> ROS 2 /livox/imu    Imu, frame livox_frame, 200 Hz, acceleration in g.
//
// Rays follow the recorded MID-360 scan pattern (mid360.npy, (azimuth, elevation) pairs) at
// 200k points/s, a slice per physics step from the sensor's pose at that step, so the blind
// cone and motion distortion come out as on hardware. They also hit the robot itself.
//
// Stamps are wall-clock time anchored at the first step and advanced by sim time, so the
// lidar and IMU share one clock even when the sim runs slower than real time.
//
// Needs MJCF sites "livox_lidar" (livox_frame) and "livox_imu", with sensors
// "livox_imu_acc" / "livox_imu_gyro" on the latter. Call step() after every mj_step.
class LivoxSim
{
public:
    LivoxSim(const mjModel* model, const std::string& pattern_path);
    ~LivoxSim();

    bool ok() const { return ok_; }
    void step(const mjModel* m, mjData* d);

private:
    void cast(const mjModel* m, mjData* d, int64_t t_ns);
    void publish_imu(const mjData* d, int64_t t_ns);
    void publish_loop();

    bool ok_ = false;
    int site_id_ = -1, acc_adr_ = -1, gyro_adr_ = -1;
    std::vector<double> pattern_;  // unit directions in livox_frame, xyz per ray
    size_t cursor_ = 0;
    int rays_per_step_ = 0;
    double dt_ = 0.0;

    int64_t anchor_ns_ = 0;  // wall time of sim time 0
    double last_time_ = -1.0, next_imu_time_ = 0.0;

    std::vector<uint8_t> frame_;  // points of the frame being accumulated
    int64_t frame_start_ns_ = 0;
    double frame_start_time_ = 0.0;
    std::vector<mjtNum> dirs_w_, dist_;
    std::vector<int> geomid_;

    unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::Imu_> imu_pub_;
    unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::PointCloud2_> cloud_pub_;

    // Clouds are ~500 kB; DDS writes them off the physics thread.
    std::thread publisher_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<sensor_msgs::msg::dds_::PointCloud2_> queue_;
    bool running_ = true;
};
