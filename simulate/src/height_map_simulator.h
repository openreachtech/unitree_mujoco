#pragma once

#include <mujoco/mujoco.h>

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <random>
#include <vector>

// Terrain height map as the MID-360 builds it: rays from the L1 mount at the nose, so a
// wall shadows the ground behind it, and cells in that shadow keep what they last saw.
//
// A plain top-down raycast will not do, and this is measured rather than assumed. Feeding
// the Phase 5 policy a clean downward-ray map in Isaac -- every cell fresh, nothing
// shadowed -- dropped its 25 cm wall crossing rate from 100 % to 12.5 %, and adding
// gaussian noise to that map changed nothing (11.7 %). The policy is not sensitive to how
// noisy the map is; it is sensitive to the map having shadows and stale cells, because
// that is what it learned to walk on.
//
// Mirrors mdp/lidar_elevation_map.py's LidarElevationMap:
//   * rays from the mount, binned by where they land, highest return per cell
//   * cells with no return this step hold their last value, starting at the flat fill
//   * the held grid is resampled onto the current pose each step, so an old reading stays
//     attached to the ground rather than to the robot (dropping this is not an option --
//     measured, it takes the same 25 cm crossing from 100 % to 0 %)
//   * returns shorter than the sensor's minimum range are dropouts, not points
//
// Deliberately simpler than the Isaac model: no scan pattern (every visible cell is
// refreshed every step, where the real sensor manages about a quarter), and no tilt or
// odometry error, since MuJoCo's pose is exact.
//
// Yaw-aligned 29x21 grid centered on base_link's XY; heights measured from base_link's z
// to match LidarElevationMap's root_pos[:, 2]. Publishes rt/height_scan.
class HeightMapSimulator
{
public:
    HeightMapSimulator(mjModel* model, mjData* data);

    void update(double sim_time);

    bool enabled() const { return enabled_; }
    const std::vector<float>& height_scan() const { return height_scan_; }
    float imu_yaw() const { return imu_yaw_; }
    // The point every cell's height is measured from: base_link's XY *and* z. The
    // published value is base_z - hit_z - kOffset, so anything reconstructing a world
    // position from it has to undo that with the same z.
    const mjtNum* grid_ref_pos() const;

private:
    void init_publisher();
    void compute_height_map();
    void publish_height_scan();
    float sample_range_noise();
    // Resample the held grid onto the current pose. Returns false the first time, when
    // there is no previous pose to measure displacement against.
    bool advance_hold(const mjtNum* base, float yaw);

    mjModel* mj_model_ = nullptr;
    mjData* mj_data_ = nullptr;
    int site_id_ = -1;
    int base_body_id_ = -1;
    int imu_quat_adr_ = -1;
    bool enabled_ = false;
    // Grid-reference point backing grid_ref_pos(); copied out of mjData rather than
    // pointed at, so it stays valid for the caller.
    mutable mjtNum grid_center_[3] = {0.0, 0.0, 0.0};

    std::vector<float> height_scan_;
    // Which cells hold a real measurement rather than the flat fill. The two move
    // differently under compensation: a measurement is relative to the robot's current z
    // and needs the vertical delta added back, while the fill is a fixed convention that
    // means nothing once a delta has been applied to it.
    std::vector<uint8_t> measured_;
    std::vector<float> hold_scratch_;
    std::vector<uint8_t> measured_scratch_;
    bool pose_valid_ = false;
    float prev_x_ = 0.0f, prev_y_ = 0.0f, prev_z_ = 0.0f, prev_yaw_ = 0.0f;
    float imu_yaw_ = 0.0f;
    std::mt19937 rng_{0xC0FFEE};
    std::normal_distribution<float> range_noise_{0.0f, kRangeNoiseStd};

    unitree::robot::RealTimePublisher<sensor_msgs::msg::dds_::PointCloud2_> height_scan_pub_{
        "rt/height_scan"};

    double last_publish_time_ = -1.0;
    double worst_ms_ = 0.0;
    static constexpr double kPublishPeriod = 0.02;  // 50 Hz

    // Per-cell, per-step Gaussian noise on the measured height. The ray is vertical, so
    // range error and height error are the same quantity here.
    //
    // 1.5 cm, between Isaac's "nominal" (1 cm) and "strong" (2 cm) MID360_NOISE_CFG
    // conditions. This map has no shadowing, no scan pattern and no stale cells -- it is a
    // perfect top-down raycast -- so the noise is the only thing standing in for all of
    // that, and sitting above nominal keeps a MuJoCo run from being the easier test on the
    // one axis where it can be made harder. Much past this and the markers jitter far
    // enough to be hard to read on screen.
    // One ray per cell aimed at the ground plane already covers 609/609 on flat terrain,
    // so the budget goes into elevation instead of into oversampling the ground.
    static constexpr int kRayOversample = 1;

    // Aiming every ray at the ground plane probes only the ground. A floating tread
    // hovering at 25 cm is then invisible unless it happens to sit 9-26 cm from the nose,
    // because that is the only stretch where a ground-bound ray is passing through that
    // height -- measured, and it is why the floating walls could not be seen at all.
    //
    // So each cell is also probed at a stack of heights: a ray aimed at (cell, z) is at
    // exactly z when it reaches that cell, and a plate whose 4 cm thickness straddles z
    // stops it. The step has to stay under the plate thickness or a plate can fall
    // between two levels.
    static constexpr float kProbeLevelStep = 0.03f;   // < tread_thickness (0.04)
    static constexpr float kProbeLevelMin = 0.06f;
    static constexpr float kProbeLevelMax = 0.36f;    // the tallest wall this lineage trains
    static constexpr int kProbeLevels =
        1 + static_cast<int>((kProbeLevelMax - kProbeLevelMin) / kProbeLevelStep + 0.5f);

    static constexpr float kRangeNoiseStd = 0.015f;

    // MID360_MIN_RANGE. A return shorter than the sensor's own minimum is a dropout, not
    // a point -- without this, short outliers land on the mount itself and, because the
    // blind disc around it is never refreshed, stay there as a cone of raised cells.
    static constexpr float kMinRange = 0.2f;
};
