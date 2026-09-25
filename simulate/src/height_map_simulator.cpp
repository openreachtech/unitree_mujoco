#include "height_map_simulator.h"

#include "height_scan_dds.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

namespace
{

constexpr uint8_t kFloat32 = sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_;

constexpr float kMaxRayDist = 30.0f;

// L1 mount in the base frame: velocity_env_cfg_go2's GO2_LIDAR_OFFSET_X/Y/Z.
constexpr float kMountX = 0.28945f;
constexpr float kMountY = 0.0f;

// Only the terrain. mj_ray's geomgroup filter is the one that does this -- flg_static
// merely decides whether world-welded geoms count, and a null geomgroup means *every*
// group, so the rays were also hitting the robot (visual meshes are group 2, collision
// geoms group 3) and the 609 marker spheres the visualiser moves around (group 2). The
// first put a cluster of readings on the robot's own back and face; the second is a
// feedback loop, the map measuring its own debug overlay. Terrain geoms carry no explicit
// group, so they are group 0.
constexpr mjtByte kTerrainGeomGroup[mjNGROUP] = {1, 0, 0, 0, 0, 0};

float clip_height(float value)
{
    return std::clamp(value, height_scan::kClipMin, height_scan::kClipMax);
}

}  // namespace

HeightMapSimulator::HeightMapSimulator(mjModel* model, mjData* data)
    : mj_model_(model), mj_data_(data)
{
    site_id_ = mj_name2id(mj_model_, mjOBJ_SITE, "utlidar");
    if (site_id_ < 0)
    {
        std::cerr << "HeightMapSimulator: site 'utlidar' not found; height map disabled\n";
        return;
    }

    base_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "base_link");
    if (base_body_id_ < 0)
    {
        std::cerr << "HeightMapSimulator: body 'base_link' not found; height map disabled\n";
        return;
    }

    const int sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
    if (sensor_id >= 0)
    {
        imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
    }

    height_scan_.assign(height_scan::kGridSize, height_scan::kFlatFill);
    measured_.assign(height_scan::kGridSize, 0);
    init_publisher();
    enabled_ = true;

    const int rays = (height_scan::kGridNx * kRayOversample)
                   * (height_scan::kGridNy * kRayOversample) * (kProbeLevels + 1);
    std::cout << "HeightMapSimulator: " << rays << " rays from the L1 mount ("
              << (kProbeLevels + 1) << " elevation levels) -> "
              << height_scan::kGridNx << "x" << height_scan::kGridNy
              << " cells (occluded cells hold), noise " << (kRangeNoiseStd * 1000.0f)
              << " mm -> rt/height_scan (" << (1.0 / kPublishPeriod) << " Hz)\n";
}

const mjtNum* HeightMapSimulator::grid_ref_pos() const
{
    if (!enabled_ || !mj_data_ || site_id_ < 0 || base_body_id_ < 0)
    {
        return nullptr;
    }
    // base_link's XY and z -- the same reference compute_height_map() measures against,
    // so debug markers land on the hits the policy actually receives.
    const mjtNum* base = mj_data_->xpos + 3 * base_body_id_;
    grid_center_[0] = base[0];
    grid_center_[1] = base[1];
    grid_center_[2] = base[2];
    return grid_center_;
}

void HeightMapSimulator::init_publisher()
{
    height_scan_pub_.lock();
    auto& msg = height_scan_pub_.msg_;
    msg.header().frame_id() = "base";
    msg.height() = static_cast<uint32_t>(height_scan::kGridNx);
    msg.width() = static_cast<uint32_t>(height_scan::kGridNy);
    msg.is_bigendian() = false;
    msg.is_dense() = true;
    msg.point_step() = sizeof(float);
    msg.row_step() = msg.point_step() * msg.width();
    std::vector<sensor_msgs::msg::dds_::PointField_> fields(1);
    fields[0] = sensor_msgs::msg::dds_::PointField_("z", 0, kFloat32, 1);
    msg.fields() = std::move(fields);
    msg.data().resize(msg.row_step() * msg.height());
    height_scan_pub_.unlock();
}

void HeightMapSimulator::update(double sim_time)
{
    if (!enabled_ || !mj_data_)
    {
        return;
    }

    if (last_publish_time_ >= 0.0 && (sim_time - last_publish_time_) < kPublishPeriod)
    {
        return;
    }
    last_publish_time_ = sim_time;

    const auto t0 = std::chrono::steady_clock::now();
    compute_height_map();
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    publish_height_scan();

    // The ray budget is the risk here: one publish period is 20 ms and the map casts
    // thousands of rays. Only say something when that is actually being exceeded --
    // a map quietly eating the simulation step is otherwise felt rather than seen.
    if (ms > kPublishPeriod * 1000.0 && ms > worst_ms_)
    {
        worst_ms_ = ms;
        std::cout << "HeightMapSimulator: " << ms << " ms to build the map, over the "
                  << (kPublishPeriod * 1000.0) << " ms publish period -- reduce "
                  << "kProbeLevels or kPublishPeriod\n";
    }
}

float HeightMapSimulator::sample_range_noise()
{
    return range_noise_(rng_);
}

bool HeightMapSimulator::advance_hold(const mjtNum* base, float yaw)
{
    // The grid is body-relative but the values in it describe the ground, so when the
    // robot moves the two come apart: a cell holding a reading from 5 cm ago describes
    // terrain that now sits one cell further back. Resample the held grid by the
    // displacement so old values stay where the ground is.
    //
    // Port of LidarElevationMap._advance_hold, which does this with grid_sample; at
    // 29 x 21 a hand-rolled bilinear read is the whole of it.
    const float x = static_cast<float>(base[0]);
    const float y = static_cast<float>(base[1]);
    const float z = static_cast<float>(base[2]);

    if (!pose_valid_)
    {
        prev_x_ = x; prev_y_ = y; prev_z_ = z; prev_yaw_ = yaw;
        pose_valid_ = true;
        return false;
    }

    const float dyaw = std::atan2(std::sin(yaw - prev_yaw_), std::cos(yaw - prev_yaw_));
    const float dz = z - prev_z_;
    // Displacement in the *new* body frame.
    const float dx_w = x - prev_x_;
    const float dy_w = y - prev_y_;
    const float c = std::cos(yaw), sn = std::sin(yaw);
    const float dx_b = c * dx_w + sn * dy_w;
    const float dy_b = -sn * dx_w + c * dy_w;

    if (std::abs(dx_b) < 1e-6f && std::abs(dy_b) < 1e-6f
        && std::abs(dyaw) < 1e-6f && std::abs(dz) < 1e-6f)
    {
        return true;
    }

    hold_scratch_ = height_scan_;
    measured_scratch_ = measured_;

    const float half_x = 0.5f * height_scan::kSizeX;
    const float half_y = 0.5f * height_scan::kSizeY;
    const float inv_res = 1.0f / height_scan::kResolution;
    const float cd = std::cos(dyaw), sd = std::sin(dyaw);

    for (int ix = 0; ix < height_scan::kGridNx; ++ix)
    {
        for (int iy = 0; iy < height_scan::kGridNy; ++iy)
        {
            // This cell's position in the new body frame, expressed in the old one.
            const float px = -half_x + static_cast<float>(ix) * height_scan::kResolution;
            const float py = -half_y + static_cast<float>(iy) * height_scan::kResolution;
            const float sx = cd * px - sd * py + dx_b;
            const float sy = sd * px + cd * py + dy_b;

            const float fx = (sx + half_x) * inv_res;
            const float fy = (sy + half_y) * inv_res;
            const int x0 = static_cast<int>(std::floor(fx));
            const int y0 = static_cast<int>(std::floor(fy));
            const int idx = ix * height_scan::kGridNy + iy;

            if (x0 < 0 || x0 + 1 >= height_scan::kGridNx
                || y0 < 0 || y0 + 1 >= height_scan::kGridNy)
            {
                // Shifted in from outside the old grid: nothing was ever seen there.
                height_scan_[idx] = height_scan::kFlatFill;
                measured_[idx] = 0;
                continue;
            }

            const float tx = fx - static_cast<float>(x0);
            const float ty = fy - static_cast<float>(y0);
            auto at = [&](int a, int b) { return a * height_scan::kGridNy + b; };
            const float w00 = (1 - tx) * (1 - ty), w01 = (1 - tx) * ty;
            const float w10 = tx * (1 - ty),       w11 = tx * ty;

            const float m = w00 * measured_scratch_[at(x0, y0)]
                          + w01 * measured_scratch_[at(x0, y0 + 1)]
                          + w10 * measured_scratch_[at(x0 + 1, y0)]
                          + w11 * measured_scratch_[at(x0 + 1, y0 + 1)];
            if (m < 0.999f)
            {
                // Any part of this sample came from a never-measured cell; the fill is a
                // convention, not a height, so do not interpolate or shift it.
                height_scan_[idx] = height_scan::kFlatFill;
                measured_[idx] = 0;
                continue;
            }

            const float v = w00 * hold_scratch_[at(x0, y0)]
                          + w01 * hold_scratch_[at(x0, y0 + 1)]
                          + w10 * hold_scratch_[at(x0 + 1, y0)]
                          + w11 * hold_scratch_[at(x0 + 1, y0 + 1)];
            // Values are relative to the robot's z, which has itself moved.
            height_scan_[idx] = v + dz;
            measured_[idx] = 1;
        }
    }

    prev_x_ = x; prev_y_ = y; prev_z_ = z; prev_yaw_ = yaw;
    return true;
}

void HeightMapSimulator::compute_height_map()
{
    // Heights are measured from base_link's z, not the utlidar site's. kOffset is
    // GO2_NOMINAL_BASE_Z + GO2_LIDAR_OFFSET_Z, so it only balances against the base --
    // pairing it with the site's z, which is another 46.825 mm lower, biases the whole map
    // down by that much. Isaac's LidarElevationMap uses root_pos[:, 2] (the base).
    const mjtNum* base = mj_data_->xpos + 3 * base_body_id_;
    const mjtNum* site = mj_data_->site_xpos + 3 * site_id_;

    // Isaac ray_alignment="base": the mount swings with the body.
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    if (imu_quat_adr_ >= 0)
    {
        w = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 0]);
        x = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 1]);
        y = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 2]);
        z = static_cast<float>(mj_data_->sensordata[imu_quat_adr_ + 3]);
    }
    imu_yaw_ = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    const float cy = std::cos(imu_yaw_);
    const float sy = std::sin(imu_yaw_);

    advance_hold(base, imu_yaw_);

    const float base_z = static_cast<float>(base[2]);
    const float half_x = 0.5f * height_scan::kSizeX;
    const float half_y = 0.5f * height_scan::kSizeY;
    const float inv_res = 1.0f / height_scan::kResolution;

    // Ray origin: the L1 mount, in world coordinates.
    const mjtNum origin[3] = {
        base[0] + cy * kMountX - sy * kMountY,
        base[1] + sy * kMountX + cy * kMountY,
        static_cast<mjtNum>(site[2]),  // the site *is* base_z + kMountZ
    };

    // Per cell, keep the *highest* return, which on this inverted convention
    // (base_z - hit_z - offset) is the smallest number -- LidarElevationMap's
    // scatter_reduce_(reduce="amin"). Cells nothing lands in are left holding.
    std::vector<float> best(height_scan::kGridSize, std::numeric_limits<float>::max());

    const int nx = height_scan::kGridNx * kRayOversample;
    const int ny = height_scan::kGridNy * kRayOversample;
    const float step = height_scan::kResolution / static_cast<float>(kRayOversample);
    const float ground_z = static_cast<float>(origin[2]) - height_scan::kOffset;

    // Level -1 is the ground; 0..kProbeLevels-1 are the raised probes.
    for (int level = -1; level < kProbeLevels; ++level)
    {
    const float aim_z = (level < 0)
        ? ground_z
        : ground_z + kProbeLevelMin + static_cast<float>(level) * kProbeLevelStep;

    for (int rx = 0; rx < nx; ++rx)
    {
        for (int ry = 0; ry < ny; ++ry)
        {
            // Aim at where this direction's ground would be if the terrain were flat.
            // Where it actually lands decides which cell it fills -- a ray aimed past a
            // wall stops on the wall and fills the wall's cell, leaving the ground behind
            // it untouched. That is the shadow, and it is the whole point of firing from
            // the mount instead of straight down.
            //
            // The aiming plane is the mount's own height minus kOffset, not the base's:
            // kOffset is the mount's nominal clearance, so site_z - kOffset is the ground,
            // while base_z - kOffset is 4.7 cm above it -- enough overshoot to throw all
            // but the nearest cells clean out of the grid.
            const float x_aim = -half_x + static_cast<float>(rx) * step;
            const float y_aim = -half_y + static_cast<float>(ry) * step;
            const mjtNum target[3] = {
                base[0] + cy * x_aim - sy * y_aim,
                base[1] + sy * x_aim + cy * y_aim,
                static_cast<mjtNum>(aim_z),
            };
            mjtNum dir[3] = {
                target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]};
            const mjtNum norm = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            if (norm < 1e-9)
            {
                continue;
            }
            dir[0] /= norm; dir[1] /= norm; dir[2] /= norm;

            int geomid[1] = {-1};
            // Group 0 only: floor / boxes / hfield. A null filter would also hit the robot
            // (visual meshes are group 2, collision geoms group 3) and the 609 marker
            // spheres the visualiser moves -- the first puts readings on the robot's own
            // back, the second is the map measuring its own debug overlay.
            mjtNum dist = mj_ray(
                mj_model_, mj_data_, origin, dir, kTerrainGeomGroup,
                /*flg_static=*/1, /*bodyexclude=*/-1, geomid);

            if (dist < 0.0 || dist > static_cast<mjtNum>(kMaxRayDist))
            {
                continue;
            }

            dist += static_cast<mjtNum>(sample_range_noise());
            if (dist < static_cast<mjtNum>(kMinRange))
            {
                continue;  // shorter than the sensor's minimum: a dropout, not a point
            }

            const float hit_x = static_cast<float>(origin[0] + dir[0] * dist);
            const float hit_y = static_cast<float>(origin[1] + dir[1] * dist);
            const float hit_z = static_cast<float>(origin[2] + dir[2] * dist);

            // Back into the yaw-aligned base frame, then to a cell.
            const float dx = hit_x - static_cast<float>(base[0]);
            const float dy = hit_y - static_cast<float>(base[1]);
            const float x_body = cy * dx + sy * dy;
            const float y_body = -sy * dx + cy * dy;

            const int ix = static_cast<int>(std::lround((x_body + half_x) * inv_res));
            const int iy = static_cast<int>(std::lround((y_body + half_y) * inv_res));
            if (ix < 0 || ix >= height_scan::kGridNx || iy < 0 || iy >= height_scan::kGridNy)
            {
                continue;
            }

            const float value = base_z - hit_z - height_scan::kOffset;
            const int idx = ix * height_scan::kGridNy + iy;
            best[idx] = std::min(best[idx], value);
        }
    }
    }

    for (int i = 0; i < height_scan::kGridSize; ++i)
    {
        if (best[i] != std::numeric_limits<float>::max())
        {
            height_scan_[i] = clip_height(best[i]);
            measured_[i] = 1;
        }
    }
}

void HeightMapSimulator::publish_height_scan()
{
    if (!height_scan_pub_.trylock())
    {
        return;
    }

    auto& msg = height_scan_pub_.msg_;
    auto& stamp = msg.header().stamp();
    const int32_t sec = static_cast<int32_t>(last_publish_time_);
    stamp.sec() = sec;
    stamp.nanosec() = static_cast<uint32_t>((last_publish_time_ - sec) * 1e9);

    msg.data().resize(msg.row_step() * msg.height());
    uint8_t* buffer = msg.data().data();
    for (int i = 0; i < height_scan::kGridSize; ++i)
    {
        std::memcpy(buffer + static_cast<size_t>(i) * msg.point_step(), &height_scan_[i], sizeof(float));
    }

    height_scan_pub_.unlockAndPublish();
}
