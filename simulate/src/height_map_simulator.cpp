#include "height_map_simulator.h"

#include "height_scan_dds.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace
{

constexpr uint8_t kFloat32 = sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_;

// Start rays slightly above the sensor; flg_static hits terrain only.
// Body footprint masking is done on the deploy side (not here).
constexpr float kRayStartUp = 1.0f;
constexpr float kMaxRayDist = 30.0f;

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

    height_scan_.assign(height_scan::kGridSize, height_scan::kEmpty);
    init_publisher();
    enabled_ = true;

    std::cout << "HeightMapSimulator: " << height_scan::kGridNx << "x" << height_scan::kGridNy
              << " ground rays -> rt/height_scan (" << (1.0 / kPublishPeriod) << " Hz)\n";
}

const mjtNum* HeightMapSimulator::site_pos() const
{
    if (!enabled_ || !mj_data_ || site_id_ < 0 || base_body_id_ < 0)
    {
        return nullptr;
    }
    // Same base-XY / site-Z composition as compute_height_map(), so debug markers land on
    // the same grid the policy actually receives.
    const mjtNum* base = mj_data_->xpos + 3 * base_body_id_;
    const mjtNum* site = mj_data_->site_xpos + 3 * site_id_;
    grid_center_[0] = base[0];
    grid_center_[1] = base[1];
    grid_center_[2] = site[2];
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

    compute_height_map();
    publish_height_scan();
}

void HeightMapSimulator::compute_height_map()
{
    // Height reference (z) comes from the utlidar site -- its -0.046825m z-offset from
    // base_link is exactly what GO2_HEIGHT_SCAN_OFFSET expects. But the grid's XY center
    // must be base_link's own XY: the utlidar site sits 0.28945m forward of base_link, and
    // training explicitly re-centers its RayCaster at (0,0) in the base frame (see
    // velocity_env_cfg_go2.GO2_HEIGHT_SCAN_CENTER_X/Y). Using the site's XY here would
    // shift every sampled cell ~29cm forward of what the policy was trained on, and would
    // also throw off deploy's under-body exclusion (computed assuming a base-centered grid).
    const mjtNum* site = mj_data_->site_xpos + 3 * site_id_;
    const mjtNum* base = mj_data_->xpos + 3 * base_body_id_;

    // Isaac ray_alignment="yaw": grid follows heading, rays stay world-vertical.
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

    const float sensor_z = static_cast<float>(site[2]);
    const float half_x = 0.5f * height_scan::kSizeX;
    const float half_y = 0.5f * height_scan::kSizeY;

    // GridPatternCfg: arange(-size/2, size/2 + eps, resolution) → 29×21.
    for (int ix = 0; ix < height_scan::kGridNx; ++ix)
    {
        for (int iy = 0; iy < height_scan::kGridNy; ++iy)
        {
            const int idx = ix * height_scan::kGridNy + iy;
            const float x_cell = -half_x + static_cast<float>(ix) * height_scan::kResolution;
            const float y_cell = -half_y + static_cast<float>(iy) * height_scan::kResolution;

            const mjtNum origin[3] = {
                base[0] + cy * x_cell - sy * y_cell,
                base[1] + sy * x_cell + cy * y_cell,
                site[2] + kRayStartUp,
            };
            const mjtNum dir[3] = {0.0, 0.0, -1.0};
            int geomid[1] = {-1};

            // Static geoms only (floor / boxes / hfield) — never the robot.
            const mjtNum dist = mj_ray(
                mj_model_,
                mj_data_,
                origin,
                dir,
                nullptr,
                /*flg_static=*/1,
                /*bodyexclude=*/-1,
                geomid);

            if (dist < 0.0 || dist > static_cast<mjtNum>(kMaxRayDist))
            {
                height_scan_[idx] = height_scan::kEmpty;
                continue;
            }

            const float hit_z = static_cast<float>(origin[2] - dist);
            // Isaac mdp.height_scan: sensor_z - hit_z - offset (full terrain, no body mask).
            height_scan_[idx] = clip_height(sensor_z - hit_z - height_scan::kOffset);
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
