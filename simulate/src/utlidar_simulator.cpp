#include "utlidar_simulator.h"

#include <cmath>
#include <cstring>
#include <iostream>

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr uint8_t kFloat32 = sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_;

void mat_vec3(const mjtNum* mat, const float in[3], mjtNum out[3])
{
    for (int i = 0; i < 3; ++i)
    {
        out[i] = mat[i * 3 + 0] * in[0] + mat[i * 3 + 1] * in[1] + mat[i * 3 + 2] * in[2];
    }
}

}  // namespace

UtLidarSimulator::UtLidarSimulator(mjModel* model, mjData* data)
    : mj_model_(model), mj_data_(data)
{
    site_id_ = mj_name2id(mj_model_, mjOBJ_SITE, "utlidar");
    if (site_id_ < 0)
    {
        std::cerr << "UtLidarSimulator: site 'utlidar' not found; LiDAR cloud disabled\n";
        return;
    }

    body_exclude_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "base_link");
    if (body_exclude_id_ < 0)
    {
        std::cerr << "UtLidarSimulator: body 'base_link' not found; LiDAR cloud disabled\n";
        return;
    }

    init_ray_pattern();
    init_point_cloud_template();
    enabled_ = true;
    std::cout << "UtLidarSimulator: publishing rt/utlidar/cloud ("
              << kHorizRays << " x " << kVertRays << " rays, " << (1.0 / kPublishPeriod)
              << " Hz)\n";
}

void UtLidarSimulator::init_ray_pattern()
{
    ray_dirs_.reserve(static_cast<size_t>(kHorizRays * kVertRays));

    const float elev_min = -0.5f * kVertFovDeg * kPi / 180.0f;
    const float elev_max = 0.5f * kVertFovDeg * kPi / 180.0f;
    const float elev_step =
        (kVertRays > 1) ? (elev_max - elev_min) / static_cast<float>(kVertRays - 1) : 0.0f;
    const float azimuth_step = 2.0f * kPi / static_cast<float>(kHorizRays);

    for (int iv = 0; iv < kVertRays; ++iv)
    {
        const float elev = elev_min + static_cast<float>(iv) * elev_step;
        const float cos_e = std::cos(elev);
        const float sin_e = std::sin(elev);

        for (int ih = 0; ih < kHorizRays; ++ih)
        {
            const float az = static_cast<float>(ih) * azimuth_step;
            const float cos_a = std::cos(az);
            const float sin_a = std::sin(az);

            RayDir dir{};
            dir.x = cos_e * cos_a;
            dir.y = cos_e * sin_a;
            dir.z = sin_e;
            ray_dirs_.push_back(dir);
        }
    }
}

void UtLidarSimulator::init_point_cloud_template()
{
    cloud_pub_.lock();

    auto& msg = cloud_pub_.msg_;
    msg.header().frame_id() = "utlidar_lidar";
    msg.height() = 1;
    msg.is_bigendian() = false;
    msg.is_dense() = true;
    msg.point_step() = 12;

    std::vector<sensor_msgs::msg::dds_::PointField_> fields(3);
    fields[0] = sensor_msgs::msg::dds_::PointField_("x", 0, kFloat32, 1);
    fields[1] = sensor_msgs::msg::dds_::PointField_("y", 4, kFloat32, 1);
    fields[2] = sensor_msgs::msg::dds_::PointField_("z", 8, kFloat32, 1);
    msg.fields() = std::move(fields);

    cloud_pub_.unlock();
}

void UtLidarSimulator::update(double sim_time)
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

    const mjtNum* site_pos = mj_data_->site_xpos + 3 * site_id_;
    const mjtNum* site_rot = mj_data_->site_xmat + 9 * site_id_;

    std::vector<LidarHit> hits;
    hits.reserve(ray_dirs_.size());

    mjtNum origin[3] = {site_pos[0], site_pos[1], site_pos[2]};
    mjtNum dir_world[3];
    int geomid[1] = {-1};

    for (const auto& dir_local : ray_dirs_)
    {
        const float local[3] = {dir_local.x, dir_local.y, dir_local.z};
        mat_vec3(site_rot, local, dir_world);

        const mjtNum dist = mj_ray(
            mj_model_,
            mj_data_,
            origin,
            dir_world,
            nullptr,
            1,
            body_exclude_id_,
            geomid);

        if (dist < 0.0 || dist > static_cast<mjtNum>(kMaxRange))
        {
            continue;
        }

        hits.push_back(
            {dir_local.x * static_cast<float>(dist),
             dir_local.y * static_cast<float>(dist),
             dir_local.z * static_cast<float>(dist)});
    }

    publish_cloud(hits);
}

void UtLidarSimulator::publish_cloud(const std::vector<LidarHit>& hits)
{
    if (!cloud_pub_.trylock())
    {
        return;
    }

    auto& msg = cloud_pub_.msg_;
    const uint32_t width = static_cast<uint32_t>(hits.size());
    msg.width() = width;
    msg.row_step() = msg.point_step() * width;
    msg.data().resize(msg.row_step());

    auto& stamp = msg.header().stamp();
    const int32_t sec = static_cast<int32_t>(last_publish_time_);
    stamp.sec() = sec;
    stamp.nanosec() = static_cast<uint32_t>((last_publish_time_ - sec) * 1e9);

    uint8_t* buffer = msg.data().data();
    for (uint32_t i = 0; i < width; ++i)
    {
        const uint32_t offset = i * msg.point_step();
        std::memcpy(buffer + offset + 0, &hits[i].x, sizeof(float));
        std::memcpy(buffer + offset + 4, &hits[i].y, sizeof(float));
        std::memcpy(buffer + offset + 8, &hits[i].z, sizeof(float));
    }

    cloud_pub_.unlockAndPublish();
}
