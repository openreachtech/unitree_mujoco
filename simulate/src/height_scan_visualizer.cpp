#include "height_scan_visualizer.h"

#include "height_scan_dds.h"

#include <cmath>
#include <cstdio>
#include <iostream>

namespace
{

constexpr float kHiddenZ = -10.0f;

}  // namespace

HeightScanVisualizer* g_height_scan_viz = nullptr;

HeightScanVisualizer::HeightScanVisualizer(mjModel* model, mjData* data)
    : mj_model_(model), mj_data_(data)
{
    site_id_ = mj_name2id(mj_model_, mjOBJ_SITE, "utlidar");
    if (site_id_ < 0)
    {
        std::cerr << "HeightScanVisualizer: site 'utlidar' not found\n";
        return;
    }

    mocap_body_ids_.reserve(height_scan::kGridSize);
    for (int idx = 0; idx < height_scan::kGridSize; ++idx)
    {
        char name[16];
        std::snprintf(name, sizeof(name), "hs_%03d", idx);
        const int body_id = mj_name2id(mj_model_, mjOBJ_BODY, name);
        if (body_id < 0)
        {
            std::cerr << "HeightScanVisualizer: mocap body '" << name << "' not found; "
                      << "include height_scan_mocap.xml in the scene\n";
            mocap_body_ids_.clear();
            return;
        }
        mocap_body_ids_.push_back(body_id);
    }

    enabled_ = true;
    g_height_scan_viz = this;
    std::cout << "HeightScanVisualizer: " << mocap_body_ids_.size()
              << " mocap cells (toggle with key 'h')\n";
}

void HeightScanVisualizer::set_enabled(bool enabled)
{
    enabled_ = enabled;
    if (!enabled_)
    {
        hide_all();
    }
}

void HeightScanVisualizer::hide_all()
{
    if (!mj_data_ || mocap_body_ids_.empty())
    {
        return;
    }

    for (const int body_id : mocap_body_ids_)
    {
        const int mocap_id = mj_model_->body_mocapid[body_id];
        if (mocap_id < 0)
        {
            continue;
        }
        mjtNum* pos = mj_data_->mocap_pos + 3 * mocap_id;
        pos[0] = 0.0;
        pos[1] = 0.0;
        pos[2] = kHiddenZ;
    }
}

void HeightScanVisualizer::update(
    const std::vector<float>& height_scan,
    const mjtNum* site_pos,
    float imu_yaw)
{
    if (!enabled_ || !mj_data_ || mocap_body_ids_.size() != static_cast<size_t>(height_scan::kGridSize))
    {
        return;
    }

    if (static_cast<int>(height_scan.size()) != height_scan::kGridSize)
    {
        return;
    }

    const float cy = std::cos(imu_yaw);
    const float sy = std::sin(imu_yaw);
    const float half_x = 0.5f * height_scan::kSizeX;
    const float half_y = 0.5f * height_scan::kSizeY;
    const float x_min = -half_x;
    const float y_min = -half_y;

    for (int ix = 0; ix < height_scan::kGridNx; ++ix)
    {
        for (int iy = 0; iy < height_scan::kGridNy; ++iy)
        {
            const int idx = ix * height_scan::kGridNy + iy;
            const int mocap_id = mj_model_->body_mocapid[mocap_body_ids_[idx]];
            if (mocap_id < 0)
            {
                continue;
            }

            mjtNum* pos = mj_data_->mocap_pos + 3 * mocap_id;
            const float value = height_scan[idx];

            if (value <= height_scan::kEmpty + 1e-3f)
            {
                pos[0] = 0.0;
                pos[1] = 0.0;
                pos[2] = kHiddenZ;
                continue;
            }

            const float x_cell = x_min + (static_cast<float>(ix) + 0.5f) * height_scan::kResolution;
            const float y_cell = y_min + (static_cast<float>(iy) + 0.5f) * height_scan::kResolution;
            const float z_cell = -(value + height_scan::kOffset);

            pos[0] = site_pos[0] + cy * x_cell - sy * y_cell;
            pos[1] = site_pos[1] + sy * x_cell + cy * y_cell;
            pos[2] = site_pos[2] + z_cell;
        }
    }
}
