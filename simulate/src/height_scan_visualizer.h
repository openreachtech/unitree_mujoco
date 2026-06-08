#pragma once

#include <mujoco/mujoco.h>

#include <vector>

class HeightScanVisualizer;

// Set from Go2Bridge; used by main.cc key callback (H toggle).
extern HeightScanVisualizer* g_height_scan_viz;

class HeightScanVisualizer
{
public:
    HeightScanVisualizer(mjModel* model, mjData* data);

    void update(const std::vector<float>& height_scan, const mjtNum* site_pos, float imu_yaw);

    bool enabled() const { return enabled_; }
    void set_enabled(bool enabled);

private:
    void hide_all();
    mjModel* mj_model_ = nullptr;
    mjData* mj_data_ = nullptr;
    int site_id_ = -1;
    bool enabled_ = false;
    std::vector<int> mocap_body_ids_;
};
