#pragma once

#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <vector>

namespace height_scan
{

inline constexpr const char* kHeightScanTopic = "rt/height_scan";
// Must match deploy/robots/go2/include/HeightScanUpdater.h's
// kHeightScanGridNx/Ny/SizeX/SizeY/Resolution (Isaac Lab
// GridPatternCfg(resolution=0.05, size=[1.4, 1.0]) -> 29x21 raw cells).
inline constexpr int kGridNx = 29;
inline constexpr int kGridNy = 21;
inline constexpr int kGridSize = kGridNx * kGridNy;
inline constexpr float kSizeX = 1.4f;
inline constexpr float kSizeY = 1.0f;
inline constexpr float kResolution = 0.05f;
// Matches deploy / velocity_env_cfg_go2.GO2_HEIGHT_SCAN_OFFSET
// (= 0.32 + (-0.046825)).
inline constexpr float kOffset = 0.273175f;
inline constexpr float kEmpty = -1.0f;
inline constexpr float kClipMin = -1.0f;
inline constexpr float kClipMax = 5.0f;

bool parse_pointcloud2(
    const sensor_msgs::msg::dds_::PointCloud2_& msg,
    std::vector<float>& out);

}  // namespace height_scan
