#pragma once

#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <vector>

namespace height_scan
{

inline constexpr const char* kHeightScanTopic = "rt/height_scan";
inline constexpr int kGridNx = 17;
inline constexpr int kGridNy = 11;
inline constexpr int kGridSize = kGridNx * kGridNy;
inline constexpr float kSizeX = 1.6f;
inline constexpr float kSizeY = 1.0f;
inline constexpr float kResolution = 0.1f;
inline constexpr float kOffset = 0.5f;
inline constexpr float kEmpty = -1.0f;

bool parse_pointcloud2(
    const sensor_msgs::msg::dds_::PointCloud2_& msg,
    std::vector<float>& out);

}  // namespace height_scan
