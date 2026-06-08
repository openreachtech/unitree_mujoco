#include "height_scan_dds.h"

#include <cstring>

namespace height_scan
{

bool parse_pointcloud2(
    const sensor_msgs::msg::dds_::PointCloud2_& msg,
    std::vector<float>& out)
{
    if (msg.height() != static_cast<uint32_t>(kGridNx) || msg.width() != static_cast<uint32_t>(kGridNy))
    {
        return false;
    }

    const uint32_t expected_points = static_cast<uint32_t>(kGridSize);
    const uint32_t point_step = msg.point_step();
    if (point_step < sizeof(float) || msg.data().size() < expected_points * point_step)
    {
        return false;
    }

    int z_offset = 0;
    for (const auto& field : msg.fields())
    {
        if (field.name() == "z")
        {
            z_offset = static_cast<int>(field.offset());
            break;
        }
    }

    out.resize(kGridSize);
    const uint8_t* buffer = msg.data().data();
    for (uint32_t i = 0; i < expected_points; ++i)
    {
        const uint32_t offset = i * point_step + static_cast<uint32_t>(z_offset);
        std::memcpy(&out[i], buffer + offset, sizeof(float));
    }
    return true;
}

}  // namespace height_scan
