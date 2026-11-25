#include "livox_sim.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>

namespace
{
constexpr const char* kFrameId = "livox_frame";
constexpr double kPointsPerSecond = 200000.0;
constexpr double kFramePeriod = 0.1;  // driver publish_freq 10
constexpr double kImuPeriod = 0.005;  // 200 Hz
constexpr double kMinRange = 0.1, kMaxRange = 40.0;  // outside: no return
constexpr double kStandardGravity = 9.80665;  // the IMU reports g
constexpr size_t kPointStep = 26;  // LivoxPointXyzrtlt, #pragma pack(1)

using sensor_msgs::msg::dds_::PointField_;
namespace PointField_Constants = sensor_msgs::msg::dds_::PointField_Constants;

int64_t wall_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

void set_stamp(std_msgs::msg::dds_::Header_& h, int64_t ns)
{
    h.stamp().sec() = static_cast<int32_t>(ns / 1000000000);
    h.stamp().nanosec() = static_cast<uint32_t>(ns % 1000000000);
    h.frame_id() = kFrameId;
}

// (N, 2) little-endian float32 .npy, C order: azimuth, elevation in radians.
std::vector<float> load_npy_f32_n2(const std::string& path, std::string& err)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) { err = "cannot open " + path; return {}; }
    char magic[6];
    uint8_t ver[2];
    f.read(magic, 6);
    f.read(reinterpret_cast<char*>(ver), 2);
    if (std::memcmp(magic, "\x93NUMPY", 6) != 0) { err = "not a .npy file"; return {}; }
    uint32_t hlen = 0;
    if (ver[0] == 1) { uint16_t h; f.read(reinterpret_cast<char*>(&h), 2); hlen = h; }
    else f.read(reinterpret_cast<char*>(&hlen), 4);
    std::string header(hlen, ' ');
    f.read(header.data(), hlen);
    std::smatch s;
    if (header.find("'<f4'") == std::string::npos || header.find("'fortran_order': False") == std::string::npos
        || !std::regex_search(header, s, std::regex(R"('shape':\s*\((\d+),\s*2\))")))
    {
        err = "expected float32 (N, 2) C-order, got " + header;
        return {};
    }
    std::vector<float> v(std::stoul(s[1]) * 2);
    f.read(reinterpret_cast<char*>(v.data()), v.size() * sizeof(float));
    if (!f) { err = "truncated"; return {}; }
    return v;
}
}  // namespace

LivoxSim::LivoxSim(const mjModel* m, const std::string& pattern_path)
    : imu_pub_("rt/livox/imu"), cloud_pub_("rt/livox/lidar")
{
    site_id_ = mj_name2id(m, mjOBJ_SITE, "livox_lidar");
    const int acc = mj_name2id(m, mjOBJ_SENSOR, "livox_imu_acc");
    const int gyro = mj_name2id(m, mjOBJ_SENSOR, "livox_imu_gyro");
    if (site_id_ < 0 || acc < 0 || gyro < 0)
    {
        std::cerr << "LivoxSim: site livox_lidar / sensors livox_imu_acc, livox_imu_gyro not in the model; off\n";
        return;
    }
    acc_adr_ = m->sensor_adr[acc];
    gyro_adr_ = m->sensor_adr[gyro];

    std::string err;
    const auto seq = load_npy_f32_n2(pattern_path, err);
    if (seq.empty())
    {
        std::cerr << "LivoxSim: scan pattern " << pattern_path << ": " << err << "; off\n";
        return;
    }
    pattern_.resize(seq.size() / 2 * 3);
    for (size_t i = 0; i < seq.size() / 2; ++i)
    {
        const double az = seq[2 * i], el = seq[2 * i + 1];
        pattern_[3 * i] = std::cos(az) * std::cos(el);
        pattern_[3 * i + 1] = std::sin(az) * std::cos(el);
        pattern_[3 * i + 2] = std::sin(el);
    }

    dt_ = m->opt.timestep;
    rays_per_step_ = static_cast<int>(std::lround(kPointsPerSecond * dt_));
    dirs_w_.resize(3 * rays_per_step_);
    dist_.resize(rays_per_step_);
    geomid_.resize(rays_per_step_);

    imu_pub_.InitChannel();
    cloud_pub_.InitChannel();
    publisher_ = std::thread([this] { publish_loop(); });
    ok_ = true;
    std::cout << "LivoxSim: " << rays_per_step_ << " rays/step, " << seq.size() / 2
              << "-ray pattern -> rt/livox/lidar (10 Hz), rt/livox/imu (200 Hz)\n";
}

LivoxSim::~LivoxSim()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
        running_ = false;
    }
    cv_.notify_one();
    if (publisher_.joinable()) publisher_.join();
}

void LivoxSim::step(const mjModel* m, mjData* d)
{
    if (!ok_) return;
    if (d->time <= last_time_)  // reset or reload: start over
    {
        frame_.clear();
        last_time_ = -1.0;
    }
    if (last_time_ < 0.0)
    {
        anchor_ns_ = wall_ns() - static_cast<int64_t>(d->time * 1e9);
        next_imu_time_ = d->time;
    }
    last_time_ = d->time;
    const int64_t t_ns = anchor_ns_ + static_cast<int64_t>(std::llround(d->time * 1e9));

    if (d->time >= next_imu_time_)
    {
        publish_imu(d, t_ns);
        next_imu_time_ += kImuPeriod;
    }

    if (frame_.empty())
    {
        frame_start_ns_ = t_ns;
        frame_start_time_ = d->time;
    }
    cast(m, d, t_ns);
    if (d->time - frame_start_time_ + dt_ >= kFramePeriod - 0.5 * dt_)  // this step ends the 100 ms
    {
        sensor_msgs::msg::dds_::PointCloud2_ msg;
        set_stamp(msg.header(), frame_start_ns_);
        msg.height() = 1;
        msg.width() = static_cast<uint32_t>(frame_.size() / kPointStep);
        msg.fields() = {
            PointField_("x", 0, PointField_Constants::FLOAT32_, 1),
            PointField_("y", 4, PointField_Constants::FLOAT32_, 1),
            PointField_("z", 8, PointField_Constants::FLOAT32_, 1),
            PointField_("intensity", 12, PointField_Constants::FLOAT32_, 1),
            PointField_("tag", 16, PointField_Constants::UINT8_, 1),
            PointField_("line", 17, PointField_Constants::UINT8_, 1),
            PointField_("timestamp", 18, PointField_Constants::FLOAT64_, 1),
        };
        msg.is_bigendian() = false;
        msg.point_step() = kPointStep;
        msg.row_step() = static_cast<uint32_t>(frame_.size());
        msg.is_dense() = true;
        msg.data() = std::move(frame_);
        frame_ = {};
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.size() < 4) queue_.push_back(std::move(msg));  // drop if DDS falls behind
        }
        cv_.notify_one();
    }
}

void LivoxSim::cast(const mjModel* m, mjData* d, int64_t t_ns)
{
    const mjtNum* origin = d->site_xpos + 3 * site_id_;
    const mjtNum* rot = d->site_xmat + 9 * site_id_;  // sensor -> world, row major
    const size_t total = pattern_.size() / 3;
    const size_t first = cursor_;
    for (int i = 0; i < rays_per_step_; ++i)
    {
        const double* s = &pattern_[3 * ((first + i) % total)];
        mju_mulMatVec3(&dirs_w_[3 * i], rot, s);
    }
    cursor_ = (first + rays_per_step_) % total;

    mj_multiRay(m, d, origin, dirs_w_.data(), nullptr, 1, m->site_bodyid[site_id_], geomid_.data(), dist_.data(),
                rays_per_step_, kMaxRange);

    const size_t base = frame_.size();
    frame_.resize(base + rays_per_step_ * kPointStep);
    uint8_t* p = frame_.data() + base;
    const size_t line0 = base / kPointStep;
    for (int i = 0; i < rays_per_step_; ++i, p += kPointStep)
    {
        const double r = dist_[i];
        const bool hit = geomid_[i] >= 0 && r >= kMinRange && r <= kMaxRange;
        const double* s = &pattern_[3 * ((first + i) % total)];
        // Intensity is a constant stand-in (MuJoCo 3.3 multiRay returns no normals).
        const float xyzi[4] = {hit ? float(s[0] * r) : 0.f, hit ? float(s[1] * r) : 0.f,
                               hit ? float(s[2] * r) : 0.f, hit ? 100.f : 0.f};
        const uint8_t tag = 0, line = static_cast<uint8_t>((line0 + i) % 4);
        // Points within a step are evenly spaced in time.
        const double stamp = double(t_ns) + i * (dt_ * 1e9 / rays_per_step_);
        std::memcpy(p, xyzi, 16);
        p[16] = tag;
        p[17] = line;
        std::memcpy(p + 18, &stamp, 8);
    }
}

void LivoxSim::publish_imu(const mjData* d, int64_t t_ns)
{
    sensor_msgs::msg::dds_::Imu_ msg;
    set_stamp(msg.header(), t_ns);
    msg.orientation().w() = 1.0;  // the driver leaves it at identity
    const mjtNum* a = d->sensordata + acc_adr_;
    const mjtNum* g = d->sensordata + gyro_adr_;
    msg.linear_acceleration().x() = a[0] / kStandardGravity;
    msg.linear_acceleration().y() = a[1] / kStandardGravity;
    msg.linear_acceleration().z() = a[2] / kStandardGravity;
    msg.angular_velocity().x() = g[0];
    msg.angular_velocity().y() = g[1];
    msg.angular_velocity().z() = g[2];
    imu_pub_.Write(msg);
}

void LivoxSim::publish_loop()
{
    std::unique_lock<std::mutex> lock(mtx_);
    while (true)
    {
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
        if (!running_) return;
        auto msg = std::move(queue_.front());
        queue_.pop_front();
        lock.unlock();
        cloud_pub_.Write(msg);
        lock.lock();
    }
}
