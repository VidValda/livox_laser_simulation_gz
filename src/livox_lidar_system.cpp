/*
 * Livox lidar simulation system plugin for Gazebo Harmonic (gz-sim 8).
 *
 * Ported from https://github.com/Livox-SDK/livox_laser_simulation
 * (Gazebo Classic + ROS 1) to the gz::sim::System interface.
 *
 * The original plugin cast individual ODE rays at the Livox CSV directions.
 * That maps to gz::rendering::RayQuery (CPU, serial) on Harmonic, which is far
 * too slow (~2% RTF for a usable cloud). Instead we exploit the GPU: subscribe
 * to a HIGH-resolution gpu_ray raster cloud (rendered in one GPU pass) and each
 * frame select only the cells hit by the next batch of mid360.csv ray
 * directions. Different batch each frame -> the real non-repetitive petal
 * coverage, at GPU speed. A fine raster (e.g. 720x128) keeps the snap error
 * below the sensor's real angular resolution, so it does NOT look like stripes.
 *
 * SDF usage (inside the lidar model; relative topics resolve against the
 * model's scoped topic, frame_id defaults to <parent>/<model>/lidar_3d):
 *   <plugin filename="LivoxLidarSystem"
 *          name="livox_laser_simulation_gz::LivoxLidarSystem">
 *     <input_topic>link/lidar_3d/sensor/gpu_ray/scan/points</input_topic>
 *     <csv_file_name>mid360.csv</csv_file_name>
 *     <samples>24000</samples>
 *     <downsample>1</downsample>
 *     <update_rate>10</update_rate>
 *     <output_topic>lidar</output_topic>
 *     <min_range>0.1</min_range>
 *     <max_range>40.0</max_range>
 *   </plugin>
 */

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace livox_laser_simulation_gz {

struct ScanPoint {
    double azimuth;   // rad
    double zenith;    // rad (elevation, shifted by -pi/2 from CSV zenith degrees)
};

static bool LoadCsv(const std::string &path, std::vector<ScanPoint> &out) {
    std::ifstream fs(path);
    if (!fs.is_open()) return false;
    std::string line;
    std::getline(fs, line); // skip header
    while (std::getline(fs, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        try {
            std::getline(ss, tok, ','); // time (unused)
            std::getline(ss, tok, ',');
            double az = std::stod(tok) * M_PI / 180.0;
            std::getline(ss, tok, ',');
            // CSV "zenith" is measured from straight up, so elevation = 90 - zenith.
            // Mid-360: zenith [37.8, 97.2] deg -> elevation [+52.2, -7.2] deg.
            double ze = M_PI_2 - std::stod(tok) * M_PI / 180.0;
            out.push_back({az, ze});
        } catch (...) { continue; }
    }
    return !out.empty();
}

class LivoxLidarSystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager & /*_ecm*/,
                   gz::sim::EventManager & /*_eventMgr*/) override
    {
        auto sdf = const_cast<sdf::Element*>(_sdf.get());

        std::string csvFile = "mid360.csv";
        if (sdf->HasElement("csv_file_name"))
            csvFile = sdf->Get<std::string>("csv_file_name");
        if (sdf->HasElement("samples"))
            samplesPerFrame_ = sdf->Get<int>("samples");
        if (sdf->HasElement("downsample"))
            downsample_ = std::max(1, sdf->Get<int>("downsample"));
        if (sdf->HasElement("update_rate"))
            updateRate_ = sdf->Get<double>("update_rate");
        if (sdf->HasElement("min_range"))
            minRange_ = sdf->Get<double>("min_range");
        if (sdf->HasElement("max_range"))
            maxRange_ = sdf->Get<double>("max_range");
        if (sdf->HasElement("frame_id"))
            frameId_ = sdf->Get<std::string>("frame_id");
        // Raster gpu_ray FOV (MUST match the lidar_3d override model). Used to
        // map CSV ray directions to raster grid cells directly -- robust,
        // independent of what the scene happens to return.
        if (sdf->HasElement("h_fov_min")) hFovMin_ = sdf->Get<double>("h_fov_min");
        if (sdf->HasElement("h_fov_max")) hFovMax_ = sdf->Get<double>("h_fov_max");
        if (sdf->HasElement("v_fov_min")) vFovMin_ = sdf->Get<double>("v_fov_min");
        if (sdf->HasElement("v_fov_max")) vFovMax_ = sdf->Get<double>("v_fov_max");

        entity_ = _entity;
        if (sdf->HasElement("output_topic"))
            outputTopic_ = sdf->Get<std::string>("output_topic");
        if (sdf->HasElement("input_topic"))
            inputTopic_ = sdf->Get<std::string>("input_topic");

        if (!LoadCsv(csvFile, pattern_)) {
            if (auto *e = std::getenv("AMENT_PREFIX_PATH")) {
                std::istringstream ss(e);
                std::string p;
                while (std::getline(ss, p, ':')) {
                    auto tp = p + "/share/livox_laser_simulation_gz/scan_mode/" + csvFile;
                    if (LoadCsv(tp, pattern_)) break;
                }
            }
        }
        if (pattern_.empty()) {
            gzerr << "[LivoxLidarSystem] Failed to load CSV: " << csvFile << "\n";
            return;
        }
        gzmsg << "[LivoxLidarSystem] Loaded " << pattern_.size()
              << " scan points from " << csvFile
              << ", samples/frame=" << samplesPerFrame_
              << ", downsample=" << downsample_ << "\n";
    }

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override
    {
        if (!connected_ && !pattern_.empty()) connect(_ecm);
        if (_info.paused) return;
        std::lock_guard<std::mutex> lk(mutex_);
        simTime_ = _info.simTime;
    }

private:
    // Deferred to the first update: a spawned model is configured before it is attached to the world.
    void connect(const gz::sim::EntityComponentManager &_ecm)
    {
        const std::string scope = gz::sim::topicFromScopedName(entity_, _ecm, false);
        auto resolve = [&](const std::string &t) { return !t.empty() && t[0] == '/' ? t : scope + "/" + t; };
        if (frameId_.empty())
            frameId_ = gz::sim::removeParentScope(
                gz::sim::scopedName(entity_, _ecm, "/", false), "/") + "/lidar_3d";
        const std::string in = resolve(inputTopic_), out = resolve(outputTopic_);
        pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(out);
        node_.Subscribe(in, &LivoxLidarSystem::OnCloud, this);
        connected_ = true;
        gzmsg << "[LivoxLidarSystem] Subscribing to " << in
              << ", publishing " << out << " in " << frameId_ << "\n";
    }

    void OnCloud(const gz::msgs::PointCloudPacked &_msg)
    {
        // Rate-limit output to the configured update_rate (wall clock).
        auto now = std::chrono::steady_clock::now();
        if (lastPub_.time_since_epoch().count() > 0) {
            double dt = std::chrono::duration<double>(now - lastPub_).count();
            if (dt < 1.0 / updateRate_) return;
        }
        lastPub_ = now;

        if (!rasterReady_) {
            buildRasterMap(_msg);
            rasterReady_ = true;
        }
        if (rasterIndices_.empty()) return;

        int xOff = -1, yOff = -1, zOff = -1, iOff = -1;
        for (const auto &f : _msg.field()) {
            if (f.name() == "x") xOff = f.offset();
            else if (f.name() == "y") yOff = f.offset();
            else if (f.name() == "z") zOff = f.offset();
            else if (f.name() == "intensity") iOff = f.offset();
        }
        if (xOff < 0 || yOff < 0 || zOff < 0) return;

        const int ps = _msg.point_step();
        const int totalPix = _msg.width() * _msg.height();
        const char *in = _msg.data().data();

        gz::msgs::PointCloudPacked out;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            auto *stamp = out.mutable_header()->mutable_stamp();
            auto sec = std::chrono::duration_cast<std::chrono::seconds>(simTime_);
            stamp->set_sec(sec.count());
            stamp->set_nsec(std::chrono::duration_cast<std::chrono::nanoseconds>(
                simTime_ - sec).count());
        }
        auto *fd = out.mutable_header()->add_data();
        fd->set_key("frame_id");
        fd->add_value(frameId_);

        auto addField = [&](const char *name, int offset) {
            auto *f = out.add_field();
            f->set_name(name); f->set_offset(offset);
            f->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
            f->set_count(1);
        };
        addField("x", 0); addField("y", 4); addField("z", 8); addField("intensity", 12);
        out.set_point_step(16);
        out.set_is_bigendian(false);
        out.set_is_dense(true);

        std::string &od = *out.mutable_data();
        int count = 0;

        int64_t end = cursor_ + samplesPerFrame_;
        for (int64_t k = cursor_; k < end; k += downsample_) {
            int64_t pi = k % static_cast<int64_t>(pattern_.size());
            int idx = rasterIndices_[pi];
            if (idx < 0 || idx >= totalPix) continue;

            const char *pt = in + idx * ps;
            float x, y, z, intensity = 1.0f;
            std::memcpy(&x, pt + xOff, 4);
            std::memcpy(&y, pt + yOff, 4);
            std::memcpy(&z, pt + zOff, 4);
            if (iOff >= 0) std::memcpy(&intensity, pt + iOff, 4);

            float r = std::sqrt(x * x + y * y + z * z);
            if (r < minRange_ || r > maxRange_) continue;

            od.append(reinterpret_cast<const char*>(&x), 4);
            od.append(reinterpret_cast<const char*>(&y), 4);
            od.append(reinterpret_cast<const char*>(&z), 4);
            od.append(reinterpret_cast<const char*>(&intensity), 4);
            count++;
        }
        cursor_ = end % static_cast<int64_t>(pattern_.size());

        out.set_height(1);
        out.set_width(count);
        out.set_row_step(out.point_step() * count);
        pub_.Publish(out);
    }

    void buildRasterMap(const gz::msgs::PointCloudPacked &_msg)
    {
        // Map each CSV ray (az, ze) to a raster grid cell using the KNOWN gpu_ray
        // FOV (set to match the lidar_3d override). The gpu_ray cloud is organized
        // height(vertical) x width(horizontal), row-major, row0=v_fov_min,
        // col0=h_fov_min. No dependence on returned data (robust to misses).
        int w = _msg.width(), h = _msg.height();
        double hR = hFovMax_ - hFovMin_, vR = vFovMax_ - vFovMin_;
        if (w < 2 || h < 2 || hR < 1e-6 || vR < 1e-6) {
            gzerr << "[LivoxLidarSystem] bad raster (" << w << "x" << h << ")\n";
            return;
        }
        gzmsg << "[LivoxLidarSystem] Raster " << w << "x" << h
              << " mapped to FOV az=[" << hFovMin_*180/M_PI << "," << hFovMax_*180/M_PI
              << "] ze=[" << vFovMin_*180/M_PI << "," << vFovMax_*180/M_PI << "] deg\n";

        rasterIndices_.resize(pattern_.size());
        for (size_t i = 0; i < pattern_.size(); ++i) {
            double az = pattern_[i].azimuth;
            if (az > M_PI) az -= 2.0 * M_PI;
            double ze = pattern_[i].zenith;
            // CSV rays outside the raster FOV can't be sampled -> mark invalid.
            if (az < hFovMin_ || az > hFovMax_ || ze < vFovMin_ || ze > vFovMax_) {
                rasterIndices_[i] = -1;
                continue;
            }
            int col = std::clamp((int)((az - hFovMin_) / hR * (w - 1)), 0, w - 1);
            int row = std::clamp((int)((ze - vFovMin_) / vR * (h - 1)), 0, h - 1);
            rasterIndices_[i] = row * w + col;
        }
    }

    std::vector<ScanPoint> pattern_;
    std::vector<int> rasterIndices_;
    bool rasterReady_ = false;
    int64_t cursor_ = 0;
    int samplesPerFrame_ = 24000;
    int downsample_ = 1;

    double minRange_ = 0.1, maxRange_ = 40.0, updateRate_ = 10.0;
    // Raster gpu_ray FOV (defaults match the Mid-360 lidar_3d override).
    // Mid-360 vertical FOV: -7.2 deg (-0.126 rad) .. +52.2 deg (+0.911 rad).
    double hFovMin_ = -M_PI, hFovMax_ = M_PI;
    double vFovMin_ = -0.126, vFovMax_ = 0.911;
    std::string frameId_;
    std::string inputTopic_ = "link/lidar_3d/sensor/gpu_ray/scan/points";
    std::string outputTopic_ = "lidar";
    gz::sim::Entity entity_ = gz::sim::kNullEntity;
    bool connected_ = false;

    std::mutex mutex_;
    std::chrono::steady_clock::duration simTime_{0};
    std::chrono::steady_clock::time_point lastPub_;

    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_;
};

}  // namespace livox_laser_simulation_gz

GZ_ADD_PLUGIN(
    livox_laser_simulation_gz::LivoxLidarSystem,
    gz::sim::System,
    livox_laser_simulation_gz::LivoxLidarSystem::ISystemConfigure,
    livox_laser_simulation_gz::LivoxLidarSystem::ISystemPostUpdate)
