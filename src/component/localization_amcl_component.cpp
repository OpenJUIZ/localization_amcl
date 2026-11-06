// localization_amcl のコンポーネント (issue #4)。
//
// コンテナ amcl が localization_amcl::Localizer を持ち、トピック odom・scan を購読して自己位置を推定する。
// 型は juiz の標準の型 (src/component/generated/、scripts/gen_types.sh で作る)。
//
//   amcl_set_odometry(odom)       odom (juiz_std.geometry.Pose2DStamped) を購読。最新のオドメトリを覚えるだけ
//   amcl_add_scan(scan)           scan (juiz_std.sensor.LaserScan) を購読。覚えているオドメトリと組にしてフィルタを進め、
//                                 推定した姿勢 (Pose2DStamped、frame map) を返す (conf でトピック amcl_pose に出す)。
//                                 推定した姿勢を ContainerPose として基準 map_frame (既定 map) に書き、odom_frame を
//                                 書いていれば map -> odom を Transform map_frame -> odom_frame の値として書く (set_transform)
//   amcl_set_laser_pose(mount)    lidar_mount (juiz_std.geometry.PoseStamped、ロボットから見た LiDAR) を購読
//   amcl_set_initial_pose(pose, cov_xx, cov_yy, cov_aa)
//                                 initialpose を購読。粒子を pose のまわりに撒き直す (共分散が負ならコンテナの引数の値)
//   amcl_global_localization()    粒子を地図の空きセル全体に撒き直す
//   amcl_set_map(image, resolution, origin, negate, occupied_thresh, free_thresh)
//                                 地図を画像で渡す。mobile_sim や mapper の get_map・get_map_info の結果をそのまま渡せる
//   amcl_load_map(path)           ROS map_server の YAML を読む
//   amcl_get_pose()               推定した姿勢
//   amcl_get_map() / amcl_get_map_info()  地図の画像 (占有 0、空き 254、不明 205) と、解像度・原点・大きさ
//   amcl_get_state()              画面用: 姿勢、共分散、粒子、更新回数など
//   (get_pose・get_map・get_map_info・get_state は readonly の ContainerProcess)
//
// プロセスの関数から漏れた例外は juiz_sdk が捕まえて呼び手にエラーとして返す (juiz 0.2.3、#400)。
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "juiz/juiz.h"
#include "juiz_idl.h"
#include "localization_amcl/localizer.h"
#include "localization_amcl/map.h"
#include "localization_amcl/version.h"

namespace {

using juiz::Value;
using juiz_std::geometry::Pose2DStamped;
using juiz_std::geometry::PoseStamped;
using juiz_std::sensor::LaserScan;
using localization_amcl::Pose2D;

double to_seconds(const juiz_std::common::Time& t) { return static_cast<double>(t.sec) + t.nanosec * 1e-9; }

juiz_std::common::Time to_time(double t) {
    juiz_std::common::Time out;
    const double sec = std::floor(t);
    out.sec = static_cast<int64_t>(sec);
    out.nanosec = static_cast<int32_t>(std::min(999999999LL, std::llround((t - sec) * 1e9)));
    return out;
}

Value pose_value(const Pose2D& p) { return Value{{"x", Value(p.x)}, {"y", Value(p.y)}, {"theta", Value(p.theta)}}; }

double yaw_of(const juiz_std::geometry::Quaternion& q) {
    juiz_quaternion jq;
    jq.x = q.x;
    jq.y = q.y;
    jq.z = q.z;
    jq.w = q.w;
    return juiz::yaw(jq);
}

double value_number(const Value& v, const std::string& what) {
    if (v.isDoubleValue()) return v.doubleValue();
    if (v.isIntValue()) return static_cast<double>(v.intValue());
    throw std::invalid_argument(what + " must be a number");
}

bool file_exists(const std::string& path) {
    std::ifstream f(path);
    return static_cast<bool>(f);
}

// 地図のパスを決める。環境変数 AMCL_MAP > JUIZ_APP_DIR (juiz run の展開先) からの相対 > カレントディレクトリからの相対
std::string resolve_map_path(const std::string& path) {
    if (const char* env = std::getenv("AMCL_MAP")) {
        if (*env) return env;
    }
    if (path.empty()) return path;
    const bool absolute = path[0] == '/' || path[0] == '\\' || (path.size() > 1 && path[1] == ':');
    if (!absolute) {
        if (const char* dir = std::getenv("JUIZ_APP_DIR")) {
            std::string p = std::string(dir) + "/" + path;
            if (file_exists(p)) return p;
        }
    }
    return path;
}

// コンテナの引数のうち、Transformations まわりと地図の設定
struct Settings {
    std::string map_frame = "map";   // 推定した姿勢を書く基準 (空なら書かない)
    std::string odom_frame;          // map_frame -> odom_frame の Transform に map -> odom を書く (空なら書かない)
    bool wait_laser_pose = false;    // lidar_mount を受け取るまでスキャンを捨てる
    std::string map;                 // 起動時に読む map_server の YAML (空なら読まない)
    int state_particles = 300;       // get_state で返す粒子の数の上限
};

Settings parse_settings(const juiz::CapsuleMap& args) {
    Settings s;
    if (args.has("map_frame")) s.map_frame = args.get_string("map_frame");
    if (args.has("odom_frame")) s.odom_frame = args.get_string("odom_frame");
    if (args.has("wait_laser_pose")) s.wait_laser_pose = args.get_bool("wait_laser_pose");
    if (args.has("map")) s.map = args.get_string("map");
    if (args.has("state_particles")) s.state_particles = static_cast<int>(args.get_int("state_particles"));
    return s;
}

// コンテナの引数 (conf の containers の項目) からパラメータを読む。名前は ROS の amcl と同じ。書いていない項目は既定値
localization_amcl::Params parse_params(const juiz::CapsuleMap& args) {
    localization_amcl::Params p;
    auto num = [&args](const char* name, double fallback) {
        if (!args.has(name)) return fallback;
        try {
            return args.get_float(name);
        } catch (...) {
            return static_cast<double>(args.get_int(name));
        }
    };
    auto integer = [&num](const char* name, int fallback) { return static_cast<int>(std::lround(num(name, fallback))); };
    auto flag = [&args](const char* name, bool fallback) { return args.has(name) ? args.get_bool(name) : fallback; };
    auto str = [&args](const char* name, const std::string& fallback) {
        return args.has(name) ? args.get_string(name) : fallback;
    };
    p.min_particles = integer("min_particles", p.min_particles);
    p.max_particles = integer("max_particles", p.max_particles);
    p.kld_err = num("kld_err", p.kld_err);
    p.kld_z = num("kld_z", p.kld_z);
    p.update_min_d = num("update_min_d", p.update_min_d);
    p.update_min_a = num("update_min_a", p.update_min_a);
    p.resample_interval = integer("resample_interval", p.resample_interval);
    p.recovery_alpha_slow = num("recovery_alpha_slow", p.recovery_alpha_slow);
    p.recovery_alpha_fast = num("recovery_alpha_fast", p.recovery_alpha_fast);
    p.selective_resampling = flag("selective_resampling", p.selective_resampling);
    p.odom_model_type = str("odom_model_type", p.odom_model_type);
    p.odom_alpha1 = num("odom_alpha1", p.odom_alpha1);
    p.odom_alpha2 = num("odom_alpha2", p.odom_alpha2);
    p.odom_alpha3 = num("odom_alpha3", p.odom_alpha3);
    p.odom_alpha4 = num("odom_alpha4", p.odom_alpha4);
    p.odom_alpha5 = num("odom_alpha5", p.odom_alpha5);
    p.laser_model_type = str("laser_model_type", p.laser_model_type);
    p.laser_max_beams = integer("laser_max_beams", p.laser_max_beams);
    p.laser_min_range = num("laser_min_range", p.laser_min_range);
    p.laser_max_range = num("laser_max_range", p.laser_max_range);
    p.laser_z_hit = num("laser_z_hit", p.laser_z_hit);
    p.laser_z_short = num("laser_z_short", p.laser_z_short);
    p.laser_z_max = num("laser_z_max", p.laser_z_max);
    p.laser_z_rand = num("laser_z_rand", p.laser_z_rand);
    p.laser_sigma_hit = num("laser_sigma_hit", p.laser_sigma_hit);
    p.laser_lambda_short = num("laser_lambda_short", p.laser_lambda_short);
    p.laser_likelihood_max_dist = num("laser_likelihood_max_dist", p.laser_likelihood_max_dist);
    p.do_beamskip = flag("do_beamskip", p.do_beamskip);
    p.beam_skip_distance = num("beam_skip_distance", p.beam_skip_distance);
    p.beam_skip_threshold = num("beam_skip_threshold", p.beam_skip_threshold);
    p.beam_skip_error_threshold = num("beam_skip_error_threshold", p.beam_skip_error_threshold);
    p.initial_pose_x = num("initial_pose_x", p.initial_pose_x);
    p.initial_pose_y = num("initial_pose_y", p.initial_pose_y);
    p.initial_pose_a = num("initial_pose_a", p.initial_pose_a);
    p.initial_cov_xx = num("initial_cov_xx", p.initial_cov_xx);
    p.initial_cov_yy = num("initial_cov_yy", p.initial_cov_yy);
    p.initial_cov_aa = num("initial_cov_aa", p.initial_cov_aa);
    p.seed = static_cast<unsigned long>(num("seed", 0.0));
    if (args.has("laser_pose")) {
        const Value lp(args.get_object("laser_pose"));
        const auto& m = lp.objectValue();
        auto get = [&m](const char* k) {
            auto it = m.find(k);
            return it == m.end() ? 0.0 : value_number(it->second, std::string("laser_pose.") + k);
        };
        p.laser_pose = Pose2D{get("x"), get("y"), get("theta")};
    }
    return p;
}

// juiz の画像 -> 1 画素あたりのチャンネル数と、RGB の並びの画素
std::vector<uint8_t> image_pixels(const juiz::Image& img, int& channels) {
    const juiz::ImageView v = img.view();
    std::vector<uint8_t> out;
    switch (v.format) {
    case juiz::PixelFormat::L8: channels = 1; break;
    case juiz::PixelFormat::LA8: channels = 2; break;
    case juiz::PixelFormat::RGB8:
    case juiz::PixelFormat::BGR8: channels = 3; break;
    case juiz::PixelFormat::RGBA8:
    case juiz::PixelFormat::BGRA8: channels = 4; break;
    default: throw std::invalid_argument("map image must be 8 bit (L8, LA8, RGB8, RGBA8, BGR8 or BGRA8)");
    }
    out.reserve(static_cast<size_t>(v.width) * v.height * channels);
    for (uint32_t y = 0; y < v.height; ++y) {
        const uint8_t* row = v.row(y);
        out.insert(out.end(), row, row + static_cast<size_t>(v.width) * channels);
    }
    // 平均を取るだけなので BGR と RGB の違いは結果に効かない
    return out;
}

}  // namespace

class AmclContainer {
public:
    AmclContainer(const localization_amcl::Params& params, const Settings& settings)
        : loc_(params), settings_(settings) {
        if (!settings_.map.empty()) {
            // 地図が読めなくてもシステムは起動させ、理由を get_state の map_error で見せる (あとで set_map・load_map できる)
            try {
                load_map_locked(settings_.map);
            } catch (const std::exception& e) {
                map_error_ = e.what();
            }
        }
    }

    const std::string& map_frame() const { return settings_.map_frame; }
    const std::string& odom_frame() const { return settings_.odom_frame; }

    bool set_odometry(const Pose2DStamped& odom) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_ = Pose2D{odom.pose.position.x, odom.pose.position.y, odom.pose.theta};
        odom_time_ = to_seconds(odom.header.stamp);
        have_odom_ = true;
        return true;
    }

    // スキャンでフィルタを進める。推定姿勢を書いてよい状態なら true
    bool add_scan(const LaserScan& v) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++scans_;
        scan_time_ = to_seconds(v.header.stamp);
        if (!have_odom_ || !loc_.has_map() || (settings_.wait_laser_pose && !have_laser_pose_)) {
            ++dropped_;
            return false;
        }
        try {
            localization_amcl::Scan scan;
            scan.stamp = scan_time_;
            scan.angle_min = v.angle_min;
            scan.angle_increment = v.angle_increment;
            scan.range_min = v.range_min;
            scan.range_max = v.range_max;
            scan.ranges = v.ranges;
            loc_.add_scan(scan, odom_);
            // 描画用に、最後のスキャンを (間引いて) 覚える
            last_scan_.clear();
            const size_t step = std::max<size_t>(1, v.ranges.size() / 180);
            for (size_t i = 0; i < v.ranges.size(); i += step) {
                last_scan_.push_back(Value(std::vector<double>{v.angle_min + i * v.angle_increment, v.ranges[i]}));
            }
            last_scan_max_ = v.range_max;
            return true;
        } catch (const std::exception& e) {
            ++dropped_;
            last_error_ = e.what();
            return false;
        }
    }

    bool set_laser_pose(const PoseStamped& mount) {
        std::lock_guard<std::mutex> lock(mutex_);
        have_laser_pose_ = true;
        return loc_.set_laser_pose(Pose2D{mount.pose.position.x, mount.pose.position.y, yaw_of(mount.pose.orientation)});
    }

    bool set_initial_pose(const Pose2DStamped& p, double cxx, double cyy, double caa) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& params = loc_.params();
        localization_amcl::Covariance cov{};
        cov[0] = cxx >= 0.0 ? cxx : params.initial_cov_xx;
        cov[4] = cyy >= 0.0 ? cyy : params.initial_cov_yy;
        cov[8] = caa >= 0.0 ? caa : params.initial_cov_aa;
        loc_.set_initial_pose(Pose2D{p.pose.position.x, p.pose.position.y, p.pose.theta}, cov);
        return true;
    }

    bool global_localization() {
        std::lock_guard<std::mutex> lock(mutex_);
        loc_.global_localization();
        return true;
    }

    bool set_map(const juiz::Image& image, double resolution, const std::vector<Value>& origin, bool negate,
                 double occupied_thresh, double free_thresh) {
        localization_amcl::MapMeta meta;
        meta.resolution = resolution;
        if (origin.size() < 2) throw std::invalid_argument("origin must be [x, y, theta]");
        meta.origin = Pose2D{value_number(origin[0], "origin[0]"), value_number(origin[1], "origin[1]"),
                             origin.size() > 2 ? value_number(origin[2], "origin[2]") : 0.0};
        meta.negate = negate;
        meta.occupied_thresh = occupied_thresh;
        meta.free_thresh = free_thresh;
        int channels = 0;
        const std::vector<uint8_t> pixels = image_pixels(image, channels);
        const auto grid = localization_amcl::grid_from_image(pixels.data(), static_cast<int>(image.width()),
                                                             static_cast<int>(image.height()), channels, meta);
        std::lock_guard<std::mutex> lock(mutex_);
        loc_.set_map(grid);
        map_source_ = "image";
        map_error_.clear();
        ++map_version_;
        return true;
    }

    std::string load_map(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        return load_map_locked(path);
    }

    Pose2DStamped pose() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return pose_locked();
    }

    std::optional<Pose2D> map_to_odom() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (loc_.updates() == 0) return std::nullopt;
        return loc_.map_to_odom();
    }

    std::optional<juiz::Image> map_image() const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& g = loc_.map();
        if (g.empty()) return std::nullopt;
        const std::vector<uint8_t> px = localization_amcl::grid_to_pixels(g);
        return juiz::Image(static_cast<uint32_t>(g.width), static_cast<uint32_t>(g.height), juiz::PixelFormat::L8,
                           px.data());
    }

    Value map_info() const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& g = loc_.map();
        return Value{{"resolution", Value(g.resolution)},
                     {"origin", Value(std::vector<double>{g.origin.x, g.origin.y, g.origin.theta})},
                     {"width", Value(static_cast<int64_t>(g.width))},
                     {"height", Value(static_cast<int64_t>(g.height))},
                     {"version", Value(map_version_)},
                     {"source", Value(map_source_)},
                     {"occupied_thresh", Value(0.65)},
                     {"free_thresh", Value(0.196)},
                     {"negate", Value(static_cast<int64_t>(0))}};
    }

    Value state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Value> particles;
        for (const auto& p : loc_.particles(static_cast<size_t>(std::max(0, settings_.state_particles)))) {
            particles.push_back(Value(std::vector<double>{p.pose.x, p.pose.y, p.pose.theta}));
        }
        const auto cov = loc_.covariance();
        return Value{{"has_map", Value(loc_.has_map())},
                     {"map_source", Value(map_source_)},
                     {"map_version", Value(map_version_)},
                     {"map_error", Value(map_error_)},
                     {"updates", Value(static_cast<int64_t>(loc_.updates()))},
                     {"scans", Value(scans_)},
                     {"dropped", Value(dropped_)},
                     {"time", Value(scan_time_)},
                     {"pose", pose_value(pose_locked_2d())},
                     {"covariance", Value(std::vector<double>(cov.begin(), cov.end()))},
                     {"odom", pose_value(odom_)},
                     {"map_to_odom", pose_value(loc_.map_to_odom())},
                     {"particle_count", Value(static_cast<int64_t>(loc_.particle_count()))},
                     {"particles", Value(particles)},
                     {"scan", Value(last_scan_)},
                     {"scan_range_max", Value(last_scan_max_)},
                     {"laser_pose", pose_value(loc_.params().laser_pose)},
                     {"waiting_laser_pose", Value(settings_.wait_laser_pose && !have_laser_pose_)},
                     {"map_frame", Value(settings_.map_frame)},
                     {"odom_frame", Value(settings_.odom_frame)},
                     {"pose_error", Value(pose_error_)},
                     {"last_error", Value(last_error_)}};
    }

    void record_pose_error(const std::string& what) {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_error_ = what;
    }

private:
    std::string load_map_locked(const std::string& path) {
        const std::string resolved = resolve_map_path(path);
        loc_.set_map(localization_amcl::load_map(resolved));
        map_source_ = resolved;
        map_error_.clear();
        ++map_version_;
        return resolved;
    }

    Pose2D pose_locked_2d() const { return loc_.pose(); }

    Pose2DStamped pose_locked() const {
        const Pose2D p = loc_.pose();
        Pose2DStamped out;
        out.header.stamp = to_time(have_odom_ ? odom_time_ : scan_time_);
        out.header.frame_id = settings_.map_frame.empty() ? "map" : settings_.map_frame;
        out.pose.position.x = p.x;
        out.pose.position.y = p.y;
        out.pose.theta = p.theta;
        return out;
    }

    // 読むだけの処理 (readonly の ContainerProcess) は const で呼ぶので mutable にする
    mutable std::mutex mutex_;
    localization_amcl::Localizer loc_;
    Settings settings_;
    std::string map_source_;
    std::string map_error_;
    int64_t map_version_ = 0;
    bool have_laser_pose_ = false;
    bool have_odom_ = false;
    Pose2D odom_;
    double odom_time_ = 0.0;
    double scan_time_ = 0.0;
    int64_t scans_ = 0;
    int64_t dropped_ = 0;
    std::vector<Value> last_scan_;
    double last_scan_max_ = 0.0;
    std::string last_error_;
    std::string pose_error_;
};

static auto amcl_manif = ContainerManifest("amcl");
AmclContainer* amcl(juiz::CapsuleMap args) { return new AmclContainer(parse_params(args), parse_settings(args)); }
COMPONENT_CONTAINER_FACTORY(amcl_manif, amcl);

static auto amcl_set_odometry_manif =
    ProcessManifest("amcl_set_odometry").add_typed_arg<Pose2DStamped>("odom", "odometry pose");
std::optional<bool> amcl_set_odometry(AmclContainer* c, const Pose2DStamped& odom) { return c->set_odometry(odom); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_set_odometry_manif, amcl_set_odometry);

static auto amcl_add_scan_manif = ProcessManifest("amcl_add_scan")
                                      .add_typed_arg<LaserScan>("scan", "laser scan")
                                      .returns<Pose2DStamped>();
std::optional<Pose2DStamped> amcl_add_scan(AmclContainer* c, const LaserScan& scan) {
    const bool ok = c->add_scan(scan);
    const Pose2DStamped p = c->pose();
    if (ok && !c->map_frame().empty()) {
        // 推定した姿勢を、基準 map_frame から見たこのコンテナの位置としてスキャンのたびに書く
        // (conf に map_frame -> このコンテナ の Transform が要る)。書けなくても推定は続ける
        try {
            juiz::set_pose(c, c->map_frame(), juiz::make_pose_2d(p.pose.position.x, p.pose.position.y, p.pose.theta));
            const auto m2o = c->map_to_odom();
            if (m2o && !c->odom_frame().empty()) {
                juiz::set_transform(c, c->map_frame(), c->odom_frame(), juiz::make_transform_2d(m2o->x, m2o->y, m2o->theta));
            }
        } catch (const juiz::TransformationsError& e) {
            c->record_pose_error(e.what());
        }
    }
    return p;
}
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_add_scan_manif, amcl_add_scan);

static auto amcl_set_laser_pose_manif =
    ProcessManifest("amcl_set_laser_pose").add_typed_arg<PoseStamped>("mount", "LiDAR pose seen from the robot");
std::optional<bool> amcl_set_laser_pose(AmclContainer* c, const PoseStamped& mount) { return c->set_laser_pose(mount); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_set_laser_pose_manif, amcl_set_laser_pose);

static auto amcl_set_initial_pose_manif =
    ProcessManifest("amcl_set_initial_pose")
        .add_typed_arg<Pose2DStamped>("pose", "initial pose in the map frame")
        .add_float_arg("cov_xx", "variance of x [m^2] (negative: the container's initial_cov_xx)", -1.0)
        .add_float_arg("cov_yy", "variance of y [m^2] (negative: the container's initial_cov_yy)", -1.0)
        .add_float_arg("cov_aa", "variance of the heading [rad^2] (negative: the container's initial_cov_aa)", -1.0);
std::optional<bool> amcl_set_initial_pose(AmclContainer* c, const Pose2DStamped& pose, double cxx, double cyy, double caa) {
    return c->set_initial_pose(pose, cxx, cyy, caa);
}
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_set_initial_pose_manif, amcl_set_initial_pose);

static auto amcl_global_localization_manif = ProcessManifest("amcl_global_localization");
std::optional<bool> amcl_global_localization(AmclContainer* c) { return c->global_localization(); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_global_localization_manif, amcl_global_localization);

static auto amcl_set_map_manif =
    ProcessManifest("amcl_set_map")
        .add_image_arg("image", "map image (white = free, black = occupied, as map_server)")
        .add_float_arg("resolution", "meters per pixel", 0.05)
        .add_array_arg("origin", "[x, y, theta] of the lower-left pixel")
        .add_bool_arg("negate", "swap black and white", false)
        .add_float_arg("occupied_thresh", "occupied above this probability", 0.65)
        .add_float_arg("free_thresh", "free below this probability", 0.196);
std::optional<bool> amcl_set_map(AmclContainer* c, const juiz::Image& image, double resolution,
                                 const std::vector<juiz::Value>& origin, bool negate, double occupied_thresh,
                                 double free_thresh) {
    return c->set_map(image, resolution, origin, negate, occupied_thresh, free_thresh);
}
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_set_map_manif, amcl_set_map);

static auto amcl_load_map_manif =
    ProcessManifest("amcl_load_map").add_string_arg("path", "map_server YAML", "assets/maps/default.yaml");
std::optional<std::string> amcl_load_map(AmclContainer* c, const std::string& path) { return c->load_map(path); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_load_map_manif, amcl_load_map);

static auto amcl_get_pose_manif = ProcessManifest("amcl_get_pose").readonly().returns<Pose2DStamped>();
std::optional<Pose2DStamped> amcl_get_pose(const AmclContainer* c) { return c->pose(); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_get_pose_manif, amcl_get_pose);

static auto amcl_get_map_manif = ProcessManifest("amcl_get_map").readonly();
std::optional<juiz::Image> amcl_get_map(const AmclContainer* c) { return c->map_image(); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_get_map_manif, amcl_get_map);

static auto amcl_get_map_info_manif = ProcessManifest("amcl_get_map_info").readonly();
std::optional<juiz::Value> amcl_get_map_info(const AmclContainer* c) { return c->map_info(); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_get_map_info_manif, amcl_get_map_info);

static auto amcl_get_state_manif = ProcessManifest("amcl_get_state").readonly();
std::optional<juiz::Value> amcl_get_state(const AmclContainer* c) { return c->state(); }
COMPONENT_CONTAINER_PROCESS_FACTORY(AmclContainer, amcl_get_state_manif, amcl_get_state);

static auto localization_amcl_version_manif = ProcessManifest("localization_amcl_version");
std::optional<std::string> localization_amcl_version() { return localization_amcl::version(); }
COMPONENT_PROCESS_FACTORY(localization_amcl_version_manif, localization_amcl_version);

auto component_manifest() {
    return ComponentManifest("localization_amcl")
        .add_process(localization_amcl_version_manifest())
        .add_container(amcl_manifest()
                           .add_process(amcl_set_odometry_manifest())
                           .add_process(amcl_add_scan_manifest())
                           .add_process(amcl_set_laser_pose_manifest())
                           .add_process(amcl_set_initial_pose_manifest())
                           .add_process(amcl_global_localization_manifest())
                           .add_process(amcl_set_map_manifest())
                           .add_process(amcl_load_map_manifest())
                           .add_process(amcl_get_pose_manifest())
                           .add_process(amcl_get_map_manifest())
                           .add_process(amcl_get_map_info_manifest())
                           .add_process(amcl_get_state_manifest()));
}
DEFINE_COMPONENT_MANIFEST_ENTRY_POINT(component_manifest)
