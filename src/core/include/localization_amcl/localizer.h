// OpenJUIZ/amcl の粒子フィルタで自己位置を推定する。juiz には依存しない。
//
// 手順は ROS の amcl ノードと同じ:
//   地図を受けたら尤度場を作り、初期姿勢のまわりに粒子を撒く。
//   スキャンとそのときのオドメトリを受け、前回の更新から update_min_d か update_min_a 動いていたら
//   動作モデル -> レーザーモデル -> (resample_interval 回に 1 回) リサンプリング、と進める。
//   推定姿勢は重みが最大のクラスタの平均。map -> odom は「推定姿勢 × 更新時のオドメトリの逆」。
//
// スレッド安全ではない (amcl の乱数は大域の drand48)。呼び手が 1 つのロックで守る。
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "localization_amcl/map.h"

namespace localization_amcl {

// パラメータ。名前と既定値は ROS の amcl に合わせる
struct Params {
    // 粒子フィルタ
    int min_particles = 100;
    int max_particles = 5000;
    double kld_err = 0.01;
    double kld_z = 0.99;
    double update_min_d = 0.2;              // [m]
    double update_min_a = 3.14159265358979323846 / 6.0;  // [rad]
    int resample_interval = 2;
    double recovery_alpha_slow = 0.0;       // 0 で使わない
    double recovery_alpha_fast = 0.0;
    bool selective_resampling = false;
    // オドメトリのモデル (diff / omni / diff-corrected / omni-corrected)
    std::string odom_model_type = "diff";
    double odom_alpha1 = 0.2;
    double odom_alpha2 = 0.2;
    double odom_alpha3 = 0.2;
    double odom_alpha4 = 0.2;
    double odom_alpha5 = 0.2;
    // レーザーのモデル (likelihood_field / beam / likelihood_field_prob)
    std::string laser_model_type = "likelihood_field";
    int laser_max_beams = 30;
    double laser_min_range = -1.0;          // 負ならスキャンの range_min
    double laser_max_range = -1.0;          // 負ならスキャンの range_max
    double laser_z_hit = 0.95;
    double laser_z_short = 0.1;
    double laser_z_max = 0.05;
    double laser_z_rand = 0.05;
    double laser_sigma_hit = 0.2;
    double laser_lambda_short = 0.1;
    double laser_likelihood_max_dist = 2.0;
    bool do_beamskip = false;
    double beam_skip_distance = 0.5;
    double beam_skip_threshold = 0.3;
    double beam_skip_error_threshold = 0.9;
    // 初期姿勢 (map の座標) と共分散
    double initial_pose_x = 0.0;
    double initial_pose_y = 0.0;
    double initial_pose_a = 0.0;
    double initial_cov_xx = 0.5 * 0.5;
    double initial_cov_yy = 0.5 * 0.5;
    double initial_cov_aa = (3.14159265358979323846 / 12.0) * (3.14159265358979323846 / 12.0);
    // ロボットの中心から見た LiDAR の位置
    Pose2D laser_pose;
    // 乱数の種 (0 なら時刻から)。Windows では srand
    unsigned long seed = 0;
};

// レーザーのスキャン 1 回分 (juiz_std.sensor.LaserScan と同じ項目)
struct Scan {
    double stamp = 0.0;
    double angle_min = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
    std::vector<float> ranges;
};

// 共分散 (x, y, theta の 3x3、行優先)
using Covariance = std::array<double, 9>;

struct Particle {
    Pose2D pose;
    double weight = 0.0;
};

class Localizer {
public:
    explicit Localizer(const Params& params);
    ~Localizer();
    Localizer(const Localizer&) = delete;
    Localizer& operator=(const Localizer&) = delete;

    // 地図を差し替える。初めての地図なら、パラメータの初期姿勢のまわりに粒子を撒く。
    // 2 回目以降は、今の推定姿勢のまわりに撒き直す (ROS の amcl と同じ)
    void set_map(const OccupancyGrid& grid);
    bool has_map() const;
    const OccupancyGrid& map() const { return grid_; }

    // 初期姿勢 (map の座標) のまわりに粒子を撒き直す。次のスキャンで必ず更新する
    void set_initial_pose(const Pose2D& pose, const Covariance& cov);
    void set_initial_pose(const Pose2D& pose);  // 共分散はパラメータの initial_cov_*
    // 粒子を地図の空きセル全体に撒き直す (ROS の global_localization サービス)
    void global_localization();
    // ロボットの中心から見た LiDAR の位置。変わったら true
    bool set_laser_pose(const Pose2D& pose);

    // スキャンと、そのときのオドメトリ (odom の座標) を渡す。フィルタを更新したら true。
    // 地図が無いときは std::runtime_error
    bool add_scan(const Scan& scan, const Pose2D& odom);

    // 推定姿勢 (map の座標)。最後の更新のあとのオドメトリの動きも足したもの
    Pose2D pose() const;
    // 最後の更新での推定姿勢の共分散
    Covariance covariance() const;
    // map から見た odom の原点 (ROS の map -> odom)
    Pose2D map_to_odom() const { return map_to_odom_; }
    // 粒子 (最大 max 個に間引く。0 なら全部)
    std::vector<Particle> particles(size_t max = 0) const;
    int particle_count() const;
    int updates() const { return updates_; }
    bool initialized() const { return initialized_; }
    const Params& params() const { return params_; }

private:
    struct Impl;
    void rebuild_filter(const Pose2D& mean, const Covariance& cov);
    void compute_estimate();

    Params params_;
    OccupancyGrid grid_;
    std::unique_ptr<Impl> impl_;
    bool initialized_ = false;      // 粒子を撒いたか
    bool have_odom_ = false;        // 更新の基準のオドメトリを持っているか
    bool have_any_odom_ = false;    // オドメトリを 1 度でも受けたか
    bool force_update_ = false;     // 次のスキャンで動いていなくても更新する
    int resample_count_ = 0;
    int updates_ = 0;
    Pose2D last_odom_;              // 最後に受けたオドメトリ
    Pose2D update_odom_;            // 最後に更新したときのオドメトリ
    Pose2D estimate_;               // 最後の更新での推定姿勢
    Covariance covariance_{};
    Pose2D map_to_odom_;
};

// 2 次元の姿勢の計算
Pose2D compose(const Pose2D& a, const Pose2D& b);  // a ⊕ b
Pose2D inverse(const Pose2D& p);
double normalize_angle(double a);

}  // namespace localization_amcl
