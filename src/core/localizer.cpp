#include "localization_amcl/localizer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_laser.h"
#include "amcl/sensors/amcl_odom.h"

namespace localization_amcl {

namespace {

const double kPi = 3.14159265358979323846;

pf_vector_t to_pf(const Pose2D& p) {
    pf_vector_t v = pf_vector_zero();
    v.v[0] = p.x;
    v.v[1] = p.y;
    v.v[2] = p.theta;
    return v;
}

Pose2D from_pf(const pf_vector_t& v) { return Pose2D{v.v[0], v.v[1], v.v[2]}; }

pf_matrix_t to_pf(const Covariance& c) {
    pf_matrix_t m = pf_matrix_zero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m.m[i][j] = c[i * 3 + j];
    return m;
}

amcl::odom_model_t odom_model(const std::string& name) {
    if (name == "diff") return amcl::ODOM_MODEL_DIFF;
    if (name == "omni") return amcl::ODOM_MODEL_OMNI;
    if (name == "diff-corrected") return amcl::ODOM_MODEL_DIFF_CORRECTED;
    if (name == "omni-corrected") return amcl::ODOM_MODEL_OMNI_CORRECTED;
    throw std::invalid_argument("unknown odom_model_type: " + name);
}

void seed_random(unsigned long seed) {
#if defined(_WIN32)
    std::srand(static_cast<unsigned>(seed));
#else
    srand48(static_cast<long>(seed));
#endif
}

}  // namespace

double normalize_angle(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a < 0) a += 2.0 * kPi;
    return a - kPi;
}

Pose2D compose(const Pose2D& a, const Pose2D& b) {
    const double c = std::cos(a.theta), s = std::sin(a.theta);
    return Pose2D{a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalize_angle(a.theta + b.theta)};
}

Pose2D inverse(const Pose2D& p) {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    return Pose2D{-c * p.x - s * p.y, s * p.x - c * p.y, normalize_angle(-p.theta)};
}

// amcl の C の構造体を持つ部分
struct Localizer::Impl {
    map_t* map = nullptr;
    pf_t* pf = nullptr;
    amcl::AMCLOdom odom;
    std::unique_ptr<amcl::AMCLLaser> laser;
    std::vector<int> free_cells;  // 空きセルの番号 (大域的な推定で粒子を撒く先)
    std::mt19937 rng;

    ~Impl() { release(); }

    void release() {
        laser.reset();
        if (pf) pf_free(pf);
        pf = nullptr;
        if (map) map_free(map);
        map = nullptr;
        free_cells.clear();
    }

    // 空きセルから一様に選んだ姿勢 (pf_init_model と、回復のための粒子に使う)
    static pf_vector_t uniform_pose(void* arg) {
        Impl* self = static_cast<Impl*>(arg);
        pf_vector_t p = pf_vector_zero();
        if (self->free_cells.empty()) return p;
        std::uniform_int_distribution<size_t> pick(0, self->free_cells.size() - 1);
        std::uniform_real_distribution<double> angle(-kPi, kPi);
        const int idx = self->free_cells[pick(self->rng)];
        const int i = idx % self->map->size_x;
        const int j = idx / self->map->size_x;
        p.v[0] = MAP_WXGX(self->map, i);
        p.v[1] = MAP_WYGY(self->map, j);
        p.v[2] = angle(self->rng);
        return p;
    }
};

Localizer::Localizer(const Params& params) : params_(params), impl_(new Impl) {
    if (params_.min_particles < 1 || params_.max_particles < params_.min_particles)
        throw std::invalid_argument("need 1 <= min_particles <= max_particles");
    if (params_.resample_interval < 1) throw std::invalid_argument("resample_interval must be >= 1");
    if (params_.laser_max_beams < 2) throw std::invalid_argument("laser_max_beams must be >= 2");
    impl_->odom.SetModel(odom_model(params_.odom_model_type), params_.odom_alpha1, params_.odom_alpha2,
                         params_.odom_alpha3, params_.odom_alpha4, params_.odom_alpha5);
    if (params_.seed != 0) {
        impl_->rng.seed(static_cast<std::mt19937::result_type>(params_.seed));
    } else {
        impl_->rng.seed(std::random_device{}());
    }
    estimate_ = Pose2D{params_.initial_pose_x, params_.initial_pose_y, params_.initial_pose_a};
}

Localizer::~Localizer() = default;

bool Localizer::has_map() const { return impl_->map != nullptr; }

void Localizer::set_map(const OccupancyGrid& grid) {
    if (grid.empty() || grid.data.size() != static_cast<size_t>(grid.width) * grid.height)
        throw std::invalid_argument("map is empty or its size does not match");
    if (!(grid.resolution > 0.0)) throw std::invalid_argument("map resolution must be positive");
    const bool first = !has_map();
    impl_->release();
    grid_ = grid;

    // OccupancyGrid -> map_t (ROS の amcl と同じ変換。origin_x/y は地図の中心)
    map_t* map = map_alloc();
    map->size_x = grid.width;
    map->size_y = grid.height;
    map->scale = grid.resolution;
    map->origin_x = grid.origin.x + (map->size_x / 2) * map->scale;
    map->origin_y = grid.origin.y + (map->size_y / 2) * map->scale;
    map->cells = static_cast<map_cell_t*>(calloc(static_cast<size_t>(map->size_x) * map->size_y, sizeof(map_cell_t)));
    if (!map->cells) {
        map_free(map);
        throw std::bad_alloc();
    }
    for (size_t k = 0; k < grid.data.size(); ++k) {
        const int8_t v = grid.data[k];
        map->cells[k].occ_state = v == 0 ? -1 : (v == 100 ? +1 : 0);
        if (v == 0) impl_->free_cells.push_back(static_cast<int>(k));
    }
    impl_->map = map;

    // レーザーのモデル (尤度場のモデルは、ここで障害物までの距離の表を作る)
    impl_->laser.reset(new amcl::AMCLLaser(static_cast<size_t>(params_.laser_max_beams), map));
    if (params_.laser_model_type == "beam") {
        impl_->laser->SetModelBeam(params_.laser_z_hit, params_.laser_z_short, params_.laser_z_max, params_.laser_z_rand,
                                   params_.laser_sigma_hit, params_.laser_lambda_short, 0.0);
    } else if (params_.laser_model_type == "likelihood_field_prob") {
        impl_->laser->SetModelLikelihoodFieldProb(params_.laser_z_hit, params_.laser_z_rand, params_.laser_sigma_hit,
                                                  params_.laser_likelihood_max_dist, params_.do_beamskip,
                                                  params_.beam_skip_distance, params_.beam_skip_threshold,
                                                  params_.beam_skip_error_threshold);
    } else if (params_.laser_model_type == "likelihood_field") {
        impl_->laser->SetModelLikelihoodField(params_.laser_z_hit, params_.laser_z_rand, params_.laser_sigma_hit,
                                              params_.laser_likelihood_max_dist);
    } else {
        impl_->release();
        throw std::invalid_argument("unknown laser_model_type: " + params_.laser_model_type);
    }
    set_laser_pose(params_.laser_pose);

    // 粒子フィルタ。初めての地図ならパラメータの初期姿勢、そうでなければ今の推定姿勢のまわりに撒く
    impl_->pf = pf_alloc(params_.min_particles, params_.max_particles, params_.recovery_alpha_slow,
                         params_.recovery_alpha_fast, &Impl::uniform_pose, impl_.get());
    impl_->pf->pop_err = params_.kld_err;
    impl_->pf->pop_z = params_.kld_z;
    pf_set_selective_resampling(impl_->pf, params_.selective_resampling ? 1 : 0);
    const Pose2D mean = first ? Pose2D{params_.initial_pose_x, params_.initial_pose_y, params_.initial_pose_a} : estimate_;
    Covariance cov{};
    cov[0] = params_.initial_cov_xx;
    cov[4] = params_.initial_cov_yy;
    cov[8] = params_.initial_cov_aa;
    rebuild_filter(mean, cov);
}

void Localizer::rebuild_filter(const Pose2D& mean, const Covariance& cov) {
    pf_init(impl_->pf, to_pf(mean), to_pf(cov));
    // pf_init は乱数の種を決め直すので、そのあとで種を入れる
    if (params_.seed != 0) seed_random(params_.seed + static_cast<unsigned long>(updates_));
    initialized_ = true;
    have_odom_ = false;  // 次のスキャンのオドメトリを基準にして、動いていなくても更新する
    force_update_ = true;
    resample_count_ = 0;
    estimate_ = mean;
    covariance_ = cov;
    if (have_any_odom_) map_to_odom_ = compose(estimate_, inverse(last_odom_));
}

void Localizer::set_initial_pose(const Pose2D& pose, const Covariance& cov) {
    if (!has_map()) {
        // 地図が来たらここに撒く
        params_.initial_pose_x = pose.x;
        params_.initial_pose_y = pose.y;
        params_.initial_pose_a = pose.theta;
        params_.initial_cov_xx = cov[0];
        params_.initial_cov_yy = cov[4];
        params_.initial_cov_aa = cov[8];
        estimate_ = pose;
        return;
    }
    rebuild_filter(pose, cov);
}

void Localizer::set_initial_pose(const Pose2D& pose) {
    Covariance cov{};
    cov[0] = params_.initial_cov_xx;
    cov[4] = params_.initial_cov_yy;
    cov[8] = params_.initial_cov_aa;
    set_initial_pose(pose, cov);
}

void Localizer::global_localization() {
    if (!has_map()) throw std::runtime_error("no map");
    if (impl_->free_cells.empty()) throw std::runtime_error("the map has no free cells");
    pf_init_model(impl_->pf, &Impl::uniform_pose, impl_.get());
    have_odom_ = false;
    force_update_ = true;
    resample_count_ = 0;
    compute_estimate();
}

bool Localizer::set_laser_pose(const Pose2D& pose) {
    const bool changed = pose.x != params_.laser_pose.x || pose.y != params_.laser_pose.y ||
                         pose.theta != params_.laser_pose.theta;
    params_.laser_pose = pose;
    if (impl_->laser) {
        // 取付の向きはビームの角度に足すので、ここでは 0 にする (ROS の amcl と同じ)
        pf_vector_t lp = pf_vector_zero();
        lp.v[0] = pose.x;
        lp.v[1] = pose.y;
        impl_->laser->SetLaserPose(lp);
    }
    return changed;
}

bool Localizer::add_scan(const Scan& scan, const Pose2D& odom) {
    if (!has_map()) throw std::runtime_error("no map");
    if (scan.ranges.size() < 2) throw std::invalid_argument("scan has fewer than 2 ranges");
    last_odom_ = odom;
    have_any_odom_ = true;

    pf_vector_t delta = pf_vector_zero();
    bool do_action = false;
    if (!have_odom_) {
        // 粒子を撒いた直後: このオドメトリを基準にして、動作モデルは飛ばしてレーザーだけで更新する
        update_odom_ = odom;
        have_odom_ = true;
    } else {
        delta.v[0] = odom.x - update_odom_.x;
        delta.v[1] = odom.y - update_odom_.y;
        delta.v[2] = normalize_angle(odom.theta - update_odom_.theta);
        const bool moved = std::fabs(delta.v[0]) > params_.update_min_d || std::fabs(delta.v[1]) > params_.update_min_d ||
                           std::fabs(delta.v[2]) > params_.update_min_a;
        if (!moved && !force_update_) return false;
        do_action = true;
    }

    if (do_action) {
        amcl::AMCLOdomData odata;
        odata.pose = to_pf(odom);
        odata.delta = delta;
        impl_->odom.UpdateAction(impl_->pf, &odata);
    }

    // スキャン -> AMCLLaserData。角度はロボットから見た向きにする
    amcl::AMCLLaserData ldata;
    ldata.sensor = impl_->laser.get();
    ldata.time = scan.stamp;
    double range_min = scan.range_min;
    if (params_.laser_min_range > range_min) range_min = params_.laser_min_range;
    double range_max = scan.range_max;
    if (params_.laser_max_range > 0.0) range_max = std::min(range_max, params_.laser_max_range);
    ldata.range_max = range_max;
    ldata.range_count = static_cast<int>(scan.ranges.size());
    ldata.ranges = new double[ldata.range_count][2];
    const double angle_min = scan.angle_min + params_.laser_pose.theta;
    for (int i = 0; i < ldata.range_count; ++i) {
        double r = scan.ranges[static_cast<size_t>(i)];
        // 近すぎるビーム・当たらないビーム・不正な値は「最大距離」として扱う
        if (!std::isfinite(r) || r <= range_min || r > range_max) r = range_max;
        ldata.ranges[i][0] = r;
        ldata.ranges[i][1] = angle_min + i * scan.angle_increment;
    }
    impl_->laser->UpdateSensor(impl_->pf, &ldata);

    force_update_ = false;
    update_odom_ = odom;
    ++updates_;
    // クラスタ (推定姿勢) はリサンプリングのときにだけ作り直されるので、推定姿勢と map -> odom もそのときだけ
    // 更新する (ROS の amcl と同じ)。その間の pose() は、前の map -> odom にオドメトリの動きを足したもの
    if (++resample_count_ % params_.resample_interval == 0) {
        pf_update_resample(impl_->pf);
        compute_estimate();
        map_to_odom_ = compose(estimate_, inverse(update_odom_));
    } else if (!do_action) {
        // 粒子を撒いた直後の最初の更新: 撒いた中心を推定姿勢にしておく
        map_to_odom_ = compose(estimate_, inverse(update_odom_));
    }
    return true;
}

void Localizer::compute_estimate() {
    pf_t* pf = impl_->pf;
    pf_sample_set_t* set = pf->sets + pf->current_set;
    double best_weight = -1.0;
    pf_vector_t best = pf_vector_zero();
    for (int c = 0; c < set->cluster_count; ++c) {
        double weight;
        pf_vector_t mean;
        pf_matrix_t cov;
        if (!pf_get_cluster_stats(pf, c, &weight, &mean, &cov)) break;
        if (weight > best_weight) {
            best_weight = weight;
            best = mean;
        }
    }
    if (best_weight < 0.0) return;  // クラスタが無い (粒子が無い) ときは前の推定のまま
    estimate_ = from_pf(best);
    estimate_.theta = normalize_angle(estimate_.theta);
    // 共分散は ROS の amcl_pose と同じく、粒子全体のもの
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) covariance_[i * 3 + j] = set->cov.m[i][j];
}

Pose2D Localizer::pose() const {
    if (!have_any_odom_) return estimate_;
    return compose(map_to_odom_, last_odom_);
}

Covariance Localizer::covariance() const { return covariance_; }

std::vector<Particle> Localizer::particles(size_t max) const {
    std::vector<Particle> out;
    if (!impl_->pf) return out;
    const pf_sample_set_t* set = impl_->pf->sets + impl_->pf->current_set;
    const size_t n = static_cast<size_t>(set->sample_count);
    const size_t step = (max == 0 || n <= max) ? 1 : (n + max - 1) / max;
    out.reserve(n / step + 1);
    for (size_t i = 0; i < n; i += step) {
        const pf_sample_t& s = set->samples[i];
        out.push_back(Particle{from_pf(s.pose), s.weight});
    }
    return out;
}

int Localizer::particle_count() const {
    if (!impl_->pf) return 0;
    return impl_->pf->sets[impl_->pf->current_set].sample_count;
}

}  // namespace localization_amcl
