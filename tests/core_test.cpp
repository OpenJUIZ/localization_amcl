// src/core のテスト。合成した部屋の地図の中をロボットが走り、スキャンを地図から作り、オドメトリにはずれを足す。
//   core_test          全部
//   core_test --quick  時間のかかる大域的な推定を飛ばす (サニタイザ用)
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "localization_amcl/localizer.h"
#include "localization_amcl/map.h"
#include "localization_amcl/version.h"

using namespace localization_amcl;

namespace {

const double kPi = 3.14159265358979323846;
int failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("  FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                      \
        }                                                                    \
    } while (0)

// 10 m x 8 m の部屋。L 字の塊と柱を置いて、対称にならないようにする。原点 (-1, -1)
OccupancyGrid make_room() {
    OccupancyGrid g;
    g.resolution = 0.05;
    g.origin = Pose2D{-1.0, -1.0, 0.0};
    g.width = 200;
    g.height = 160;
    g.data.assign(static_cast<size_t>(g.width) * g.height, 0);
    for (int j = 0; j < g.height; ++j) {
        for (int i = 0; i < g.width; ++i) {
            const double x = g.origin.x + (i + 0.5) * g.resolution;
            const double y = g.origin.y + (j + 0.5) * g.resolution;
            const bool wall = i < 2 || j < 2 || i >= g.width - 2 || j >= g.height - 2;
            const bool block = (x > 5.0 && x < 6.0 && y > 1.0 && y < 4.0) || (x > 5.0 && x < 7.5 && y > 3.0 && y < 4.0);
            const bool pillar = std::hypot(x - 1.5, y - 5.0) < 0.3;
            if (wall || block || pillar) g.data[static_cast<size_t>(j) * g.width + i] = 100;
        }
    }
    return g;
}

bool occupied(const OccupancyGrid& g, double x, double y) {
    const int i = static_cast<int>(std::floor((x - g.origin.x) / g.resolution));
    const int j = static_cast<int>(std::floor((y - g.origin.y) / g.resolution));
    if (i < 0 || j < 0 || i >= g.width || j >= g.height) return true;
    return g.data[static_cast<size_t>(j) * g.width + i] == 100;
}

// LiDAR の位置 (map の座標) から、地図を光線でたどってスキャンを作る。当たらないビームは range_max + 1 (mobile_sim と同じ)
Scan simulate_scan(const OccupancyGrid& g, const Pose2D& lidar, std::mt19937& rng, double stamp) {
    Scan s;
    s.stamp = stamp;
    s.angle_min = -kPi * 0.75;
    s.angle_increment = kPi * 1.5 / 270;
    s.range_min = 0.1;
    s.range_max = 6.0;
    std::normal_distribution<double> noise(0.0, 0.01);
    for (int k = 0; k <= 270; ++k) {
        const double a = lidar.theta + s.angle_min + k * s.angle_increment;
        double r = s.range_max + 1.0;
        for (double d = s.range_min; d <= s.range_max; d += g.resolution * 0.25) {
            if (occupied(g, lidar.x + d * std::cos(a), lidar.y + d * std::sin(a))) {
                r = d + noise(rng);
                break;
            }
        }
        s.ranges.push_back(static_cast<float>(r));
    }
    return s;
}

double dist(const Pose2D& a, const Pose2D& b) { return std::hypot(a.x - b.x, a.y - b.y); }

struct RunResult {
    double final_error = 1e9;
    double worst_error = 0.0;  // 最初の skip 回の更新のあと
    double odom_error = 0.0;
    int updates = 0;
};

// 決まった経路を走る。truth は真の姿勢 (map)、odom はずれの入ったオドメトリ (odom の座標、原点は走り出した位置)
RunResult drive(Localizer& loc, const OccupancyGrid& g, Pose2D truth, const Pose2D& laser_mount, int skip,
                const std::function<void(int, const Pose2D&, const Pose2D&)>& on_update = nullptr) {
    std::mt19937 rng(11);
    std::normal_distribution<double> noise(0.0, 1.0);
    const std::vector<Pose2D> route = {{4.0, 0.5, 0}, {7.5, 1.0, 0}, {7.5, 5.5, 0}, {3.0, 5.8, 0},
                                       {0.3, 4.0, 0}, {0.5, 0.8, 0}, {3.0, 0.5, 0}};
    const Pose2D start = truth;
    Pose2D odom{0, 0, 0};
    RunResult res;
    double t = 0.0;
    for (const auto& wp : route) {
        for (int k = 0; k < 600; ++k) {
            const double dx = wp.x - truth.x, dy = wp.y - truth.y;
            if (std::hypot(dx, dy) < 0.05) break;
            const double turn = normalize_angle(std::atan2(dy, dx) - truth.theta);
            double v = 0.0, w = 0.0;
            if (std::fabs(turn) > 0.05) {
                w = std::copysign(std::min(0.1, std::fabs(turn)), turn);
            } else {
                v = std::min(0.04, std::hypot(dx, dy));
                w = 0.3 * turn;
            }
            truth.theta = normalize_angle(truth.theta + w);
            truth.x += v * std::cos(truth.theta);
            truth.y += v * std::sin(truth.theta);
            // オドメトリ: 速度に 5 % のずれと、進むほど左に曲がる癖
            const double vo = v * (1.0 + 0.05 * noise(rng));
            const double wo = w * (1.0 + 0.05 * noise(rng)) + 0.02 * v;
            odom.theta = normalize_angle(odom.theta + wo);
            odom.x += vo * std::cos(odom.theta);
            odom.y += vo * std::sin(odom.theta);
            t += 0.05;
            const Scan scan = simulate_scan(g, compose(truth, laser_mount), rng, t);
            if (loc.add_scan(scan, odom)) {
                ++res.updates;
                const double err = dist(loc.pose(), truth);
                if (res.updates > skip && err > res.worst_error) res.worst_error = err;
                if (on_update) on_update(res.updates, truth, odom);
            }
        }
    }
    res.final_error = dist(loc.pose(), truth);
    res.odom_error = dist(compose(start, odom), truth);
    return res;
}

void test_map_io() {
    std::printf("map_io\n");
    std::string image;
    const MapMeta meta = parse_map_yaml(
        "image: \"room.pgm\"  # comment\nresolution: 0.1\norigin: [-2.5, 1.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n",
        image);
    CHECK(image == "room.pgm");
    CHECK(std::fabs(meta.resolution - 0.1) < 1e-12);
    CHECK(std::fabs(meta.origin.x + 2.5) < 1e-12 && std::fabs(meta.origin.y - 1.0) < 1e-12);
    CHECK(!meta.negate);
    bool threw = false;
    try {
        parse_map_yaml("resolution: 0.1\n", image);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);

    // 4x3 の PGM。上の行の左端が黒 (占有)、右端が灰 (不明)、他は白 (空き)
    {
        std::ofstream f("core_test_map.pgm", std::ios::binary);
        f << "P5\n4 3\n255\n";
        const unsigned char px[12] = {0, 254, 254, 205, 254, 254, 254, 254, 254, 254, 254, 254};
        f.write(reinterpret_cast<const char*>(px), sizeof(px));
        std::ofstream y("core_test_map.yaml");
        y << "image: core_test_map.pgm\nresolution: 0.1\norigin: [-2.5, 1.0, 0.0]\nnegate: 0\n"
             "occupied_thresh: 0.65\nfree_thresh: 0.196\n";
    }
    const OccupancyGrid g = load_map("./core_test_map.yaml");
    CHECK(g.width == 4 && g.height == 3);
    // 画像の上の行 = 格子の最後の行
    CHECK(g.data[2 * 4 + 0] == 100);
    CHECK(g.data[2 * 4 + 3] == -1);
    CHECK(g.data[0] == 0);
    const std::vector<uint8_t> px = grid_to_pixels(g);
    CHECK(px[0] == 0 && px[3] == 205 && px[4] == 254);

    // 画像から (L8 と RGBA)
    const uint8_t rgba[8] = {0, 0, 0, 255, 255, 255, 255, 255};
    MapMeta m;
    const OccupancyGrid g2 = grid_from_image(rgba, 2, 1, 4, m);
    CHECK(g2.data[0] == 100 && g2.data[1] == 0);
    m.negate = true;
    const OccupancyGrid g3 = grid_from_image(rgba, 2, 1, 4, m);
    CHECK(g3.data[0] == 0 && g3.data[1] == 100);
}

void test_pose_math() {
    std::printf("pose math\n");
    const Pose2D a{1.0, 2.0, 0.5}, b{-0.3, 0.7, -1.2};
    const Pose2D c = compose(inverse(a), compose(a, b));
    CHECK(std::fabs(c.x - b.x) < 1e-9 && std::fabs(c.y - b.y) < 1e-9 && std::fabs(c.theta - b.theta) < 1e-9);
    CHECK(std::fabs(normalize_angle(3 * kPi) - kPi) < 1e-9 || std::fabs(normalize_angle(3 * kPi) + kPi) < 1e-9);
}

void test_tracking(const OccupancyGrid& room) {
    std::printf("tracking from an offset initial pose\n");
    Params p;
    p.seed = 3;
    const Pose2D start{1.0, 0.5, 0.0};
    Localizer loc(p);
    loc.set_map(room);
    loc.set_initial_pose(Pose2D{start.x + 0.3, start.y + 0.2, 0.15});
    bool consistent = true;
    const RunResult r = drive(loc, room, start, Pose2D{}, 20, [&](int n, const Pose2D& tr, const Pose2D& odom) {
        if (getenv("DEBUG")) std::printf("   %d truth %.2f %.2f %.2f est %.2f %.2f %.2f\n", n, tr.x, tr.y, tr.theta, loc.pose().x, loc.pose().y, loc.pose().theta);
        const Pose2D viaodom = compose(loc.map_to_odom(), odom);
        if (dist(viaodom, loc.pose()) > 1e-9) consistent = false;
    });
    std::printf("  updates %d, particles %d, final error %.3f m, worst %.3f m, odometry only %.3f m\n", r.updates,
                loc.particle_count(), r.final_error, r.worst_error, r.odom_error);
    CHECK(r.updates > 50);
    CHECK(r.final_error < 0.1);
    CHECK(r.worst_error < 0.15);
    CHECK(r.odom_error > r.final_error);
    CHECK(consistent);
    const Covariance cov = loc.covariance();
    CHECK(cov[0] > 0.0 && cov[0] < 0.05 && cov[4] > 0.0 && cov[4] < 0.05);
    CHECK(loc.particles(100).size() <= 100);
}

void test_laser_mount(const OccupancyGrid& room) {
    std::printf("tracking with a LiDAR mounted 0.2 m ahead, facing backwards\n");
    Params p;
    p.seed = 5;
    const Pose2D mount{0.2, 0.0, kPi};
    const Pose2D start{1.0, 0.5, 0.0};
    Localizer loc(p);
    loc.set_map(room);
    CHECK(loc.set_laser_pose(mount));
    CHECK(!loc.set_laser_pose(mount));
    loc.set_initial_pose(start);
    const RunResult r = drive(loc, room, start, mount, 20);
    std::printf("  updates %d, final error %.3f m, worst %.3f m\n", r.updates, r.final_error, r.worst_error);
    CHECK(r.final_error < 0.1);
    CHECK(r.worst_error < 0.15);
}

void test_global(const OccupancyGrid& room) {
    std::printf("global localization\n");
    Params p;
    p.seed = 7;
    p.min_particles = 500;
    p.max_particles = 20000;
    Localizer loc(p);
    loc.set_map(room);
    loc.global_localization();
    CHECK(loc.particle_count() == 20000);
    int converged_at = -1;
    const RunResult r = drive(loc, room, Pose2D{1.0, 0.5, 0.0}, Pose2D{}, 1000000, [&](int n, const Pose2D& truth, const Pose2D&) {
        const double err = dist(loc.pose(), truth);
        if (err < 0.2 && converged_at < 0) converged_at = n;
        if (err >= 0.5) converged_at = -1;  // 離れたら数え直す
    });
    std::printf("  converged after %d updates, final error %.3f m, particles %d\n", converged_at, r.final_error,
                loc.particle_count());
    CHECK(converged_at > 0);
    CHECK(r.final_error < 0.15);
    CHECK(loc.particle_count() < 5000);
}

void test_errors(const OccupancyGrid& room) {
    std::printf("errors\n");
    Params p;
    Localizer loc(p);
    bool threw = false;
    try {
        loc.add_scan(Scan{}, Pose2D{});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);
    threw = false;
    try {
        Params bad;
        bad.laser_model_type = "sonar";
        Localizer l2(bad);
        l2.set_map(room);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
    // 地図の前に与えた初期姿勢は、地図が来たときに使う
    Localizer l3(p);
    l3.set_initial_pose(Pose2D{2.0, 3.0, 1.0});
    l3.set_map(room);
    CHECK(dist(l3.pose(), Pose2D{2.0, 3.0, 1.0}) < 0.2);
    CHECK(l3.initialized());
    // 動いていないときは、粒子を撒いた直後の 1 回だけ更新する
    std::mt19937 rng(1);
    const Scan s = simulate_scan(room, Pose2D{2.0, 3.0, 1.0}, rng, 0.0);
    CHECK(l3.add_scan(s, Pose2D{}));
    CHECK(!l3.add_scan(s, Pose2D{}));
    CHECK(l3.add_scan(s, Pose2D{0.25, 0.0, 0.0}));
}

}  // namespace

int main(int argc, char** argv) {
    const bool quick = argc > 1 && std::strcmp(argv[1], "--quick") == 0;
    std::printf("localization_amcl %s\n", version().c_str());
    const OccupancyGrid room = make_room();
    test_map_io();
    test_pose_math();
    test_errors(room);
    test_tracking(room);
    test_laser_mount(room);
    if (!quick) test_global(room);
    std::printf(failures ? "%d FAILED\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
