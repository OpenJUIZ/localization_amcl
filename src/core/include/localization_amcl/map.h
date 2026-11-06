// 占有格子の地図と、ROS map_server 形式 (YAML + 画像) の読み込み。juiz には依存しない
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace localization_amcl {

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// ROS の nav_msgs/OccupancyGrid と同じ並び。data[x + y * width]、y = 0 が地図の下 (origin の側)。
// 値は -1 = 不明、0 = 空き、100 = 占有 (map_server の scale モードのときは 1..99 もある)
struct OccupancyGrid {
    double resolution = 0.05;  // [m/セル]
    Pose2D origin;             // 左下のセルの角の位置 (theta は map_server と同じく無視する)
    int width = 0;
    int height = 0;
    std::vector<int8_t> data;

    bool empty() const { return width <= 0 || height <= 0; }
};

// map_server の YAML の項目 (画像の解釈のしかた)
struct MapMeta {
    double resolution = 0.05;
    Pose2D origin;
    bool negate = false;
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    std::string mode = "trinary";  // trinary / scale / raw
};

// 画像 (左上が先頭、1 画素 channels バイト: 1 = L、2 = LA、3 = RGB、4 = RGBA) を map_server と同じ規則で格子にする
OccupancyGrid grid_from_image(const uint8_t* pixels, int width, int height, int channels, const MapMeta& meta);

// map_server の YAML を読む。画像 (PNG・PGM・BMP など) の相対パスは YAML のディレクトリから。失敗したら std::runtime_error
OccupancyGrid load_map(const std::string& yaml_path);

// YAML の中身から MapMeta と画像のパスを取り出す (load_map の中身。テスト用に出している)
MapMeta parse_map_yaml(const std::string& text, std::string& image_path);

// 画面用の 8 bit グレースケール (左上が先頭)。占有 0、空き 254、不明 205 (map_saver と同じ)
std::vector<uint8_t> grid_to_pixels(const OccupancyGrid& grid);

}  // namespace localization_amcl
