#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "localization_amcl/map.h"
#include "stb_image.h"

namespace localization_amcl {

namespace {

std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::string unquote(const std::string& s) {
    if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front()) return s.substr(1, s.size() - 2);
    return s;
}

// 行の # から後ろを消す (引用符の中の # は残す)
std::string strip_comment(const std::string& line) {
    char quote = 0;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quote) {
            if (c == quote) quote = 0;
        } else if (c == '"' || c == '\'') {
            quote = c;
        } else if (c == '#') {
            return line.substr(0, i);
        }
    }
    return line;
}

double to_number(const std::string& key, const std::string& value) {
    try {
        size_t used = 0;
        const double d = std::stod(value, &used);
        if (trim(value.substr(used)).empty()) return d;
    } catch (...) {
    }
    throw std::runtime_error("map yaml: " + key + " must be a number: " + value);
}

bool to_bool(const std::string& value) {
    std::string v = value;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v == "1" || v == "true" || v == "yes" || v == "on";
}

std::string dirname_of(const std::string& path) {
    const size_t p = path.find_last_of("/\\");
    return p == std::string::npos ? std::string() : path.substr(0, p + 1);
}

bool is_absolute(const std::string& path) {
    if (path.empty()) return false;
    if (path[0] == '/' || path[0] == '\\') return true;
    return path.size() > 1 && path[1] == ':';  // Windows のドライブ名
}

}  // namespace

MapMeta parse_map_yaml(const std::string& text, std::string& image_path) {
    MapMeta meta;
    bool have_resolution = false, have_image = false;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        line = trim(strip_comment(line));
        if (line.empty()) continue;
        const size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        const std::string key = trim(line.substr(0, colon));
        const std::string value = unquote(trim(line.substr(colon + 1)));
        if (key == "image") {
            image_path = value;
            have_image = true;
        } else if (key == "resolution") {
            meta.resolution = to_number(key, value);
            have_resolution = true;
        } else if (key == "origin") {
            std::string v = value;
            v.erase(std::remove_if(v.begin(), v.end(), [](char c) { return c == '[' || c == ']'; }), v.end());
            std::vector<double> nums;
            std::istringstream parts(v);
            std::string part;
            while (std::getline(parts, part, ',')) nums.push_back(to_number(key, trim(part)));
            if (nums.size() < 2) throw std::runtime_error("map yaml: origin must be [x, y, theta]");
            meta.origin = Pose2D{nums[0], nums[1], nums.size() > 2 ? nums[2] : 0.0};
        } else if (key == "negate") {
            meta.negate = to_bool(value);
        } else if (key == "occupied_thresh") {
            meta.occupied_thresh = to_number(key, value);
        } else if (key == "free_thresh") {
            meta.free_thresh = to_number(key, value);
        } else if (key == "mode") {
            meta.mode = value;
        }
    }
    if (!have_image) throw std::runtime_error("map yaml: image is missing");
    if (!have_resolution || !(meta.resolution > 0.0)) throw std::runtime_error("map yaml: resolution is missing");
    return meta;
}

OccupancyGrid grid_from_image(const uint8_t* pixels, int width, int height, int channels, const MapMeta& meta) {
    if (!pixels || width <= 0 || height <= 0) throw std::runtime_error("map image is empty");
    if (channels < 1 || channels > 4) throw std::runtime_error("map image: unsupported channel count");
    if (!(meta.resolution > 0.0)) throw std::runtime_error("map resolution must be positive");
    const bool raw = meta.mode == "raw";
    const bool trinary = meta.mode != "scale" && !raw;
    const bool has_alpha = channels == 2 || channels == 4;
    // 色の平均に使うチャンネル数 (raw でなければアルファを除く。map_server と同じ)
    const int avg_channels = (has_alpha && !raw) ? channels - 1 : channels;

    OccupancyGrid g;
    g.resolution = meta.resolution;
    g.origin = meta.origin;
    g.width = width;
    g.height = height;
    g.data.assign(static_cast<size_t>(width) * height, -1);
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            const uint8_t* p = pixels + (static_cast<size_t>(j) * width + i) * channels;
            int sum = 0;
            for (int k = 0; k < avg_channels; ++k) sum += p[k];
            double color_avg = static_cast<double>(sum) / avg_channels;
            const double alpha = has_alpha ? p[channels - 1] / 255.0 : 1.0;
            if (meta.negate) color_avg = 255.0 - color_avg;
            int8_t value;
            if (raw) {
                value = static_cast<int8_t>(color_avg > 127 ? -1 : static_cast<int>(color_avg));
            } else {
                const double occ = (255.0 - color_avg) / 255.0;
                if (occ > meta.occupied_thresh) {
                    value = 100;
                } else if (occ < meta.free_thresh) {
                    value = 0;
                } else if (trinary || alpha < 1.0) {
                    value = -1;
                } else {
                    const double ratio = (occ - meta.free_thresh) / (meta.occupied_thresh - meta.free_thresh);
                    value = static_cast<int8_t>(1 + 98 * ratio);
                }
            }
            // 画像は上の行が先、格子は下の行が先
            g.data[static_cast<size_t>(height - j - 1) * width + i] = value;
        }
    }
    return g;
}

OccupancyGrid load_map(const std::string& yaml_path) {
    std::ifstream f(yaml_path);
    if (!f) throw std::runtime_error("cannot open map yaml: " + yaml_path);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string image;
    const MapMeta meta = parse_map_yaml(ss.str(), image);
    const std::string image_path = is_absolute(image) ? image : dirname_of(yaml_path) + image;
    int w = 0, h = 0, n = 0;
    std::unique_ptr<stbi_uc, void (*)(void*)> pixels(stbi_load(image_path.c_str(), &w, &h, &n, 0), stbi_image_free);
    if (!pixels) {
        const char* why = stbi_failure_reason();
        throw std::runtime_error("cannot read map image " + image_path + (why ? std::string(": ") + why : std::string()));
    }
    return grid_from_image(pixels.get(), w, h, n, meta);
}

std::vector<uint8_t> grid_to_pixels(const OccupancyGrid& grid) {
    std::vector<uint8_t> out(static_cast<size_t>(grid.width) * grid.height, 205);
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const int8_t v = grid.data[static_cast<size_t>(y) * grid.width + x];
            uint8_t px = 205;
            if (v == 0) px = 254;
            else if (v >= 65) px = 0;
            else if (v > 0) px = 205;
            out[static_cast<size_t>(grid.height - y - 1) * grid.width + x] = px;
        }
    }
    return out;
}

}  // namespace localization_amcl
