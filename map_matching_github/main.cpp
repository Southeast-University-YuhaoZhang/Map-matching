#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <iomanip>          // setprecision
#include <cstdint>           // uint64_t, uint32_t
#include "rapidcsv.h"        // 仍用于路网加载，轨迹部分已替换
#include <map>

using namespace std;
using namespace std::chrono;

// ================== 数据结构 ==================
struct Point {
    double lon, lat;
};

struct Edge {
    string linkid;              // 完整保留，如 "1606243197_3388927620"
    vector<Point> coords;
    int oneway;
    double maxspeed;            // km/h
};

struct Candidate {
    int edge_idx;
    Point proj_point;
    double dist;                // 米（已从度换算）
    double bearing_diff;        // 度
};

struct GPSObservation {
    Point pt;
    double bearing;             // -1 表示无
    double speed_kmh;
    long long timestamp;
};

struct MatchingResult {
    int edge_idx;
    Point matched_point;
    string linkid;
};

// ================== 常量 / 超参数 ==================
const double DEG_TO_M   = 111000.0;
const double SIGMA_DIST = 20.0;
const double SIGMA_BEARING = 30.0;
const double BETA = 50.0;
const double SPEED_THRESHOLD_FACTOR = 1.5;
const double BEARING_THRESHOLD_SPEED = 5.0;
const double BEARING_THRESHOLD_ANGLE = 60.0;
const double PI = 3.14159265358979323846;

// ================== 工具函数 ==================
double stod_safe(const string& s) {
    try { return stod(s); } catch (...) { return numeric_limits<double>::quiet_NaN(); }
}

vector<Point> parse_linestring(const string& wkt) {
    vector<Point> pts;
    size_t start = wkt.find('(');
    if (start == string::npos) return pts;
    size_t end = wkt.rfind(')');
    if (end == string::npos || end <= start) return pts;
    string content = wkt.substr(start+1, end - start -1);
    stringstream ss(content);
    string token;
    while (getline(ss, token, ',')) {
        stringstream pt_stream(token);
        double lon, lat;
        if (pt_stream >> lon >> lat) {
            pts.push_back({lon, lat});
        }
    }
    return pts;
}

double parse_maxspeed(const string& speed_str) {
    if (speed_str.empty()) return 50.0;
    try {
        return stod(speed_str);
    } catch (...) {
        size_t pos = speed_str.find("mph");
        if (pos != string::npos) {
            string num_part = speed_str.substr(0, pos);
            try { return stod(num_part) * 1.609; } catch (...) { return 50.0; }
        }
        return 50.0;
    }
}

// ================== 网格索引（加速候选搜索，包含边界安全扩展） ==================
struct GridIndex {
    double step_lon, step_lat;   // 网格步长，例如 0.005
    // 修复 UB：使用 uint64_t 键，避免有符号左移未定义行为
    unordered_map<uint64_t, vector<int>> cells;

    // 安全的键生成函数（无符号位操作）
    static inline uint64_t make_key(int x, int y) {
        uint32_t ux = static_cast<uint32_t>(x);
        uint32_t uy = static_cast<uint32_t>(y);
        return (static_cast<uint64_t>(ux) << 32) | uy;
    }

    void build(const vector<Edge>& edges, double step = 0.005) {
        step_lon = step;
        step_lat = step;
        for (size_t i = 0; i < edges.size(); ++i) {
            const auto& coords = edges[i].coords;
            if (coords.empty()) continue;
            double min_lon = coords[0].lon, max_lon = min_lon;
            double min_lat = coords[0].lat, max_lat = min_lat;
            for (const auto& p : coords) {
                if (p.lon < min_lon) min_lon = p.lon;
                if (p.lon > max_lon) max_lon = p.lon;
                if (p.lat < min_lat) min_lat = p.lat;
                if (p.lat > max_lat) max_lat = p.lat;
            }
            int i0 = static_cast<int>(floor(min_lon / step_lon));
            int i1 = static_cast<int>(floor(max_lon / step_lon));
            int j0 = static_cast<int>(floor(min_lat / step_lat));
            int j1 = static_cast<int>(floor(max_lat / step_lat));
            for (int x = i0; x <= i1; ++x)
                for (int y = j0; y <= j1; ++y) {
                    auto key = make_key(x, y);   // 修复后的安全写法
                    cells[key].push_back(static_cast<int>(i));
                }
        }
    }

    // 查询时向外扩展一个步长，确保不遗漏边界边
    vector<int> query(double lon, double lat, double radius_deg) const {
        vector<int> res;
        double min_lon = lon - radius_deg - step_lon;
        double max_lon = lon + radius_deg + step_lon;
        double min_lat = lat - radius_deg - step_lat;
        double max_lat = lat + radius_deg + step_lat;
        int i0 = static_cast<int>(floor(min_lon / step_lon));
        int i1 = static_cast<int>(floor(max_lon / step_lon));
        int j0 = static_cast<int>(floor(min_lat / step_lat));
        int j1 = static_cast<int>(floor(max_lat / step_lat));
        for (int x = i0; x <= i1; ++x)
            for (int y = j0; y <= j1; ++y) {
                auto key = make_key(x, y);       // 修复后的安全写法
                auto it = cells.find(key);
                if (it != cells.end())
                    res.insert(res.end(), it->second.begin(), it->second.end());
            }
        sort(res.begin(), res.end());
        res.erase(unique(res.begin(), res.end()), res.end());
        return res;
    }
};

// ================== 度距离（与 Python 一致） ==================
double degree_distance(const Point& a, const Point& b) {
    double dx = a.lon - b.lon;
    double dy = a.lat - b.lat;
    return sqrt(dx*dx + dy*dy);
}

pair<Point, double> point_to_segment_deg(const Point& p, const Point& seg1, const Point& seg2) {
    double dx = seg2.lon - seg1.lon;
    double dy = seg2.lat - seg1.lat;
    double len2 = dx*dx + dy*dy;
    if (len2 == 0.0) {
        return {seg1, degree_distance(p, seg1)};
    }
    double t = ((p.lon - seg1.lon)*dx + (p.lat - seg1.lat)*dy) / len2;
    t = max(0.0, min(1.0, t));
    double proj_lon = seg1.lon + t*dx;
    double proj_lat = seg1.lat + t*dy;
    Point proj{proj_lon, proj_lat};
    double dist = degree_distance(p, proj);
    return {proj, dist};
}

pair<Point, double> point_to_line_projection_deg(const Point& p, const vector<Point>& coords) {
    if (coords.size() < 2) {
        return {coords[0], degree_distance(p, coords[0])};
    }
    double min_dist = numeric_limits<double>::max();
    Point best_proj;
    for (size_t i = 0; i < coords.size()-1; ++i) {
        auto [proj, d] = point_to_segment_deg(p, coords[i], coords[i+1]);
        if (d < min_dist) {
            min_dist = d;
            best_proj = proj;
        }
    }
    return {best_proj, min_dist};
}

double segment_bearing_at_projection(const vector<Point>& coords, const Point& proj_point) {
    if (coords.size() < 2) return 0.0;
    size_t best_seg = 0;
    double min_dist = numeric_limits<double>::max();
    for (size_t i = 0; i < coords.size()-1; ++i) {
        auto [p, d] = point_to_segment_deg(proj_point, coords[i], coords[i+1]);
        if (d < min_dist) {
            min_dist = d;
            best_seg = i;
        }
    }
    const Point& p1 = coords[best_seg];
    const Point& p2 = coords[best_seg+1];
    double dx = p2.lon - p1.lon;
    double dy = p2.lat - p1.lat;
    double angle_rad = atan2(dx, dy);
    double angle_deg = angle_rad * 180.0 / PI;
    if (angle_deg < 0) angle_deg += 360.0;
    return angle_deg;
}

double bearing_difference(double gps_bearing, double road_bearing) {
    double diff = fabs(gps_bearing - road_bearing);
    if (diff > 180.0) diff = 360.0 - diff;
    return diff;
}

// ================== 路网加载 ==================
vector<Edge> load_network(const string& csv_path, GridIndex& grid) {
    try {
        rapidcsv::Document doc(csv_path, rapidcsv::LabelParams(0, 0));
        size_t rows = doc.GetRowCount();
        cout << "路网文件共 " << rows << " 行数据" << endl;
        vector<Edge> edges;
        for (size_t i = 0; i < rows; ++i) {
            try {
                string linkid_str = doc.GetCell<string>(0, i);
                string wkt_str    = doc.GetCell<string>(1, i);
                string oneway_str = doc.GetCell<string>(2, i);
                string maxspeed_str = (doc.GetColumnCount() > 3) ? doc.GetCell<string>(3, i) : "";
                vector<Point> coords = parse_linestring(wkt_str);
                if (coords.empty()) continue;
                int oneway = 0;
                if (oneway_str == "1" || oneway_str == "yes" || oneway_str == "true") oneway = 1;
                else if (oneway_str == "-1") oneway = -1;
                double maxspeed = parse_maxspeed(maxspeed_str);
                edges.push_back({linkid_str, coords, oneway, maxspeed});
            } catch (const exception& e) {
                cerr << "警告：解析第 " << i << " 行失败: " << e.what() << endl;
            }
        }
        grid.build(edges, 0.005);
        return edges;
    } catch (const exception& e) {
        cerr << "严重错误：无法读取路网文件 '" << csv_path << "': " << e.what() << endl;
        return {};
    }
}

// ================== 候选生成（网格索引 + 候选限制） ==================
const size_t MAX_CAND = 5;

vector<Candidate> find_candidates(const Point& pt, double gps_bearing, double speed_kmh,
                                   const vector<Edge>& edges, double radius_m,
                                   const GridIndex& grid) {
    vector<Candidate> candidates;
    double radius_deg = radius_m / 111000.0;
    vector<int> nearby = grid.query(pt.lon, pt.lat, radius_deg);

    for (int idx : nearby) {
        const Edge& e = edges[idx];
        auto [proj, deg_dist] = point_to_line_projection_deg(pt, e.coords);
        double dist_m = deg_dist * 111000.0;
        if (dist_m > radius_m) continue;

        if (!isnan(speed_kmh) && speed_kmh > e.maxspeed * SPEED_THRESHOLD_FACTOR) continue;

        double bearing_diff = 0.0;
        if (!isnan(gps_bearing) && !isnan(speed_kmh) && speed_kmh > BEARING_THRESHOLD_SPEED) {
            double road_bearing = segment_bearing_at_projection(e.coords, proj);
            bearing_diff = bearing_difference(gps_bearing, road_bearing);
            if (bearing_diff > BEARING_THRESHOLD_ANGLE) continue;
        }
        candidates.push_back({idx, proj, dist_m, bearing_diff});
    }

    if (candidates.size() > MAX_CAND) {
        partial_sort(candidates.begin(), candidates.begin() + MAX_CAND, candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.dist < b.dist; });
        candidates.resize(MAX_CAND);
    }
    return candidates;
}

// ================== 维特比匹配 ==================
vector<MatchingResult> viterbi_matching_with_time(const vector<GPSObservation>& obs,
                                                  const vector<Edge>& edges,
                                                  const GridIndex& grid,
                                                  long long& elapsed_us) {
    auto t_start = high_resolution_clock::now();
    size_t n = obs.size();
    if (n == 0) return {};

    vector<vector<Candidate>> cands_by_obs(n);
    for (size_t i = 0; i < n; ++i) {
        double radius_m = max(50.0, (isnan(obs[i].speed_kmh) ? 0.0 : obs[i].speed_kmh * 0.5));
        cands_by_obs[i] = find_candidates(obs[i].pt, obs[i].bearing, obs[i].speed_kmh, edges, radius_m, grid);
        if (cands_by_obs[i].empty()) {
            cands_by_obs[i].push_back({-1, obs[i].pt, 999.0, 0.0});
        }
    }

    vector<vector<pair<double, int>>> V(n);
    for (size_t i = 0; i < n; ++i) {
        V[i].resize(cands_by_obs[i].size());
        for (size_t k = 0; k < cands_by_obs[i].size(); ++k) {
            const Candidate& cand = cands_by_obs[i][k];
            double log_obs;
            if (cand.edge_idx == -1) {
                log_obs = -50.0;
            } else {
                double dist_prob = -0.5 * pow(cand.dist / SIGMA_DIST, 2);
                double bearing_penalty = 0.0;
                if (obs[i].bearing >= 0) {
                    bearing_penalty = -0.5 * pow(cand.bearing_diff / SIGMA_BEARING, 2);
                }
                log_obs = dist_prob + bearing_penalty;
            }

            if (i == 0) {
                V[i][k] = {log_obs, -1};
            } else {
                double max_log = -numeric_limits<double>::infinity();
                int best_prev = -1;
                for (size_t j = 0; j < cands_by_obs[i-1].size(); ++j) {
                    const Candidate& prev = cands_by_obs[i-1][j];
                    double trans_log;
                    if (prev.edge_idx == -1 || cand.edge_idx == -1) {
                        trans_log = -10.0;
                    } else {
                        double gps_dist_deg = degree_distance(obs[i-1].pt, obs[i].pt);
                        double route_dist_deg = degree_distance(prev.proj_point, cand.proj_point);
                        double gps_dist_m = gps_dist_deg * 111000.0;
                        double route_dist_m = route_dist_deg * 111000.0;
                        double diff = fabs(route_dist_m - max(gps_dist_m, 0.1));
                        trans_log = -diff / BETA;
                    }
                    double total = V[i-1][j].first + log_obs + trans_log;
                    if (total > max_log) {
                        max_log = total;
                        best_prev = static_cast<int>(j);
                    }
                }
                V[i][k] = {max_log, best_prev};
            }
        }
    }

    // 回溯
    size_t last_best = 0;
    {
        double max_val = -numeric_limits<double>::infinity();
        for (size_t k = 0; k < V[n-1].size(); ++k) {
            if (V[n-1][k].first > max_val) {
                max_val = V[n-1][k].first;
                last_best = k;
            }
        }
    }
    vector<size_t> best_path(n);
    best_path[n-1] = last_best;
    for (int i = static_cast<int>(n)-2; i >= 0; --i) {
        best_path[i] = V[i+1][best_path[i+1]].second;
    }

    vector<MatchingResult> results;
    for (size_t i = 0; i < n; ++i) {
        const Candidate& cand = cands_by_obs[i][best_path[i]];
        MatchingResult res;
        res.edge_idx = cand.edge_idx;
        res.matched_point = cand.proj_point;
        res.linkid = (cand.edge_idx >= 0) ? edges[cand.edge_idx].linkid : "";
        results.push_back(res);
    }

    auto t_end = high_resolution_clock::now();
    elapsed_us = duration_cast<microseconds>(t_end - t_start).count();
    return results;
}

// ================== 主程序 ==================
int main(int argc, char* argv[]) {
    string network_path = "net_gcj02_new3.csv";
    string trajectory_path = "trajectory.csv";
    string output_path = "matched_result.csv";
    if (argc == 1) {
        cout << "用法: map_match --network <路网CSV> --trajectory <轨迹CSV> [--output <结果CSV>]\n";
        cout << "默认文件: " << network_path << "，" << trajectory_path << "，输出 " << output_path << "\n";
    }
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        auto next = [&](string& value) -> bool {
            if (i + 1 >= argc) { cerr << "缺少参数值: " << arg << endl; return false; }
            value = argv[++i]; return true;
        };
        if (arg == "--network" || arg == "-n") { if (!next(network_path)) return 2; }
        else if (arg == "--trajectory" || arg == "-t") { if (!next(trajectory_path)) return 2; }
        else if (arg == "--output" || arg == "-o") { if (!next(output_path)) return 2; }
        else if (arg == "--help" || arg == "-h") { return 0; }
        else { cerr << "未知参数: " << arg << endl; return 2; }
    }

    // 1. 加载路网
    cout << "加载路网..." << endl;
    auto start_load = high_resolution_clock::now();
    GridIndex grid;
    vector<Edge> edges = load_network(network_path, grid);
    if (edges.empty()) return 1;
    auto end_load = high_resolution_clock::now();
    cout << "路网加载完成，边数: " << edges.size()
         << "，耗时: " << duration_cast<milliseconds>(end_load - start_load).count() << " ms" << endl;

    // 2. 读取轨迹（替换为安全自定义解析，不再使用 rapidcsv）
    cout << "读取轨迹（安全解析）..." << endl;
    multimap<string, GPSObservation> trips_map;

    // 定义安全转换辅助函数
    auto safe_stod = [](const string& s) -> double {
        if (s.empty()) return numeric_limits<double>::quiet_NaN();
        try { return stod(s); } catch (...) { return numeric_limits<double>::quiet_NaN(); }
    };
    auto safe_stoll = [](const string& s) -> long long {
        if (s.empty()) return -1;
        try { return stoll(s); } catch (...) { return -1; }
    };

    // 自定义 CSV 行分割（处理引号）
    auto trim = [](const string& str) {
        size_t first = str.find_first_not_of(" \t\r\n");
        if (first == string::npos) return string();
        size_t last = str.find_last_not_of(" \t\r\n");
        return str.substr(first, last - first + 1);
    };
    auto split_csv = [&](const string& line, char delim = ',') {
        vector<string> fields;
        string field;
        bool in_quotes = false;
        for (char c : line) {
            if (c == '"') in_quotes = !in_quotes;
            else if (c == delim && !in_quotes) {
                fields.push_back(trim(field));
                field.clear();
            } else {
                field += c;
            }
        }
        fields.push_back(trim(field));
        return fields;
    };

    ifstream traj_file(trajectory_path);
    if (!traj_file.is_open()) {
        cerr << "无法打开轨迹文件！" << endl;
        return -1;
    }

    // 读取标题行
    string header_line;
    if (!getline(traj_file, header_line)) {
        cerr << "轨迹文件为空！" << endl;
        return -1;
    }
    // 处理 UTF-8 BOM
    if (header_line.size() >= 3 &&
        (unsigned char)header_line[0] == 0xEF &&
        (unsigned char)header_line[1] == 0xBB &&
        (unsigned char)header_line[2] == 0xBF) {
        header_line = header_line.substr(3);
    }

    auto headers = split_csv(header_line);
    unordered_map<string, int> col_map;
    for (size_t i = 0; i < headers.size(); ++i) col_map[headers[i]] = static_cast<int>(i);

    // 检查必要列
    if (col_map.find("tripId") == col_map.end() ||
        col_map.find("longitude") == col_map.end() ||
        col_map.find("latitude") == col_map.end()) {
        cerr << "轨迹文件缺少必要列 (tripId, longitude, latitude)!" << endl;
        return -1;
    }

    int idx_trip    = col_map["tripId"];
    int idx_lon     = col_map["longitude"];
    int idx_lat     = col_map["latitude"];
    int idx_time    = col_map.count("time") ? col_map["time"] : -1;
    int idx_speed   = col_map.count("speed") ? col_map["speed"] : -1;
    int idx_bearing = col_map.count("bearing") ? col_map["bearing"] : -1;
    int idx_accuracy= col_map.count("horizontalAccuracyMeters") ? col_map["horizontalAccuracyMeters"] : -1;

    string line;
    int line_num = 1;
    while (getline(traj_file, line)) {
        ++line_num;
        if (line.empty()) continue;

        auto fields = split_csv(line);
        // 验证字段数是否足够
        if (idx_trip >= (int)fields.size() || idx_lon >= (int)fields.size() || idx_lat >= (int)fields.size()) {
            cerr << "警告：第 " << line_num << " 行字段数不足，跳过。" << endl;
            continue;
        }

        string trip_id = fields[idx_trip];
        double lon = safe_stod(fields[idx_lon]);
        double lat = safe_stod(fields[idx_lat]);
        if (isnan(lon) || isnan(lat)) {
            cerr << "警告：第 " << line_num << " 行经纬度无效，跳过。" << endl;
            continue;
        }

        double accuracy = (idx_accuracy >= 0 && idx_accuracy < (int)fields.size())
                          ? safe_stod(fields[idx_accuracy]) : 999.0;
        if (!isnan(accuracy) && accuracy > 30.0) continue;   // 精度太差则过滤

        double speed_ms = (idx_speed >= 0 && idx_speed < (int)fields.size())
                          ? safe_stod(fields[idx_speed]) : 0.0;
        double speed_kmh = isnan(speed_ms) ? 0.0 : speed_ms * 3.6;
        if (!isnan(speed_kmh) && speed_kmh >= 150.0) continue;  // 异常速度过滤

        double bearing = (idx_bearing >= 0 && idx_bearing < (int)fields.size())
                         ? safe_stod(fields[idx_bearing]) : -1.0;
        long long timestamp = (idx_time >= 0 && idx_time < (int)fields.size())
                              ? safe_stoll(fields[idx_time]) : -1;

        GPSObservation obs;
        obs.pt = {lon, lat};
        obs.bearing = (isnan(bearing)) ? -1.0 : bearing;
        obs.speed_kmh = speed_kmh;
        obs.timestamp = timestamp;

        trips_map.insert({trip_id, obs});
    }

    cout << "有效轨迹点数: " << trips_map.size() << endl;

    // 3. 按 tripId 分组匹配，输出高精度结果
    ofstream outfile(output_path);
    if (!outfile) { cerr << "无法创建输出文件: " << output_path << endl; return 1; }
    outfile << "tripId,timestamp,longitude_original,latitude_original,matched_linkid,matched_lon,matched_lat" << endl;
    outfile << fixed << setprecision(12);    // 所有浮点数保留12位小数，避免精度丢失

    string current_trip = "";
    vector<GPSObservation> trip_points;
    vector<long long> trip_timestamps;
    long long total_us = 0;

    for (auto it = trips_map.begin(); it != trips_map.end(); ++it) {
        string trip = it->first;
        if (trip != current_trip) {
            if (!trip_points.empty()) {
                sort(trip_points.begin(), trip_points.end(), [](const GPSObservation& a, const GPSObservation& b) { return a.timestamp < b.timestamp; });
                cout << "匹配 tripId = " << current_trip << "，点数: " << trip_points.size() << endl;
                long long trip_us = 0;
                vector<MatchingResult> matched = viterbi_matching_with_time(trip_points, edges, grid, trip_us);
                total_us += trip_us;
                cout << "  >>> 匹配耗时: " << trip_us << " 微秒 ("
                     << setprecision(2) << (trip_us / 1000.0) << " 毫秒)" << endl;

                for (size_t i = 0; i < trip_points.size(); ++i) {
                    outfile << current_trip << ","
                            << trip_points[i].timestamp << ","
                            << trip_points[i].pt.lon << ","
                            << trip_points[i].pt.lat << ","
                            << matched[i].linkid << ","
                            << matched[i].matched_point.lon << ","
                            << matched[i].matched_point.lat << "\n";
                }
                trip_points.clear();
                trip_timestamps.clear();
            }
            current_trip = trip;
        }
        trip_points.push_back(it->second);
        trip_timestamps.push_back(it->second.timestamp);
    }

    // 最后一组
    if (!trip_points.empty()) {
        sort(trip_points.begin(), trip_points.end(), [](const GPSObservation& a, const GPSObservation& b) { return a.timestamp < b.timestamp; });
        cout << "匹配 tripId = " << current_trip << "，点数: " << trip_points.size() << endl;
        long long trip_us = 0;
        vector<MatchingResult> matched = viterbi_matching_with_time(trip_points, edges, grid, trip_us);
        total_us += trip_us;
        cout << "  >>> 匹配耗时: " << trip_us << " 微秒 ("
             << setprecision(2) << (trip_us / 1000.0) << " 毫秒)" << endl;
        for (size_t i = 0; i < trip_points.size(); ++i) {
            outfile << current_trip << ","
                    << trip_points[i].timestamp << ","
                    << trip_points[i].pt.lon << ","
                    << trip_points[i].pt.lat << ","
                    << matched[i].linkid << ","
                    << matched[i].matched_point.lon << ","
                    << matched[i].matched_point.lat << "\n";
        }
    }
    outfile.close();

    cout << "\n全部匹配完成！" << endl;
    cout << "所有 trip 匹配总耗时: " << total_us << " 微秒 ("
         << setprecision(2) << (total_us / 1000.0) << " 毫秒)" << endl;
    cout << "结果已保存至 matched_result.csv" << endl;

    return 0;
}
