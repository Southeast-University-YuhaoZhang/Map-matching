# C++ GPS Map Matching

基于候选路段与 Viterbi 的离线 GPS 轨迹绑路工具，使用网格索引加速候选搜索，并结合距离、航向、速度和轨迹连续性评分。

> 当前仓库提供离线命令行程序；原始代码中没有可直接复用的在线 HTTP 服务。

## 功能

- 从 WKT `LINESTRING` 路网 CSV 加载道路几何、单行道和限速信息
- 使用网格索引缩小 GPS 点的候选路段范围
- 结合距离、航向、速度约束进行候选过滤
- 对每个 `tripId` 使用 Viterbi 算法进行连续轨迹匹配
- 输出原始点、匹配路段 ID 和投影坐标

## 构建

需要 CMake 3.15+ 和支持 C++17 的编译器：

```bash
cmake -S . -B build
cmake --build build --config Release
```

Windows 下也可以使用 Visual Studio 打开 CMake 项目；Linux/macOS 使用 GCC 或 Clang 均可。

## 使用

```bash
map_match --network net_gcj02_new3.csv --trajectory trajectory.csv --output matched_result.csv
```

路网 CSV 至少包含前 3 列：`linkid,WKT,oneway`，可选第 4 列 `maxspeed`。WKT 使用 `LINESTRING (lon lat, ...)`。

轨迹 CSV 必须包含：`tripId,longitude,latitude`；可选：`time,speed,bearing,horizontalAccuracyMeters`。`speed` 按 m/s 读取并转换为 km/h。

坐标必须与路网使用同一坐标系（示例为 GCJ-02）。速度字段按 m/s 读取并转换为 km/h。

## 注意事项

- 路网和轨迹必须使用相同坐标系，否则匹配结果会明显偏移。
- `horizontalAccuracyMeters` 大于 30 的轨迹点会被过滤。
- 没有候选路段的点会保留在输出中，但 `matched_linkid` 为空。
- `rapidcsv.h` 已随项目提供，无需额外下载依赖。

## 项目结构

```text
.
├── CMakeLists.txt
├── main.cpp
├── rapidcsv.h
├── README.md
└── .gitignore
```

## 许可证

项目代码可按 MIT License 使用；`rapidcsv.h` 请同时遵守其文件头部声明的许可条款。
