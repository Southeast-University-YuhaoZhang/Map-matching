<img width="1256" height="722" alt="4e0fbc0c-362d-4883-899f-35072c754afa" src="https://github.com/user-attachments/assets/9e719411-4b4a-4779-856f-3f9aefba00a7" />Map-matching/

├── fmm/

├── st-matching/

├── graphhopper-map-matching/

└── offline-map-matching/

目前找了四个开源的项目，主要是HMM和STM算法

## 1) FMM（Fast Map Matching）

### 1.1 算法原理概述
FMM 将 **HMM（隐马尔可夫模型）** 与 **UBODT 预计算**结合：
- 观测概率：GPS 点到候选道路的几何误差；
- 转移概率：相邻候选之间路网路径一致性；
- 解码方式：Viterbi 最优序列；
- 速度优势：UBODT 减少在线最短路计算。

### 1.2 输入接口（CLI 参数）
> 命令示例：`fmm --ubodt ubodt.txt --network network.shp --gps traj.csv --output out.csv`

| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `--ubodt` | UBODT 预计算表文件（必填） | 路径 | `data/ubodt.txt` |
| `--network` | 路网文件（必填） | 路径 | `data/network.shp` |
| `--gps` | GPS 输入文件（必填） | 路径 | `data/traj.csv` |
| `--output` | 输出文件（必填） | 路径 | `result/match.csv` |
| `--network_id` | 路网边 ID 字段名 | 无 | `id` |
| `--source` | 路网起点字段名 | 无 | `source` |
| `--target` | 路网终点字段名 | 无 | `target` |
| `--gps_id` | 轨迹 ID 字段名 | 无 | `id` |
| `--gps_geom` | GPS 几何字段名 | 无 | `geom` |
| `--candidates` | 每个 GPS 点候选道路数量 | 个 | `8` |
| `--radius` | 候选搜索半径 | 米（m） | `300` |
| `--error` | GPS 噪声参数 | 米（m） | `50` |
| `--pf` | 路径惩罚因子 | 无 | `0` |
| `--output_fields` | 输出字段集合 | 无 | `opath,cpath,tpath,error` |

### 1.3 输入数据字段（常见）
#### A. 路网（Shapefile/CSV 对应字段）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `id` | 路段唯一标识 | 无 | `102938` |
| `source` | 路段起点节点 ID | 无 | `501` |
| `target` | 路段终点节点 ID | 无 | `502` |
| `geom` | 路段几何 | WKT/几何对象 | `LINESTRING(...)` |

#### B. GPS 轨迹（CSV 常见字段）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `id` | 轨迹/点所属对象 ID | 无 | `traj_0001` |
| `geom` | 点几何 | WKT/几何对象 | `POINT(121.4737 31.2304)` |
| `timestamp` | 采样时间（可选） | ISO8601 | `2026-05-09T08:00:00Z` |

### 1.4 输出接口（`--output_fields`）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `opath` | 原始点序列在路段上的投影路径 | 路段 ID 列表 | `12,18,21` |
| `cpath` | 候选状态路径 | 候选索引列表 | `0,2,1` |
| `tpath` | 连通路径（补全路网段） | 路段 ID 列表 | `12,15,18,19,21` |
| `ogeom` | 原始轨迹几何 | WKT | `LINESTRING(...)` |
| `mgeom` | 匹配后轨迹几何 | WKT | `LINESTRING(...)` |
| `pgeom` | 投影点几何 | WKT | `MULTIPOINT(...)` |
| `offset` | 点在边上的偏移量 | 米（m）/比例（实现相关） | `34.2` |
| `error` | 点到匹配道路误差 | 米（m） | `7.8` |

---

## 2) ST-Matching（`st-matching/` Python 版本）

### 2.1 算法原理概述
ST-Matching 结合：
- **Spatial**：点到道路的几何邻近；
- **Temporal**：相邻点在路网中的连通与距离一致性。

该实现适合低采样率轨迹批处理。

### 2.2 输入接口（路网文件）
> 默认读取当前目录下 `Point.csv` / `Edge.csv` / `Network.csv`。

#### A. `Point.csv`
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `node` | 节点 ID | 无 | `1001` |
| `lng` | 经度 | 度（°） | `121.4737` |
| `lat` | 纬度 | 度（°） | `31.2304` |

#### B. `Edge.csv`
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `edge` | 路段 ID | 无 | `20001` |
| `s_node` | 起点节点 ID | 无 | `1001` |
| `e_node` | 终点节点 ID | 无 | `1002` |
| `s_lng` | 起点经度 | 度（°） | `121.4731` |
| `s_lat` | 起点纬度 | 度（°） | `31.2300` |
| `e_lng` | 终点经度 | 度（°） | `121.4742` |
| `e_lat` | 终点纬度 | 度（°） | `31.2310` |
| `c_lng` | 中点经度 | 度（°） | `121.4737` |
| `c_lat` | 中点纬度 | 度（°） | `31.2305` |

#### C. `Network.csv`
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `section_id` | 路段 ID（与 `edge` 对应） | 无 | `20001` |
| `s_node` | 起点节点 ID | 无 | `1001` |
| `e_node` | 终点节点 ID | 无 | `1002` |
| `length` | 路段长度 | 米（m） | `126.4` |

### 2.3 输入接口（轨迹数据）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `TRAJ_ID` | 轨迹 ID | 无 | `T_20260509_001` |
| `LON` | 经度 | 度（°） | `121.4739` |
| `LAT` | 纬度 | 度（°） | `31.2306` |

### 2.4 输出接口（`trajectory_matching` 返回）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `TRAJ_ID` | 轨迹 ID | 无 | `T_20260509_001` |
| `MATCHED_EDGE` | 匹配后的边 ID 序列 | 列表 | `[20001,20002,20010]` |
| `MATCHED_NODE` | 匹配后的节点 ID 序列 | 列表 | `[1001,1002,1008]` |

失败场景（候选不足、不可达）时：`MATCHED_EDGE=-1`, `MATCHED_NODE=-1`。

### 2.5 关键参数（实现默认值）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `loninter` | 候选搜索经度窗口 | 度（°） | `0.000976` |
| `latinter` | 候选搜索纬度窗口 | 度（°） | `0.0009` |
| `shortest_dist<=35` | 点到边候选筛选阈值 | 米（m） | `35` |
| `sigma≈20` | 观测概率高斯噪声参数 | 米（m） | `20` |

---

## 3) GraphHopper Map Matching

### 3.1 算法原理概述
基于 HMM + Viterbi：
- 候选搜索：每个 GPS 点周围道路候选；
- 概率建模：观测距离 + 转移路网距离；
- 解码：hmm-lib 求最优候选序列。

### 3.2 输入接口（CLI）
#### A. 地图导入命令
`java -jar ... import <map.osm.pbf>`

| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `import` | 导入子命令 | 无 | `import` |
| `<map.osm.pbf>` | OSM 地图数据 | 文件路径 | `map-data/leipzig_germany.osm.pbf` |
| `--vehicle` | 车辆/出行方式（可选） | 无 | `car` |

#### B. 轨迹匹配命令
`java -jar ... match <*.gpx>`

| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `match` | 匹配子命令 | 无 | `match` |
| `<*.gpx>` | GPX 轨迹文件（可通配） | 文件路径 | `trace/test1.gpx` |
| `--vehicle` | 匹配所用 profile（可选） | 无 | `bike` |

### 3.3 输入接口（Web API）
**Endpoint**: `POST /match?vehicle=car&type=json`

#### Query 参数
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `vehicle` | 出行方式 profile | 无 | `car` |
| `type` | 返回格式 | 无 | `json` / `gpx` |

#### Header
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `Content-Type` | 请求体类型 | MIME | `application/gpx+xml` |

#### Body（GPX）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `trkpt@lat` | 轨迹点纬度 | 度（°） | `31.2304` |
| `trkpt@lon` | 轨迹点经度 | 度（°） | `121.4737` |
| `trkpt/time` | 采样时间（可选） | ISO8601 | `2026-05-09T08:00:00Z` |

### 3.4 输出接口
#### A. Web `type=json`（常见结构）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `paths` | 匹配结果路径数组 | 列表 | `[{...}]` |
| `paths[].distance` | 匹配路径长度 | 米（m） | `3421.6` |
| `paths[].time` | 估计通行时间 | 毫秒（ms） | `412000` |
| `paths[].points` | 匹配几何（编码点串/坐标） | 无 | `{"type":"LineString"...}` |

#### B. Web `type=gpx`
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `gpx/trk/trkseg/trkpt` | 匹配后的轨迹点 | 经/纬度 | `<trkpt lat="..." lon="...">` |

---

## 4) offline-map-matching（hmm-lib 离线框架示例）

### 4.1 算法原理概述
该项目提供 HMM 框架，不绑定具体地图引擎；你需要自行接入候选生成、距离计算、最短路。

### 4.2 输入接口（需自行实现）
#### A. 观测输入（GPS 点）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `obs_id` | 观测点序号 | 无 | `0` |
| `lon` | 经度 | 度（°） | `121.4737` |
| `lat` | 纬度 | 度（°） | `31.2304` |
| `time` | 时间戳（可选） | ISO8601 | `2026-05-09T08:00:00Z` |

#### B. 候选输入（每个观测点对应）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `candidate_id` | 候选状态 ID | 无 | `obs0_c1` |
| `edge_id` | 所属路段 ID | 无 | `E20001` |
| `emission_cost` | 观测代价（点到边距离） | 米（m）或代价值 | `8.4` |

#### C. 转移输入（相邻观测点候选对）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `from_candidate` | 起始候选 ID | 无 | `obs0_c1` |
| `to_candidate` | 终止候选 ID | 无 | `obs1_c2` |
| `transition_cost` | 转移代价（最短路相关） | 米（m）或代价值 | `132.7` |

### 4.3 输出接口（典型）
| 字段名 | 解释 | 单位 | 示例 |
|---|---|---|---|
| `state_sequence` | 最优候选状态序列 | 列表 | `[obs0_c1, obs1_c2, ...]` |
| `edge_sequence` | 对应道路序列 | 列表 | `[E20001, E20005, ...]` |
| `score` | 序列总分/似然 | 无 | `-123.45` |

---
