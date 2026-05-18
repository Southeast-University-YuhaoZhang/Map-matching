#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
HMM（隐马尔可夫模型）地图匹配算法 - 优化版本 v2
针对问题：边连续性约束不足，导致一条路只被匹配一次

优化点：
1. 添加"边连续性"约束 - 优先选择与前一个候选点属于同一条边的候选
2. 加强方向一致性约束
3. 使用空间连续性判断减少"跳边"
"""

import numpy as np
import math
import heapq
from collections import defaultdict
from datetime import datetime

# 常量定义
EARTH_RADIUS = 6371000  # 地球半径(米)

# 网格参数（约100米范围）
LAT_INTERVAL = 0.0009    # 约100米纬度
LON_INTERVAL = 0.001337  # 约100米经度

class Point:
    """点坐标类"""
    def __init__(self, lon, lat, timestamp=None):
        self.lon = lon
        self.lat = lat
        self.timestamp = timestamp

class Edge:
    """路段类"""
    def __init__(self, edge_id, source, target, geometry, speed_limit=22.22, two_way=True):
        self.id = edge_id
        self.source = source
        self.target = target
        self.geometry = geometry
        self.length = self.calculate_length()
        self.speed_limit = speed_limit
        self.two_way = two_way
        
        coords = list(geometry.coords)
        self.lons = [c[0] for c in coords]
        self.lats = [c[1] for c in coords]
        self.min_lon, self.max_lon = min(self.lons), max(self.lons)
        self.min_lat, self.max_lat = min(self.lats), max(self.lats)
        
        if len(coords) >= 2:
            start = Point(coords[0][0], coords[0][1])
            end = Point(coords[-1][0], coords[-1][1])
            self.direction = math.atan2(end.lat - start.lat, end.lon - start.lon)
    
    def calculate_length(self):
        coords = list(self.geometry.coords)
        total_length = 0
        for i in range(len(coords)-1):
            p1 = Point(coords[i][0], coords[i][1])
            p2 = Point(coords[i+1][0], coords[i+1][1])
            total_length += haversine_distance(p1, p2)
        return total_length
    
    def get_direction(self):
        return getattr(self, 'direction', 0.0)
    
    def distance_to_point(self, point):
        """计算点到边的最近距离"""
        coords = list(self.geometry.coords)
        min_dist = float('inf')
        for i in range(len(coords) - 1):
            seg_start = coords[i]
            seg_end = coords[i + 1]
            dist, _, _ = point_to_segment_distance(point, seg_start, seg_end)
            if dist < min_dist:
                min_dist = dist
        return min_dist

class Candidate:
    """候选点类"""
    def __init__(self, edge, offset, distance, point, direction_diff=0, edge_continuity=0):
        self.edge = edge
        self.offset = offset
        self.distance = distance
        self.point = point
        self.direction_diff = direction_diff
        self.edge_continuity = edge_continuity  # 边连续性得分
        self.obs_prob = 0.0
        self.path_prob = float('-inf')
        self.prev_candidate = None

def haversine_distance(p1, p2):
    """计算两点间的球面距离(米)"""
    lat1, lon1 = math.radians(p1.lat), math.radians(p1.lon)
    lat2, lon2 = math.radians(p2.lat), math.radians(p2.lon)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    return EARTH_RADIUS * c

def point_to_segment_distance(point, seg_start, seg_end):
    """计算点到线段的距离和投影点"""
    p = np.array([point.lon, point.lat])
    a = np.array([seg_start[0], seg_start[1]])
    b = np.array([seg_end[0], seg_end[1]])
    
    ap = p - a
    ab = b - a
    ab_len_sq = np.dot(ab, ab)
    
    if ab_len_sq == 0:
        dist = haversine_distance(point, Point(a[0], a[1]))
        return dist, Point(a[0], a[1]), 0.0
    
    t = max(0, min(1, np.dot(ap, ab) / ab_len_sq))
    projection = a + t * ab
    
    offset = t * haversine_distance(Point(a[0], a[1]), Point(b[0], b[1]))
    proj_point = Point(projection[0], projection[1])
    dist = haversine_distance(point, proj_point)
    
    return dist, proj_point, offset

def calculate_bearing(p1, p2):
    """计算两点间的方位角（弧度）"""
    lat1, lon1 = math.radians(p1.lat), math.radians(p1.lon)
    lat2, lon2 = math.radians(p2.lat), math.radians(p2.lon)
    
    dlon = lon2 - lon1
    
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    return math.atan2(y, x)

class HMMMapMatching:
    """HMM地图匹配算法主类 - 优化版 v2"""
    
    def __init__(self, edges):
        self.edges = edges
        self.num_edges = len(edges)
        
        print("  正在构建网格索引...")
        self.grid = defaultdict(list)
        self._build_grid_index()
        print(f"  网格索引构建完成")
        
        print("  正在构建邻接表...")
        self.adj_list = defaultdict(list)
        for edge in edges:
            self.adj_list[edge.source].append((edge.target, edge.id, edge.length, edge.speed_limit))
            if edge.two_way:
                self.adj_list[edge.target].append((edge.source, edge.id, edge.length, edge.speed_limit))
        print(f"  邻接表构建完成")
        
        self.edge_id_map = {edge.id: edge for edge in edges}
    
    def _build_grid_index(self):
        for edge in self.edges:
            min_grid_x = int(math.floor(edge.min_lon / LON_INTERVAL))
            max_grid_x = int(math.floor(edge.max_lon / LON_INTERVAL))
            min_grid_y = int(math.floor(edge.min_lat / LAT_INTERVAL))
            max_grid_y = int(math.floor(edge.max_lat / LAT_INTERVAL))
            
            for grid_x in range(min_grid_x, max_grid_x + 1):
                for grid_y in range(min_grid_y, max_grid_y + 1):
                    self.grid[(grid_x, grid_y)].append(edge)
    
    def _get_candidate_edges_from_grid(self, lon, lat, expand=1):
        grid_x = int(math.floor(lon / LON_INTERVAL))
        grid_y = int(math.floor(lat / LAT_INTERVAL))
        
        candidate_edges = set()
        
        for dx in range(-expand, expand + 1):
            for dy in range(-expand, expand + 1):
                key = (grid_x + dx, grid_y + dy)
                if key in self.grid:
                    candidate_edges.update(self.grid[key])
        
        return list(candidate_edges)
    
    def _dijkstra(self, source, max_dist=None):
        """Dijkstra算法计算最短路径"""
        dist = {source: 0}
        time_dist = {source: 0}
        heap = [(0, 0, source)]
        
        while heap:
            current_dist, current_time, u = heapq.heappop(heap)
            
            if max_dist is not None and current_dist > max_dist:
                continue
            
            if current_dist > dist.get(u, float('inf')):
                continue
            
            for v, edge_id, length, speed_limit in self.adj_list.get(u, []):
                travel_time = length / speed_limit
                alt_dist = current_dist + length
                alt_time = current_time + travel_time
                
                if alt_dist < dist.get(v, float('inf')):
                    if max_dist is None or alt_dist <= max_dist:
                        dist[v] = alt_dist
                        time_dist[v] = alt_time
                        heapq.heappush(heap, (alt_dist, alt_time, v))
        
        return dist, time_dist
    
    def _generate_candidates(self, gps_point, radius=300, prev_bearing=None, prev_edge_id=None, 
                           direction_weight=0.3, continuity_weight=50):
        """
        生成候选点（考虑边连续性）
        prev_edge_id: 前一个匹配点所属的边ID，用于判断连续性
        """
        candidates = []
        
        candidate_edges = self._get_candidate_edges_from_grid(gps_point.lon, gps_point.lat, expand=2)
        
        for edge in candidate_edges:
            coords = list(edge.geometry.coords)
            min_dist = float('inf')
            best_proj = None
            best_offset = 0.0
            best_dir_diff = 0.0
            
            for i in range(len(coords) - 1):
                seg_start = coords[i]
                seg_end = coords[i + 1]
                dist, proj_point, offset = point_to_segment_distance(gps_point, seg_start, seg_end)
                
                if dist < min_dist:
                    min_dist = dist
                    best_proj = proj_point
                    best_offset = offset
                    
                    if prev_bearing is not None:
                        seg_bearing = calculate_bearing(Point(seg_start[0], seg_start[1]), 
                                                       Point(seg_end[0], seg_end[1]))
                        dir_diff = abs(prev_bearing - seg_bearing)
                        if dir_diff > math.pi:
                            dir_diff = 2 * math.pi - dir_diff
                        best_dir_diff = dir_diff
                    else:
                        best_dir_diff = 0.0
            
            if min_dist <= radius:
                # 计算边连续性：如果和前一个边相同，得分高
                edge_continuity = 1 if (prev_edge_id is not None and edge.id == prev_edge_id) else 0
                
                candidates.append(Candidate(
                    edge=edge,
                    offset=best_offset,
                    distance=min_dist,
                    point=best_proj,
                    direction_diff=best_dir_diff,
                    edge_continuity=edge_continuity
                ))
        
        # 按距离、方向差异和边连续性综合排序
        # 边连续性权重很高，确保连续点尽量选同一条边
        candidates.sort(key=lambda c: (
            c.distance + direction_weight * c.direction_diff * 100 - continuity_weight * c.edge_continuity
        ))
        return candidates
    
    def _observation_probability(self, distance, gps_error=50):
        """计算观测概率（对数形式，高斯分布）"""
        sigma = gps_error
        return -distance**2 / (2 * sigma**2) - math.log(sigma * math.sqrt(2 * math.pi))
    
    def _transition_probability(self, sp_dist, time_interval, speed_limit=30):
        """计算转移概率（对数形式，指数分布）"""
        expected_dist = speed_limit * time_interval
        
        if expected_dist <= 0:
            expected_dist = 1.0
        
        beta = expected_dist / 2.0
        
        if sp_dist <= 0:
            sp_dist = 1e-10
        
        return -sp_dist / beta - math.log(beta)
    
    def _get_sp_distance(self, cand_a, cand_b):
        """获取两个候选点之间的最短路径距离和时间"""
        edge_a = cand_a.edge
        edge_b = cand_b.edge
        
        # 获取候选点在边上的实际位置（0到1之间的比例）
        ratio_a = cand_a.offset / edge_a.length if edge_a.length > 0 else 0.5
        ratio_b = cand_b.offset / edge_b.length if edge_b.length > 0 else 0.5
        
        # 根据投影位置选择端点：靠近起点选source，靠近终点选target
        node_a = edge_a.source if ratio_a < 0.5 else edge_a.target
        node_b = edge_b.source if ratio_b < 0.5 else edge_b.target
        
        # 先检查是否直接连接
        if edge_a.id == edge_b.id:
            # 同一条边，直接计算距离
            sp_dist = abs(cand_a.offset - cand_b.offset)
            sp_time = sp_dist / edge_a.speed_limit
            return sp_dist, sp_time
        
        # 检查是否直接相邻（共享节点）
        directly_connected = False
        if edge_a.target == edge_b.source or edge_a.target == edge_b.target:
            directly_connected = True
        elif edge_a.source == edge_b.source or edge_a.source == edge_b.target:
            directly_connected = True
        
        if directly_connected:
            # 直接连接的边，计算沿着边的距离
            dist_a = cand_a.offset if node_a == edge_a.source else edge_a.length - cand_a.offset
            dist_b = cand_b.offset if node_b == edge_b.source else edge_b.length - cand_b.offset
            sp_dist = dist_a + dist_b
            avg_speed = (edge_a.speed_limit + edge_b.speed_limit) / 2
            sp_time = sp_dist / avg_speed
            return sp_dist, sp_time
        
        # 如果不是直接连接，使用Dijkstra计算最短路径
        dist_dict, time_dict = self._dijkstra(node_a)
        
        if node_b in dist_dict:
            # 添加候选点到端点的距离
            dist_a = cand_a.offset if node_a == edge_a.source else edge_a.length - cand_a.offset
            dist_b = cand_b.offset if node_b == edge_b.source else edge_b.length - cand_b.offset
            sp_dist = dist_a + dist_dict[node_b] + dist_b
            sp_time = time_dict[node_b] + dist_a / edge_a.speed_limit + dist_b / edge_b.speed_limit
        else:
            # 不可达，使用直线距离（会被惩罚）
            sp_dist = haversine_distance(cand_a.point, cand_b.point) * 10  # 惩罚系数
            sp_time = sp_dist / 30.0
        
        return sp_dist, sp_time
    
    def match(self, trajectory, k=8, radius=300, gps_error=50, progress_callback=None):
        """
        执行HMM地图匹配（优化版 v2）
        添加边连续性约束，减少"跳边"现象
        """
        if not trajectory:
            return []
        
        total_steps = len(trajectory) * 2
        current_step = 0
        
        # 计算相邻GPS点的方位角
        bearings = []
        for i in range(1, len(trajectory)):
            bearing = calculate_bearing(trajectory[i-1], trajectory[i])
            bearings.append(bearing)
        bearings.append(None)
        
        # 计算时间间隔
        time_intervals = []
        for i in range(1, len(trajectory)):
            if trajectory[i].timestamp and trajectory[i-1].timestamp:
                dt = trajectory[i].timestamp - trajectory[i-1].timestamp
                time_intervals.append(dt.total_seconds())
            else:
                time_intervals.append(1.0)
        
        # 记录前一个匹配的边ID（用于连续性约束）
        prev_edge_id = None
        
        # 为每个GPS点生成候选点
        candidate_layers = []
        for i, point in enumerate(trajectory):
            prev_bearing = bearings[i-1] if i > 0 else None
            
            # 传入前一个边ID以增强连续性
            candidates = self._generate_candidates(
                point, radius, prev_bearing, prev_edge_id,
                direction_weight=0.3,
                continuity_weight=50  # 边连续性权重
            )
            candidates = candidates[:k]
            
            # 更新前一个边ID（使用概率最高的候选边）
            if candidates:
                best_cand = min(candidates, key=lambda c: c.distance)
                prev_edge_id = best_cand.edge.id
            
            for cand in candidates:
                cand.obs_prob = self._observation_probability(cand.distance, gps_error)
            
            candidate_layers.append(candidates)
            
            current_step += 1
            if progress_callback:
                progress_callback(current_step, total_steps, f"生成候选点 {i+1}/{len(trajectory)}")
        
        if not candidate_layers[0]:
            return []
        
        # Viterbi算法初始化
        for cand in candidate_layers[0]:
            cand.path_prob = cand.obs_prob
        
        # Viterbi算法逐层处理
        for i in range(1, len(candidate_layers)):
            current_layer = candidate_layers[i]
            prev_layer = candidate_layers[i-1]
            
            for curr_cand in current_layer:
                max_prob = float('-inf')
                best_prev = None
                
                for prev_cand in prev_layer:
                    sp_dist, sp_time = self._get_sp_distance(prev_cand, curr_cand)
                    
                    time_interval = time_intervals[i-1] if i-1 < len(time_intervals) else 1.0
                    
                    avg_speed = (prev_cand.edge.speed_limit + curr_cand.edge.speed_limit) / 2
                    
                    trans_prob = self._transition_probability(sp_dist, time_interval, avg_speed)
                    
                    # 速度约束惩罚：如果时间间隔内需要行驶的距离超过合理速度，给予惩罚
                    max_reasonable_dist = 50 * time_interval  # 最大合理速度约180 km/h
                    speed_penalty = 0
                    if sp_dist > max_reasonable_dist:
                        excess_ratio = sp_dist / max_reasonable_dist
                        speed_penalty = math.log(excess_ratio) * 2  # 超过越多惩罚越大
                    
                    # 方向一致性惩罚
                    dir_penalty = 0
                    if hasattr(curr_cand, 'direction_diff') and curr_cand.direction_diff > math.pi/4:
                        dir_penalty = curr_cand.direction_diff * 0.3
                    
                    # 边连续性奖励：如果和前一个边相同，给正向奖励
                    continuity_bonus = 0
                    if prev_cand.edge.id == curr_cand.edge.id:
                        continuity_bonus = 3.0  # 同边奖励
                    
                    # 检查边是否直接连接或相邻
                    edge_a = prev_cand.edge
                    edge_b = curr_cand.edge
                    is_connected = False
                    if edge_a.id == edge_b.id:
                        is_connected = True
                    elif edge_a.target == edge_b.source:
                        is_connected = True
                    elif edge_a.source == edge_b.target:
                        is_connected = True
                    elif edge_a.source == edge_b.source and edge_a.two_way and edge_b.two_way:
                        is_connected = True
                    elif edge_a.target == edge_b.target and edge_a.two_way and edge_b.two_way:
                        is_connected = True
                    
                    # 如果边不直接连接，给予惩罚
                    connection_penalty = 0
                    if not is_connected:
                        # 非直接连接惩罚，距离越远惩罚越大
                        connection_penalty = min(sp_dist / 100.0, 5.0)  # 每100米惩罚1，最大5
                    
                    total_prob = (prev_cand.path_prob + trans_prob + curr_cand.obs_prob 
                                 - dir_penalty + continuity_bonus - connection_penalty - speed_penalty)
                    
                    if total_prob > max_prob:
                        max_prob = total_prob
                        best_prev = prev_cand
                
                curr_cand.path_prob = max_prob
                curr_cand.prev_candidate = best_prev
            
            current_step += 1
            if progress_callback:
                progress_callback(current_step, total_steps, f"Viterbi处理 {i+1}/{len(trajectory)}")
        
        if progress_callback:
            progress_callback(total_steps, total_steps, "路径回溯中...")
        
        # 回溯找最优路径
        last_layer = candidate_layers[-1]
        best_candidate = max(last_layer, key=lambda c: c.path_prob)
        
        path = []
        current = best_candidate
        while current:
            path.append(current)
            current = current.prev_candidate
        
        path.reverse()
        return path
    
    def get_matched_edges(self, trajectory, **kwargs):
        path = self.match(trajectory, **kwargs)
        return [cand.edge.id for cand in path]
    
    def get_matched_path(self, trajectory, **kwargs):
        path = self.match(trajectory, **kwargs)
        matched_points = []
        for cand in path:
            matched_points.append((cand.point.lon, cand.point.lat))
        return matched_points
