#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
HMM地图匹配主调用脚本 - 优化版
功能：
1. 读取Excel数据（GPS轨迹、路网）
2. 数据格式转换
3. 调用优化版HMM算法进行地图匹配
4. 输出匹配结果
"""

import os
import sys
import pandas as pd
from shapely.geometry import LineString
from shapely.wkt import loads as wkt_loads
from datetime import datetime

# 添加当前目录到路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from hmm_map_matching import HMMMapMatching, Point, Edge

# 路径配置
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data')
OUTPUT_DIR = os.path.join(BASE_DIR, 'output')

# 输入文件路径
GPS_FILE = os.path.join(DATA_DIR, 'gps_data.xlsx')
ROAD_NETWORK_FILE = os.path.join(DATA_DIR, 'road_network.xlsx')
GROUND_TRUTH_FILE = os.path.join(DATA_DIR, 'ground_truth_route.xlsx')

# 输出文件路径
OUTPUT_RESULT = os.path.join(OUTPUT_DIR, 'match_result.csv')
OUTPUT_MATCHED_PATH = os.path.join(OUTPUT_DIR, 'matched_path.txt')


def create_output_dir():
    """创建输出目录"""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"输出目录已创建：{OUTPUT_DIR}")


def read_gps_data():
    """读取GPS轨迹数据（支持时间戳）"""
    print("\n[1/3] 读取GPS轨迹数据...")

    try:
        df = pd.read_excel(GPS_FILE)
        print(f"  读取成功，共 {len(df)} 条记录")
        print(f"  原始列名：{list(df.columns)}")

        # 清理列名
        df.columns = [col.strip() for col in df.columns]
        print(f"  清理后列名：{list(df.columns)}")

        # 检查必需列
        required_cols = ['Latitude', 'Longitude']
        if not all(col in df.columns for col in required_cols):
            print(f"  错误：缺少必需列！需要 {required_cols}")
            print(f"  当前列：{list(df.columns)}")
            return None

        # 检查可选列（时间戳）
        has_timestamp = 'Date (UTC)' in df.columns and 'Time (UTC)' in df.columns

        # 转换为Point对象列表（带时间戳）
        trajectory = []
        for _, row in df.iterrows():
            lon = row['Longitude']
            lat = row['Latitude']
            
            if pd.notnull(lon) and pd.notnull(lat):
                timestamp = None
                if has_timestamp:
                    try:
                        date_str = str(row['Date (UTC)'])
                        time_str = str(row['Time (UTC)'])
                        datetime_str = f"{date_str} {time_str}"
                        timestamp = datetime.strptime(datetime_str, '%d-%b-%Y %H:%M:%S')
                    except:
                        pass
                
                trajectory.append(Point(lon, lat, timestamp))

        print(f"  有效GPS点：{len(trajectory)} 个")
        print(f"  时间戳支持：{'是' if has_timestamp else '否'}")
        
        return trajectory

    except Exception as e:
        print(f"  错误：{e}")
        import traceback
        traceback.print_exc()
        return None


def read_road_network():
    """读取路网数据（支持限速和双向属性）"""
    print("\n[2/3] 读取路网数据...")

    try:
        df = pd.read_excel(ROAD_NETWORK_FILE)
        print(f"  读取成功，共 {len(df)} 条记录")
        print(f"  原始列名：{list(df.columns)}")

        # 清理列名
        df.columns = [col.strip() for col in df.columns]
        print(f"  清理后列名：{list(df.columns)}")

        # 检查必需列
        required_cols = ['Edge ID', 'From Node ID', 'To Node ID', 'LINESTRING()']
        if not all(col in df.columns for col in required_cols):
            print(f"  错误：缺少必需列！需要 {required_cols}")
            print(f"  当前列：{list(df.columns)}")
            return None

        # 转换为Edge对象列表
        edges = []
        for _, row in df.iterrows():
            edge_id = row['Edge ID']
            source = row['From Node ID']
            target = row['To Node ID']
            wkt_str = row['LINESTRING()']
            
            # 获取限速（如果存在）
            speed_limit = row.get('Speed (m/s)', 22.22)
            
            # 获取双向属性（如果存在）
            two_way = True
            if 'Two Way' in df.columns:
                two_way = bool(row['Two Way'])

            try:
                geom = wkt_loads(wkt_str)
                edge = Edge(edge_id, source, target, geom, speed_limit, two_way)
                edges.append(edge)
            except Exception as e:
                print(f"  警告：解析路段 {edge_id} 失败 - {e}")

        print(f"  有效路段：{len(edges)} 条")
        return edges

    except Exception as e:
        print(f"  错误：{e}")
        import traceback
        traceback.print_exc()
        return None


def progress_callback(step, total, message):
    """进度回调函数"""
    progress = (step / total) * 100
    bar_length = 40
    filled_length = int(bar_length * step // total)
    bar = '█' * filled_length + '░' * (bar_length - filled_length)
    print(f"\r  [{bar}] {progress:.1f}% - {message}", end='', flush=True)


def run_hmm_matching(trajectory, edges):
    """运行优化版HMM地图匹配"""
    print("\n[3/3] 执行HMM地图匹配...")

    try:
        hmm = HMMMapMatching(edges)
        print(f"  HMM实例创建成功")

        print(f"  开始匹配...")
        result = hmm.match(
            trajectory,
            k=5,           # 候选点数量
            radius=150,    # 搜索半径(米)
            gps_error=50,  # GPS误差(米)
            progress_callback=progress_callback
        )

        print()
        print(f"  匹配完成，共匹配 {len(result)} 个点")
        return result

    except Exception as e:
        print()
        print(f"  错误：{e}")
        import traceback
        traceback.print_exc()
        return None


def save_results(result, trajectory):
    """保存匹配结果"""
    print("\n[4/4] 保存匹配结果...")

    result_data = []
    for i, cand in enumerate(result):
        gps_point = trajectory[i]
        timestamp_str = gps_point.timestamp.strftime('%Y-%m-%d %H:%M:%S') if gps_point.timestamp else ''
        
        result_data.append({
            'POINT_INDEX': i,
            'TIMESTAMP': timestamp_str,
            'GPS_LON': gps_point.lon,
            'GPS_LAT': gps_point.lat,
            'MATCHED_EDGE': cand.edge.id,
            'MATCHED_LON': cand.point.lon,
            'MATCHED_LAT': cand.point.lat,
            'MATCHED_NODE': cand.edge.source if cand.offset < cand.edge.length/2 else cand.edge.target,
            'DISTANCE_TO_EDGE': cand.distance,
            'OFFSET_ON_EDGE': cand.offset,
            'SPEED_LIMIT': cand.edge.speed_limit
        })

    df = pd.DataFrame(result_data)
    df.to_csv(OUTPUT_RESULT, index=False, encoding='utf-8')
    print(f"  匹配结果已保存：{OUTPUT_RESULT}")

    matched_coords = [(cand.point.lon, cand.point.lat) for cand in result]
    line_string = LineString(matched_coords)
    with open(OUTPUT_MATCHED_PATH, 'w', encoding='utf-8') as f:
        f.write(f"MATCHED_PATH\n")
        f.write(f"{line_string.wkt}\n")
        f.write(f"\nEDGE_SEQUENCE\n")
        edges_str = ' -> '.join([str(cand.edge.id) for cand in result])
        f.write(edges_str)
    print(f"  匹配路径已保存：{OUTPUT_MATCHED_PATH}")

    print("\n匹配结果摘要：")
    print(f"  原始GPS点数：{len(trajectory)}")
    print(f"  匹配成功点数：{len(result)}")
    avg_distance = sum(c.distance for c in result) / len(result)
    print(f"  平均匹配距离：{avg_distance:.2f} 米")


def main():
    print("=" * 70)
    print("HMM地图匹配 - 优化版")
    print("=" * 70)

    create_output_dir()

    trajectory = read_gps_data()
    if not trajectory:
        print("\n错误：GPS数据读取失败")
        return

    edges = read_road_network()
    if not edges:
        print("\n错误：路网数据读取失败")
        return

    result = run_hmm_matching(trajectory, edges)
    if not result:
        print("\n错误：匹配失败")
        return

    save_results(result, trajectory)

    print("\n" + "=" * 70)
    print("任务完成！")
    print(f"输出目录：{OUTPUT_DIR}")
    print("=" * 70)


if __name__ == '__main__':
    main()
