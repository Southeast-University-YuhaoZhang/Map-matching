#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
地图匹配结果评估脚本
功能：对比匹配结果与Ground Truth，计算准确率、召回率、F1分数
"""

import os
import pandas as pd
from datetime import datetime, timedelta
from shapely.geometry import LineString
from shapely.wkt import loads as wkt_loads

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(BASE_DIR, 'output')
DATA_DIR = os.path.join(BASE_DIR, 'data')

GROUND_TRUTH_FILE = os.path.join(DATA_DIR, 'ground_truth_route.xlsx')
MATCH_RESULT_FILE = os.path.join(OUTPUT_DIR, 'match_result.csv')
GPS_FILE = os.path.join(DATA_DIR, 'gps_data.xlsx')
ROAD_NETWORK_FILE = os.path.join(DATA_DIR, 'road_network.xlsx')


def load_ground_truth():
    df = pd.read_excel(GROUND_TRUTH_FILE)
    df.columns = [col.strip() for col in df.columns]

    # 直接读取所有边作为真值（每行都是真值）
    all_edges = df['Edge ID'].astype(str).tolist()
    return set(all_edges), len(df)


def load_gps_start_time():
    """从GPS数据获取起始时间戳"""
    try:
        df = pd.read_excel(GPS_FILE)
        df.columns = [col.strip() for col in df.columns]

        first_date = df['Date (UTC)'].iloc[0]
        first_time = df['Time (UTC)'].iloc[0]

        if isinstance(first_date, datetime):
            return first_date
        elif hasattr(first_date, 'to_pydatetime'):
            return first_date.to_pydatetime()
        else:
            return datetime.strptime(f"{first_date} {first_time}", '%d-%b-%Y %H:%M:%S')
    except:
        return None


def load_matched_edges():
    """加载匹配结果"""
    df = pd.read_csv(MATCH_RESULT_FILE)
    matched_edges = df['MATCHED_EDGE'].astype(str).tolist()
    return matched_edges, len(df)


def load_road_network():
    """加载路网数据"""
    df = pd.read_excel(ROAD_NETWORK_FILE)
    df.columns = [col.strip() for col in df.columns]
    edge_geometry = {}
    for _, row in df.iterrows():
        edge_id = str(row['Edge ID'])
        wkt_str = row['LINESTRING()']
        try:
            geom = wkt_loads(wkt_str)
            edge_geometry[edge_id] = geom
        except:
            pass
    return edge_geometry


def build_route_geometry(edge_ids, edge_geometry):
    """根据边ID列表构建路线几何（按顺序连接）"""
    if not edge_ids:
        return None

    unique_edges = list(dict.fromkeys(edge_ids))
    coords = []
    for edge_id in unique_edges:
        if edge_id in edge_geometry:
            geom = edge_geometry[edge_id]
            if isinstance(geom, LineString):
                edge_coords = list(geom.coords)
                if not coords:
                    coords.extend(edge_coords)
                else:
                    last_coord = coords[-1]
                    first_coord = edge_coords[0]
                    if (last_coord[0] - first_coord[0])**2 + (last_coord[1] - first_coord[1])**2 < 1e-10:
                        coords.extend(edge_coords[1:])
                    else:
                        coords.extend(edge_coords)
    return LineString(coords) if coords else None


def plot_routes_on_map(predicted_route, gt_edges, edge_geometry, output_file):
    """在地图上绘制预测路线和真值边"""
    try:
        import folium
        from folium import PolyLine, FeatureGroup

        center_lat = 47.66
        center_lon = -122.12

        m = folium.Map(location=[center_lat, center_lon], zoom_start=14)

        gt_group = FeatureGroup(name='Ground Truth (Blue)')
        for edge_id in gt_edges:
            if edge_id in edge_geometry:
                geom = edge_geometry[edge_id]
                if isinstance(geom, LineString):
                    coords = [[c[1], c[0]] for c in geom.coords]
                    PolyLine(coords, color='blue', weight=4, opacity=0.7).add_to(gt_group)
        gt_group.add_to(m)

        if predicted_route and isinstance(predicted_route, LineString):
            pred_group = FeatureGroup(name='Predicted (Red)')
            pred_coords = [[c[1], c[0]] for c in predicted_route.coords]
            PolyLine(pred_coords, color='red', weight=5, opacity=0.9).add_to(pred_group)
            folium.Marker(
                pred_coords[0],
                popup='Pred Start',
                icon=folium.Icon(color='red', icon='play')
            ).add_to(pred_group)
            folium.Marker(
                pred_coords[-1],
                popup='Pred End',
                icon=folium.Icon(color='red', icon='stop')
            ).add_to(pred_group)
            pred_group.add_to(m)

        folium.LayerControl().add_to(m)

        legend_html = '''
        <div style="position: fixed; bottom: 50px; left: 50px; z-index: 1000; background-color: white; padding: 10px; border: 2px solid gray; border-radius: 5px;">
            <b>Route Legend</b><br>
            <i style="background: blue; width: 20px; height: 5px; display: inline-block;"></i> Ground Truth (散点)<br>
            <i style="background: red; width: 20px; height: 5px; display: inline-block;"></i> Predicted (路线)
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend_html))

        m.save(output_file)
        print(f"  路线图已保存: {output_file}")
        return True
    except ImportError:
        print("  警告: folium未安装，跳过地图绘制")
        print("  可运行: pip install folium")
        return False


def plot_routes_matplotlib(predicted_route, gt_edges, edge_geometry, output_file):
    """使用matplotlib绘制路线对比图"""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch

        fig, ax = plt.subplots(1, 1, figsize=(12, 8))

        for edge_id in gt_edges:
            if edge_id in edge_geometry:
                geom = edge_geometry[edge_id]
                if isinstance(geom, LineString):
                    coords = list(geom.coords)
                    lons = [c[0] for c in coords]
                    lats = [c[1] for c in coords]
                    ax.plot(lons, lats, 'b-', linewidth=1.5, alpha=0.5)

        if predicted_route and isinstance(predicted_route, LineString):
            pred_coords = list(predicted_route.coords)
            lons = [c[0] for c in pred_coords]
            lats = [c[1] for c in pred_coords]
            ax.plot(lons, lats, 'r-', linewidth=3, label='Predicted', alpha=0.9)
            ax.scatter(lons[0], lats[0], c='red', s=100, marker='o', zorder=5, label='Start')
            ax.scatter(lons[-1], lats[-1], c='red', s=100, marker='s', zorder=5, label='End')

        legend_elements = [
            Patch(facecolor='blue', alpha=0.5, label='Ground Truth (散点)'),
            plt.Line2D([0], [0], color='red', linewidth=3, label='Predicted (路线)')
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.set_title('Map Matching Route Comparison')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(output_file, dpi=150)
        print(f"  路线图已保存: {output_file}")
        plt.close()
        return True
    except Exception as e:
        print(f"  警告: matplotlib绑定失败 - {e}")
        return False


def evaluate(ground_truth_edges, matched_edges, total_gps_points, start_time):
    correct = sum(1 for e in matched_edges if e in ground_truth_edges)
    unique_matched = set(matched_edges)

    correct_minutes = 0
    total_minutes = 0

    if start_time:
        minute_buckets = {}
        for i, edge in enumerate(matched_edges):
            minute_key = (start_time + timedelta(seconds=i)).strftime('%Y-%m-%d %H:%M')
            if minute_key not in minute_buckets:
                minute_buckets[minute_key] = set()
            minute_buckets[minute_key].add(edge)

        total_minutes = len(minute_buckets)
        correct_minutes = sum(1 for edges in minute_buckets.values() if edges.issubset(ground_truth_edges))

    minute_accuracy = correct_minutes / total_minutes if total_minutes > 0 else 0

    return {
        'num_predicted': len(unique_matched),
        'num_ground_truth': len(ground_truth_edges),
        'num_correct': len(unique_matched & ground_truth_edges),
        'total_gps_points': total_gps_points,
        'correct_seconds': correct,
        'second_accuracy': correct / total_gps_points if total_gps_points > 0 else 0,
        'total_minutes': total_minutes,
        'correct_minutes': correct_minutes,
        'minute_accuracy': minute_accuracy
    }


def main():
    print("=" * 60)
    print("地图匹配评估")
    print("=" * 60)

    if not os.path.exists(GROUND_TRUTH_FILE) or not os.path.exists(MATCH_RESULT_FILE):
        print("错误：文件不存在")
        return

    print("\n[1/3] 加载数据...")
    ground_truth_edges, _ = load_ground_truth()
    matched_edges, total_gps_points = load_matched_edges()
    start_time = load_gps_start_time()

    print("[2/3] 评估匹配结果...")
    r = evaluate(ground_truth_edges, matched_edges, total_gps_points, start_time)

    precision = r['num_correct'] / r['num_predicted'] if r['num_predicted'] > 0 else 0
    recall = r['num_correct'] / r['num_ground_truth'] if r['num_ground_truth'] > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

    print("\n[3/3] 绘制路线对比图...")

    if os.path.exists(ROAD_NETWORK_FILE):
        edge_geometry = load_road_network()
        predicted_route = build_route_geometry(matched_edges, edge_geometry)

        html_file = os.path.join(OUTPUT_DIR, 'route_comparison_map.html')
        png_file = os.path.join(OUTPUT_DIR, 'route_comparison.png')

        folium_ok = plot_routes_on_map(predicted_route, ground_truth_edges, edge_geometry, html_file)
        if not folium_ok:
            plot_routes_matplotlib(predicted_route, ground_truth_edges, edge_geometry, png_file)
    else:
        print("  警告: 路网文件不存在，无法绘制路线图")

    print(f"\n【边数统计】")
    print(f"  预测匹配边数:    {r['num_predicted']}")
    print(f"  Ground Truth边数: {r['num_ground_truth']}")
    print(f"  正确匹配边数:    {r['num_correct']}")

    print(f"\n【评估指标】")
    print(f"  准确率: {precision:.4f}")
    print(f"  召回率: {recall:.4f}")
    print(f"  F1分数: {f1:.4f}")

    print(f"\n【时间准确率指标（按秒）】")
    print(f"  总GPS点数: {r['total_gps_points']}")
    print(f"  正确秒数: {r['correct_seconds']}")
    print(f"  秒准确率: {r['second_accuracy']:.4f}")

    print(f"\n【时间准确率指标（按分钟）】")
    print(f"  总分钟数: {r['total_minutes']}")
    print(f"  正确分钟数: {r['correct_minutes']}")
    print(f"  分钟准确率: {r['minute_accuracy']:.4f}")

    print("\n" + "=" * 60)


if __name__ == '__main__':
    main()
