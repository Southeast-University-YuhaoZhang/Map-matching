#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
路网数据调试脚本 - 可视化路网
"""

import os
import pandas as pd
from shapely.geometry import LineString
from shapely.wkt import loads as wkt_loads

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data')
OUTPUT_DIR = os.path.join(BASE_DIR, 'output')
ROAD_NETWORK_FILE = os.path.join(DATA_DIR, 'road_network.xlsx')

def plot_road_network():
    print("=" * 60)
    print("路网可视化")
    print("=" * 60)
    
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    df = pd.read_excel(ROAD_NETWORK_FILE)
    df.columns = [col.strip() for col in df.columns]
    
    print(f"读取路网数据: {len(df)} 条边")
    
    edge_geometry = {}
    for _, row in df.iterrows():
        edge_id = str(row['Edge ID'])
        wkt_str = row['LINESTRING()']
        try:
            geom = wkt_loads(wkt_str)
            edge_geometry[edge_id] = geom
        except:
            pass
    
    print(f"成功解析: {len(edge_geometry)} 条边")
    
    # 尝试使用 folium 绘制
    try:
        import folium
        from folium import PolyLine
        
        center_lat = 47.66
        center_lon = -122.12
        
        m = folium.Map(location=[center_lat, center_lon], zoom_start=14)
        
        for edge_id, geom in edge_geometry.items():
            if isinstance(geom, LineString):
                coords = [[c[1], c[0]] for c in geom.coords]
                PolyLine(coords, color='gray', weight=2, opacity=0.5).add_to(m)
        
        legend_html = '''
        <div style="position: fixed; bottom: 50px; left: 50px; z-index: 1000; background-color: white; padding: 10px; border: 2px solid gray; border-radius: 5px;">
            <b>Road Network</b><br>
            <i style="background: gray; width: 20px; height: 5px; display: inline-block;"></i> All Roads
        </div>
        '''
        m.get_root().html.add_child(folium.Element(legend_html))
        
        output_file = os.path.join(OUTPUT_DIR, 'road_network_map.html')
        m.save(output_file)
        print(f"路网地图已保存: {output_file}")
        
    except ImportError:
        # 备用方案：使用 matplotlib
        try:
            import matplotlib.pyplot as plt
            
            fig, ax = plt.subplots(1, 1, figsize=(12, 8))
            
            for edge_id, geom in edge_geometry.items():
                if isinstance(geom, LineString):
                    coords = list(geom.coords)
                    lons = [c[0] for c in coords]
                    lats = [c[1] for c in coords]
                    ax.plot(lons, lats, 'gray', linewidth=0.5, alpha=0.5)
            
            ax.set_xlabel('Longitude')
            ax.set_ylabel('Latitude')
            ax.set_title('Road Network Visualization')
            ax.grid(True, alpha=0.3)
            
            output_file = os.path.join(OUTPUT_DIR, 'road_network_map.png')
            plt.tight_layout()
            plt.savefig(output_file, dpi=150)
            plt.close()
            print(f"路网地图已保存: {output_file}")
            
        except ImportError:
            print("错误: 未安装 folium 或 matplotlib")

if __name__ == '__main__':
    plot_road_network()
