import rasterio
import numpy as np
import folium
from folium.plugins import HeatMap
from rasterio.windows import from_bounds


def create_interactive_heatmap(tif_path, center_lat, center_lon, side_km, output_html):
    with rasterio.open(tif_path) as src:
        # 1. 计算裁剪边界 (5km 区域)
        delta_lat = (side_km / 2) / 111.0
        delta_lon = (side_km / 2) / (111.0 * np.cos(np.radians(center_lat)))

        west, south = center_lon - delta_lon, center_lat - delta_lat
        east, north = center_lon + delta_lon, center_lat + delta_lat

        # 2. 读取区域数据
        window = from_bounds(west, south, east, north, src.transform)
        data = src.read(1, window=window)
        win_transform = src.window_transform(window)

        # 3. 提取有效数据点的坐标和权重
        # 过滤掉无数据(NoData)和零值
        rows, cols = np.where(data > 0)
        weights = data[rows, cols]

        # 转换为经纬度
        lons, lats = rasterio.transform.xy(win_transform, rows, cols)

        # 组装热力图数据格式: [[lat, lon, weight], ...]
        heat_data = [[lat, lon, float(w)] for lat, lon, w in zip(lats, lons, weights)]

        # 4. 创建地图
        m = folium.Map(location=[center_lat, center_lon], zoom_start=14, tiles='OpenStreetMap')

        # 添加热力图层
        HeatMap(heat_data, radius=15, blur=10, max_zoom=1).add_to(m)

        m.save(output_html)
        print(f"交互式热力图已保存至: {output_html}")

# 调用示例
create_interactive_heatmap('../data/yushu_5km_pop.tif', 33.001, 97.009, 5, 'heatmap.html')