import pandas as pd
import folium
from folium.plugins import HeatMap


def draw_users_on_map(csv_path, output_html):
    # 1. 加载数据
    df = pd.read_csv(csv_path)

    # 2. 确定地图中心点
    center_lat = df['latitude'].mean()
    center_lon = df['longitude'].mean()

    # 3. 创建地图对象 (使用 OpenStreetMap 风格)
    m = folium.Map(location=[center_lat, center_lon],
                   zoom_start=14,
                   tiles='OpenStreetMap')

    # 4. 绘制点 (如果用户数太多，建议只画前1000个，或者使用聚合)
    # 这里我们演示绘制散点
    for _, row in df.head(1000).iterrows():
        folium.CircleMarker(
            location=[row['latitude'], row['longitude']],
            radius=2,
            color='red',
            fill=True,
            fill_color='red',
            fill_opacity=0.6
        ).add_to(m)

    # 5. 保存为 HTML
    m.save(output_html)
    print(f"地图已生成：{output_html}")

# 执行
draw_users_on_map('yushu_users_improved.csv', 'disaster_map.html')