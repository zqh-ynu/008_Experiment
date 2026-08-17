import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
import contextily as cx
from shapely.geometry import Point
import platform

# --- 字体设置 (保持不变) ---
system_name = platform.system()
if system_name == "Windows":
    plt.rcParams['font.sans-serif'] = ['SimHei']
elif system_name == "Darwin":
    plt.rcParams['font.sans-serif'] = ['Arial Unicode MS']
else:
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['pdf.fonttype'] = 42


# -------------------------

def plot_map_corrected(csv_path, output_path, style='light', n=None):
    # 1. 读取数据
    df = pd.read_csv(csv_path, nrows=n)
    if df.empty: return

    geometry = [Point(xy) for xy in zip(df['longitude'], df['latitude'])]
    gdf = gpd.GeoDataFrame(df, geometry=geometry, crs="EPSG:4326")

    # 2. 转换坐标系 -> Web Mercator (EPSG:3857)
    # 这一步非常重要！转换后单位变成了“米”，我们才能进行精确的距离平移
    gdf = gdf.to_crs(epsg=3857)

    # ================= [新增] 坐标修正 =================
    # 现状：点偏西(左) 1km，偏北(上) 1km
    # 修正：点需要往东(右)移 1000米，往南(下)移 1000米

    offset_x = 550  # 向东（右）平移 1000米
    offset_y = -400  # 向南（下）平移 1000米 (注意是负数)

    print(f"正在应用坐标修正: 向东 {offset_x}m, 向南 {-offset_y}m")
    gdf['geometry'] = gdf.geometry.translate(xoff=offset_x, yoff=offset_y)
    # ===================================================

    # 3. 绘图
    fig, ax = plt.subplots(figsize=(12, 12))

    point_color = 'cyan' if style == 'dark' else 'red'
    # 这里的 label 加个标注说明已修正
    gdf.plot(ax=ax, markersize=10, color=point_color, alpha=0.6, label='User Location (Corrected)')

    # 4. 加载底图
    # CartoDB Positron (浅色极简): cx.providers.CartoDB.Positron (强烈推荐)
    # CartoDB DarkMatter (深色极简): cx.providers.CartoDB.DarkMatter (适合做“发光”效果或者是夜间模式)
    # Esri World Imagery: cx.providers.Esri.WorldImagery (高清卫星地图，适合展示真实地形)
    # OpenTopoMap: cx.providers.OpenTopoMap (带有等高线，适合展示山区分布)
    # 街道底图： cx.providers.OpenStreetMap.Mapnik

    if style == 'light':
        cx.add_basemap(ax, source=cx.providers.CartoDB.Positron)
    elif style == 'dark':
        cx.add_basemap(ax, source=cx.providers.CartoDB.DarkMatter)
    elif style == 'chinese':
        chinese_url = "http://map.geoq.cn/ArcGIS/rest/services/ChinaOnlineCommunity/MapServer/tile/{z}/{y}/{x}"
        cx.add_basemap(ax, source=chinese_url)

    title_text = f"灾后用户分布 (已修正坐标偏差)"
    ax.set_title(title_text, fontsize=15)
    ax.set_axis_off()

    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"保存成功: {output_path}")


# --- 执行 ---
# 建议先用 n=100 测试一下对齐情况
plot_map_corrected('../data/generated_user_loc/371721.csv', 'luliang_5000.png', style='light', n=5000)