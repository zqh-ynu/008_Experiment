import osmnx as ox
import rasterio
from rasterio.mask import mask
import json

# 1. 获取玉树市的行政边界矢量 (OSM 坐标系默认为 EPSG:4326)
print("正在获取玉树市边界...")
gdf_boundary = ox.geocode_to_gdf("Yushu City, Qinghai, China")

# 2. 将 GeoDataFrame 的几何图形转换为 rasterio 识别的格式
geoms = gdf_boundary.geometry.values
from shapely.geometry import mapping
shapes = [mapping(g) for g in geoms]

# 3. 打开全中国 WorldPop 原始数据进行裁剪
input_file = "../data/chn_pop_2025_CN_100m_R2025A_v1.tif"
output_file = "../data/yushu_city_pop_2025.tif"

print("正在执行掩模裁剪 (这可能需要一点时间，因为原始文件很大)...")
with rasterio.open(input_file) as src:
    # 使用 mask 函数，crop=True 表示裁剪到边界边缘
    out_image, out_transform = mask(src, shapes, crop=True)
    out_meta = src.meta.copy()

    # 更新元数据，确保新的栅格大小和坐标系匹配裁剪后的区域
    out_meta.update({
        "driver": "GTiff",
        "height": out_image.shape[1],
        "width": out_image.shape[2],
        "transform": out_transform
    })

    # 保存结果
    with rasterio.open(output_file, "w", **out_meta) as dest:
        dest.write(out_image)

print(f"裁剪完成！玉树市人口数据已保存至: {output_file}")