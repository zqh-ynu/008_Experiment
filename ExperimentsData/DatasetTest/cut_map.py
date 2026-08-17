# 将../data/chn_pop_2025_CN_100m_R2025A_v1.tif 文件按照'../data/china_counties_coords.csv'中所列的中国县级行政中心，切割为长度为5Km的子图文件。

import os
import pandas as pd
import rasterio
from rasterio.mask import mask
import geopandas as gpd
from shapely.geometry import box, Point


def batch_cut_tif_by_csv(tif_path, csv_path, output_dir="../data/cut_maps_tif"):
    # 1. 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 2. 读取坐标数据
    df = pd.read_csv(csv_path)
    print(f"成功读取 CSV，共包含 {len(df)} 条行政区数据。")

    # 3. 打开全国人口密度 TIF 文件
    with rasterio.open(tif_path) as src:
        src_crs = src.crs
        nodata = src.nodata

        for index, row in df.iterrows():
            adcode = str(row['行政代码'])
            name = row['县区']
            lon = row['经度']
            lat = row['纬度']

            # 4. 精确计算 5km 矩形边界
            # 将 WGS84 坐标的点转为米制投影 (EPSG:3857) 来计算 5km 范围
            point_geom = Point(lon, lat)
            point_gdf = gpd.GeoDataFrame(index=[0], crs="EPSG:4326", geometry=[point_geom])
            point_3857 = point_gdf.to_crs(epsg=3857).geometry.iloc[0]

            half_side = 2500  # 2.5公里，总边长5公里
            # 在投影坐标系下生成 5km x 5km 的矩形框
            bbox_3857 = box(point_3857.x - half_side, point_3857.y - half_side,
                            point_3857.x + half_side, point_3857.y + half_side)

            # 将矩形框转回与栅格一致的坐标系 (通常是 WGS84)
            geo_bbox = gpd.GeoSeries([bbox_3857], crs="EPSG:3857").to_crs(src_crs).geometry.iloc[0]

            try:
                # 5. 执行裁剪 (crop=True 确保输出文件只包含 5km 范围)
                out_image, out_transform = mask(src, [geo_bbox], crop=True)

                # 更新元数据
                out_meta = src.meta.copy()
                out_meta.update({
                    "driver": "GTiff",
                    "height": out_image.shape[1],
                    "width": out_image.shape[2],
                    "transform": out_transform,
                    "nodata": nodata
                })

                # 6. 保存文件：adcode_县名.tif
                file_name = f"{adcode}_{name}.tif"
                output_path = os.path.join(output_dir, file_name)

                with rasterio.open(output_path, "w", **out_meta) as dest:
                    dest.write(out_image)

                if index % 100 == 0:
                    print(f"进度: 已完成 {index}/{len(df)}")

            except ValueError:
                # 如果中心点坐标不在栅格范围内，跳过
                # print(f"跳过: {name} ({adcode}) 坐标超出栅格范围。")
                continue

    print(f"\n裁剪任务全部完成！结果存储在: {output_dir}")

# --- 使用说明 ---
# 1. 确保已安装必要库: pip install rasterio pandas geopandas shapely
# 2. 将 'china_population.tif' 替换为你电脑上实际的文件名
batch_cut_tif_by_csv('data/chn_pop_2025_CN_100m_R2025A_v1.tif', '../data/china_counties_coords.csv')