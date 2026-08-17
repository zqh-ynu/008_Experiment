# 该文件
import rasterio
import numpy as np
import pandas as pd
from pytz import country_names
from rasterio.windows import from_bounds
import os

def generate_disaster_users(tif_path, center_lat, center_lon, side_km, n, output_file):
    """
    根据人员密度栅格文件生成用户坐标
    """
    with rasterio.open(tif_path) as src:
        # 1. 计算 5km 矩形区域的地理边界 (WGS84 粗略估计)
        # 纬度 1度约 111km，经度 1度在玉树(33°N)约 93km
        delta_lat = (side_km / 2) / 111.0
        delta_lon = (side_km / 2) / (111.0 * np.cos(np.radians(center_lat)))

        west, south = center_lon - delta_lon, center_lat - delta_lat
        east, north = center_lon + delta_lon, center_lat + delta_lat

        # 2. 从 .tif 中读取该窗口的数据
        window = from_bounds(west, south, east, north, src.transform)
        # 使用 masked=True 自动处理无数据区域
        data = src.read(1, window=window)
        win_transform = src.window_transform(window)

        # 数据清洗：处理负值或 NaN
        data = np.nan_to_num(data, nan=0.0)
        data[data < 0] = 0

        if data.sum() <= 0:
            raise ValueError("选定区域内没有有效的人口密度数据（密度总和为0）。")

        # 3. 概率加权抽样
        flat_data = data.flatten()
        probabilities = flat_data / flat_data.sum()

        # 随机抽取 n 个像素索引
        pixel_indices = np.random.choice(len(flat_data), size=n, p=probabilities)
        rows, cols = np.unravel_index(pixel_indices, data.shape)

        # 4. 转换回经纬度坐标并添加 100m 随机扰动
        # 栅格分辨率为 100m，即约 0.0009 度
        lons, lats = rasterio.transform.xy(win_transform, rows, cols)

        res_lat = 100 / 111000
        res_lon = 100 / (111000 * np.cos(np.radians(center_lat)))

        # 在 100m 栅格内均匀随机分布
        final_lats = np.array(lats) + (np.random.rand(n) - 0.5) * res_lat
        final_lons = np.array(lons) + (np.random.rand(n) - 0.5) * res_lon

        # 5. 保存结果
        df = pd.DataFrame({
            'user_id': range(1, n + 1),
            'latitude': final_lats,
            'longitude': final_lons
        })

        df.to_csv(output_file, index=False)
        print(f"成功生成 {n} 条数据并保存至 {output_file}")


def generate_disaster_users_csv(side_km = 5, num_users=5000):
    """
    遍历区县中心点，根据对应的 .tif 文件生成用户坐标 CSV。
    """
    # 路径配置
    coords_path = '../data/china_counties_coords.csv'
    tif_dir = '../data/cut_maps_tif/'
    output_base_dir = '../data/generated_user_loc/'

    # 确保输出目录存在
    if not os.path.exists(output_base_dir):
        os.makedirs(output_base_dir)

    # 1. 使用 pandas 读取，自动处理表头和列索引
    try:
        df = pd.read_csv(coords_path)
    except Exception as e:
        print(f"读取坐标文件失败: {e}")
        return

    # 2. 遍历每一行（每一个区县）
    for index, row in df.iterrows():
        try:
            # 根据你的表头提取数据
            county_name = str(row['县区'])
            county_id = str(row['行政代码'])
            center_lat = row['纬度']
            center_lon = row['经度']

            # 构建文件路径
            tif_file_path = os.path.join(tif_dir, f"{county_id}_{county_name}.tif")
            output_path = os.path.join(output_base_dir, f"{county_id}.csv")

            # 3. 检查 TIF 文件是否存在，不存在则跳过
            if not os.path.exists(tif_file_path):
                print(f"跳过：未找到 TIF 文件 {tif_file_path}")
                continue

            print(f"正在处理 [{county_id}] {county_name}...")

            # 4. 调用生成函数
            generate_disaster_users(
                tif_file_path,
                center_lat,
                center_lon,
                side_km,
                num_users,
                output_path
            )

        except Exception as e:
            print(f"处理第 {index} 行数据时出错: {e}")


# 执行 (请确保当前目录下有对应的 tif 文件)
# 使用示例
if __name__ == "__main__":
    # 示例调用参数
    generate_disaster_users_csv()