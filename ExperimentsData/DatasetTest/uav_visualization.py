import pandas as pd
import matplotlib.pyplot as plt
import contextily as cx
from matplotlib.patches import Circle
import numpy as np
from math import sqrt


def plot_uav_deployment(user_file, uav_file, output_image=None):
    """
    可视化无人机部署和用户分布

    参数:
        user_file: 用户数据CSV文件路径
        uav_file: 无人机位置CSV文件路径
        output_image: 输出图像文件路径(可选)
    """
    # 读取数据

    users_df = pd.read_csv(user_file)
    uavs_df = pd.read_csv(uav_file)

    print(f"读取用户数据: {len(users_df)} 个用户")
    print(f"读取无人机数据: {len(uavs_df)} 架无人机")

    # 创建图形
    fig, ax = plt.subplots(figsize=(15, 15), dpi=150)

    # 提取用户坐标
    user_lons = users_df['longitude'].values
    user_lats = users_df['latitude'].values

    # 提取无人机坐标
    uav_lons = uavs_df['longitude'].values
    uav_lats = uavs_df['latitude'].values

    # 绘制用户位置(红色小圆点)
    ax.scatter(user_lons, user_lats,
               c='red',
               s=20,
               alpha=0.6,
               zorder=3,
               label='Users')

    # 绘制无人机覆盖范围(浅蓝色半透明圆)
    # 首先需要将米转换为经纬度差值
    # 在中纬度地区,1度纬度约等于111km
    # 经度的距离取决于纬度
    avg_lat = np.mean(uav_lats)

    # 500米对应的纬度差
    coverage_radius_m = 500  # 米
    lat_degree_per_meter = 1 / 111000  # 1米对应的纬度度数
    coverage_radius_lat = coverage_radius_m * lat_degree_per_meter

    # 经度差需要考虑纬度
    lon_degree_per_meter = 1 / (111000 * np.cos(np.radians(avg_lat)))
    coverage_radius_lon = coverage_radius_m * lon_degree_per_meter

    # 绘制覆盖圆
    for lon, lat in zip(uav_lons, uav_lats):
        # 计算该位置的经度度数/米比例
        local_lon_per_meter = 1 / (111000 * np.cos(np.radians(lat)))
        local_coverage_lon = coverage_radius_m * local_lon_per_meter

        circle = Circle((lon, lat),
                        radius=coverage_radius_lat,  # 使用纬度半径作为近似
                        facecolor='lightblue',
                        edgecolor='none',
                        alpha=0.3,
                        zorder=1)
        ax.add_patch(circle)

    # 计算并绘制无人机之间的连接(距离<600m)
    connection_distance_m = 800  # 米

    for i in range(len(uav_lons)):
        for j in range(i + 1, len(uav_lons)):
            # 计算两个无人机之间的距离(使用Haversine公式的简化版本)
            lat1, lon1 = uav_lats[i], uav_lons[i]
            lat2, lon2 = uav_lats[j], uav_lons[j]

            # 转换为米(近似计算)
            dlat = (lat2 - lat1) * 111000
            dlon = (lon2 - lon1) * 111000 * np.cos(np.radians((lat1 + lat2) / 2))
            distance = sqrt(dlat ** 2 + dlon ** 2)

            if distance <= connection_distance_m:
                ax.plot([lon1, lon2], [lat1, lat2],
                        'b-',
                        linewidth=1.5,
                        alpha=0.5,
                        zorder=2)

    # 绘制无人机位置(蓝色五角星)
    ax.scatter(uav_lons, uav_lats,
               marker='*',
               c='blue',
               s=400,
               edgecolors='darkblue',
               linewidths=1.5,
               zorder=4,
               label='UAVs')
    # === 在这里添加 ID 编号 ===
    for i, (lon, lat) in enumerate(zip(uav_lons, uav_lats)):
        # 提取 ID，如果 csv 里有 'uav_id' 列则用 uavs_df['uav_id'].iloc[i]，否则用 i
        uav_id = uavs_df['uav_id'].iloc[i] if 'uav_id' in uavs_df.columns else i
        ax.text(lon, lat, str(uav_id),
                fontsize=12,
                fontweight='bold',
                color='yellow',
                ha='center',  # 水平居中
                va='center',  # 垂直居中
                zorder=5)  # 确保在五角星上方

    # 设置坐标轴
    ax.set_xlabel('Longitude', fontsize=12)
    ax.set_ylabel('Latitude', fontsize=12)
    ax.set_title('UAV Deployment and User Distribution', fontsize=14, fontweight='bold')

    # 添加图例
    ax.legend(loc='upper right', fontsize=10)

    # 添加背景地图
    try:
        cx.add_basemap(ax,
                       crs='EPSG:4326',
                       source=cx.providers.CartoDB.Positron,
                       attribution=False)  # 不显示文字
        print("成功添加背景地图")
    except Exception as e:
        print(f"添加背景地图时出错: {e}")
        print("继续绘制,不使用背景地图")

    # 调整布局
    plt.tight_layout()

    # 保存或显示
    if output_image:
        plt.savefig(output_image, dpi=300, bbox_inches='tight')
        print(f"图像已保存到: {output_image}")
        plt.show()

    return fig, ax


def main():
    user_num = 1000
    # 设置文件路径
    data_dir = fr"E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\data\variable_user_num\{user_num}u_num"

    # 示例:读取第一组文件
    # 您可以修改这些文件名来选择要可视化的文件1_20uavs_loc_230717
    user_file = f"{data_dir}\\user_data\\1_{user_num}users_data_230717.csv"
    uav_file = f"{data_dir}\\uav_data\\1_25uavs_loc_230717.csv"
    output_image = f"{data_dir}\\1_deployment25uav.png"

    # 绘制可视化图
    plot_uav_deployment(user_file, uav_file, output_image)


if __name__ == "__main__":
    main()