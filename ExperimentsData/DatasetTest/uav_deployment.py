import os
import pandas as pd
import numpy as np
from math import radians, cos, sin, sqrt, atan2
import glob


class UAVDeployment:
    def __init__(self, input_dir, output_dir):
        """
        初始化无人机部署系统

        参数:
            input_dir: 输入文件目录
            output_dir: 输出文件目录
        """
        self.input_dir = input_dir
        self.output_dir = output_dir
        self.area_size = 5000  # 区域边长 (米)
        self.grid_size = 100  # 网格边长 (米)
        self.uav_height = 300  # 无人机高度 (米)
        self.coverage_radius = 600  # 覆盖半径 (米)
        self.connection_distance = 800  # 连接距离 (米)
        self.num_uavs = 15  # 无人机数量
        self.capacity = 5000 # 无人机覆盖能力
        self.bandwidth = 40 # 无人机带宽

    def latlon_to_meters(self, lat, lon, ref_lat, ref_lon):
        """
        将经纬度转换为以参考点为原点的米制坐标

        参数:
            lat, lon: 待转换的纬度和经度
            ref_lat, ref_lon: 参考点的纬度和经度
        返回:
            (x, y): 米制坐标
        """
        R = 6371000  # 地球半径(米)

        lat_rad = radians(lat)
        lon_rad = radians(lon)
        ref_lat_rad = radians(ref_lat)
        ref_lon_rad = radians(ref_lon)

        # 计算x坐标 (东西方向)
        x = R * (lon_rad - ref_lon_rad) * cos(ref_lat_rad)

        # 计算y坐标 (南北方向)
        y = R * (lat_rad - ref_lat_rad)

        return x, y

    def meters_to_latlon(self, x, y, ref_lat, ref_lon):
        """
        将米制坐标转换回经纬度

        参数:
            x, y: 米制坐标
            ref_lat, ref_lon: 参考点的纬度和经度
        返回:
            (lat, lon): 纬度和经度
        """
        R = 6371000  # 地球半径(米)

        ref_lat_rad = radians(ref_lat)
        ref_lon_rad = radians(ref_lon)

        # 转换回经纬度
        lat_rad = ref_lat_rad + (y / R)
        lon_rad = ref_lon_rad + (x / (R * cos(ref_lat_rad)))

        lat = np.degrees(lat_rad)
        lon = np.degrees(lon_rad)

        return lat, lon

    def generate_candidate_points(self):
        """
        生成候选部署点(网格中心点)

        返回:
            candidate_points: 候选点坐标列表 [(x, y), ...]
        """
        candidate_points = []

        # 从网格边长的一半开始,每隔grid_size放置一个候选点
        for x in np.arange(self.grid_size / 2, self.area_size, self.grid_size):
            for y in np.arange(self.grid_size / 2, self.area_size, self.grid_size):
                candidate_points.append((x, y))

        return candidate_points

    def count_covered_users(self, uav_pos, user_positions, covered_users):
        """
        计算无人机在指定位置能覆盖的新用户数量

        参数:
            uav_pos: 无人机位置 (x, y)
            user_positions: 所有用户位置列表 [(x, y), ...]
            covered_users: 已覆盖用户的集合
        返回:
            新覆盖的用户数量
        """
        new_covered = 0
        for i, user_pos in enumerate(user_positions):
            if i not in covered_users:
                # 计算用户到无人机的水平距离
                distance = sqrt((uav_pos[0] - user_pos[0]) ** 2 +
                                (uav_pos[1] - user_pos[1]) ** 2)
                if distance <= self.coverage_radius:
                    new_covered += 1
        return new_covered

    def is_connected(self, new_uav_pos, deployed_uavs):
        """
        检查新无人机位置是否与已部署的无人机网络连通

        参数:
            new_uav_pos: 新无人机位置 (x, y)
            deployed_uavs: 已部署的无人机位置列表
        返回:
            True如果连通,否则False
        """
        if len(deployed_uavs) == 0:
            return True

        for uav_pos in deployed_uavs:
            distance = sqrt((new_uav_pos[0] - uav_pos[0]) ** 2 +
                            (new_uav_pos[1] - uav_pos[1]) ** 2)
            if distance <= self.connection_distance:
                return True
        return False

    from math import sqrt

    def get_newly_covered_users(self, uav_pos, user_positions, covered_users):
        """
        获取无人机新覆盖的用户索引集合
        （优先选择距离最近的用户，且受限于 UAV 容量）

        参数:
            uav_pos: 无人机位置 (x, y)
            user_positions: 所有用户位置列表
            covered_users: 已覆盖用户的集合
        返回:
            新覆盖用户的索引集合
        """
        # 1. 找出所有在半径范围内且尚未被覆盖的"候选用户"
        candidates = []

        for i, user_pos in enumerate(user_positions):
            if i not in covered_users:
                distance = sqrt((uav_pos[0] - user_pos[0]) ** 2 +
                                (uav_pos[1] - user_pos[1]) ** 2)

                # 只有在覆盖半径内的用户才会被考虑
                if distance <= self.coverage_radius:
                    # 将 (距离, 索引) 作为元组存入列表，方便后续排序
                    candidates.append((distance, i))

        # 2. 按照距离从小到大排序
        # Python 的 sort 默认对元组的第一个元素（即 distance）进行排序
        candidates.sort(key=lambda x: x[0])

        # 3. 截取前 self.capacity 个用户
        # 如果候选人数少于容量，切片操作会自动取所有候选人
        selected_candidates = candidates[:self.capacity]

        # 4. 提取用户索引并转为集合返回
        newly_covered = {idx for _, idx in selected_candidates}

        return newly_covered

    def deploy_uavs(self, user_positions):
        """
        使用贪心算法部署无人机

        参数:
            user_positions: 用户位置列表 [(x, y), ...]
        返回:
            deployed_uavs: 部署的无人机位置列表
        """
        candidate_points = self.generate_candidate_points()
        deployed_uavs = []
        covered_users = set()

        # print(f"候选部署点数量: {len(candidate_points)}")
        # print(f"用户数量: {len(user_positions)}")

        for uav_count in range(self.num_uavs):
            best_pos = None
            best_coverage = 0

            for candidate_pos in candidate_points:
                # 检查是否已被使用
                if candidate_pos in deployed_uavs:
                    continue

                # 检查连通性
                if not self.is_connected(candidate_pos, deployed_uavs):
                    continue

                # 计算覆盖的新用户数
                new_coverage = self.count_covered_users(candidate_pos, user_positions, covered_users)

                if new_coverage > best_coverage:
                    best_coverage = new_coverage
                    best_pos = candidate_pos

            if best_pos is None:
                print(f"警告: 无法部署第 {uav_count + 1} 架无人机 (没有满足连通性约束的候选点)")
                break

            deployed_uavs.append(best_pos)

            # 更新已覆盖用户
            newly_covered = self.get_newly_covered_users(best_pos, user_positions, covered_users)
            covered_users.update(newly_covered)

            # print(f"部署无人机 {uav_count + 1}: 位置 {best_pos}, 新覆盖用户数: {best_coverage}, "
            #       f"累计覆盖用户数: {len(covered_users)}")

        return deployed_uavs

    def process_file(self, input_file):
        """
        处理单个输入文件

        参数:
            input_file: 输入文件路径
        """
        print(f"\n处理文件: {input_file}")

        # 读取CSV文件
        df = pd.read_csv(input_file)

        # 提取经纬度
        latitudes = df['latitude'].values
        longitudes = df['longitude'].values

        # 使用第一个用户位置作为参考点
        ref_lat = latitudes[0]
        ref_lon = longitudes[0]

        # 将所有用户坐标转换为米制坐标
        user_positions = []
        for lat, lon in zip(latitudes, longitudes):
            x, y = self.latlon_to_meters(lat, lon, ref_lat, ref_lon)
            user_positions.append((x, y))

        # 将坐标归一化到5000m x 5000m区域
        # 找到最小和最大坐标
        x_coords = [pos[0] for pos in user_positions]
        y_coords = [pos[1] for pos in user_positions]

        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        # 归一化到 [0, 5000]
        normalized_positions = []
        for x, y in user_positions:
            norm_x = (x - min_x) / (max_x - min_x) * self.area_size if max_x != min_x else self.area_size / 2
            norm_y = (y - min_y) / (max_y - min_y) * self.area_size if max_y != min_y else self.area_size / 2
            normalized_positions.append((norm_x, norm_y))

        # 部署无人机
        deployed_uavs = self.deploy_uavs(normalized_positions)

        # 将无人机坐标转换回经纬度
        uav_latlons = []
        for uav_x, uav_y in deployed_uavs:
            # 反归一化
            orig_x = uav_x / self.area_size * (max_x - min_x) + min_x if max_x != min_x else 0
            orig_y = uav_y / self.area_size * (max_y - min_y) + min_y if max_y != min_y else 0

            # 转换回经纬度
            lat, lon = self.meters_to_latlon(orig_x, orig_y, ref_lat, ref_lon)
            uav_latlons.append((lat, lon))

        # 生成输出文件名
        input_filename = os.path.basename(input_file)
        # 解析文件名: "序号_用户数users_data_数据源id.csv"
        parts = input_filename.split('_')
        file_id = parts[0]
        data_source_id = parts[-1].replace('.csv', '')
        output_filename = f"{file_id}_{self.num_uavs}uavs_loc_{data_source_id}.csv"
        output_file = os.path.join(self.output_dir, output_filename)

        # 创建输出DataFrame
        output_data = {
            'uav_id': list(range(1, len(uav_latlons) + 1)),
            'longitude': [lon for lat, lon in uav_latlons],
            'latitude': [lat for lat, lon in uav_latlons],
            'bandwidth': self.bandwidth
        }
        output_df = pd.DataFrame(output_data)

        # 保存到CSV
        output_df.to_csv(output_file, index=False)
        print(f"输出文件已保存: {output_file}")
        print(f"成功部署 {len(deployed_uavs)} 架无人机")

    def process_all_files(self):
        """
        处理目录中的所有CSV文件
        """
        # 获取所有CSV文件
        pattern = os.path.join(self.input_dir, "*users_data*.csv")
        input_files = glob.glob(pattern)

        if len(input_files) == 0:
            print(f"在目录 {self.input_dir} 中没有找到匹配的CSV文件")
            return

        print(f"找到 {len(input_files)} 个输入文件")

        # 按遍历顺序处理每个文件
        for input_file in input_files:
            try:
                self.process_file(input_file)
                # break
            except Exception as e:
                print(f"处理文件 {input_file} 时出错: {str(e)}")
                import traceback
                traceback.print_exc()


def main():
    user_nums = [1000, 2000, 3000, 4000, 5000]
    # user_nums = [1000]
    DATA_DIR = r"E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\data\variable_user_num"
    for u_num in user_nums:
        # 设置输入输出目录
        input_data_dir = DATA_DIR + fr"\{u_num}u_num\user_data"
        output_data_dir = DATA_DIR + fr"\{u_num}u_num\uav_data"
        print(input_data_dir)
        print(output_data_dir)
        if not os.path.exists(output_data_dir):
            # os.makedirs 可以递归创建多级目录
            os.makedirs(output_data_dir)
        # 创建UAV部署对象
        deployment = UAVDeployment(input_data_dir, output_data_dir)

        # 处理所有文件
        deployment.process_all_files()

        print("\n所有文件处理完成!")


if __name__ == "__main__":
    main()