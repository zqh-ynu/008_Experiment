"""
UAV Bandwidth Allocation System - System Model
系统模型

包含用户和UAV的管理，以及它们之间的信道参数矩阵
"""
import sys
# 插入SystemModel目录到搜索路径最前面（对应错误日志里的目录）
sys.path.insert(0, r'../SystemModel')

import pandas as pd
from SystemModel.entity_definition import *
from SystemModel import sys_config
import numpy as np

class SystemMd:
    """
    系统模型类
    管理所有用户和UAV，维护它们之间的信道参数矩阵
    """

    def __init__(self, users: Optional[List[User]] = None,
                 uavs: Optional[List[Uav]] = None,
                 user_file: Optional[str] = None,
                 uav_file: Optional[str] = None,
                 config_file: Optional[str] = None):
        """
        初始化SystemMd对象

        有两种初始化方式：
        1. 直接提供users和uavs列表
        2. 提供CSV文件路径和配置文件路径

        Args:
            users: 用户对象列表
            uavs: UAV对象列表
            user_file: 用户CSV文件路径
            uav_file: UAV CSV文件路径
            config_file: 配置JSON文件路径
        """
        # 系统统计变量
        self.m = 0      # 无人机数量
        self.n1 = 0     # 离散效用用户数量（硬性QoS用户）
        self.n2 = 0     # 连续效用用户数量（弹性用户）
        self.max_user_utility = 0.0  # 系统中用户的最大效用值（权重）

        # 用户和UAV列表
        self.users: List[User] = []
        self.uavs: List[Uav] = []

        # 信道参数矩阵 (m x n, 其中 n = n1 + n2)
        self.dis_list: np.ndarray = None       # 距离矩阵
        self.SNRave_list: np.ndarray = None    # 平均信噪比矩阵
        self.SNRth_list: np.ndarray = None     # 信噪比阈值矩阵
        self.M_list: np.ndarray = None         # Nakagami-M参数矩阵
        self.cap_list: np.ndarray = None       # 信道容量矩阵
        self.Bth_list: np.ndarray = None       # 硬性用户最小带宽需求矩阵 (m x n1)

        # 根据不同的初始化方式
        if user_file is not None and uav_file is not None:
            # 从文件加载
            if config_file is not None:
                sys_config.load_global_channel_config(config_file)
            self._load_from_files(user_file, uav_file)
        elif users is not None and uavs is not None:
            # 从列表初始化
            self.users = users
            self.uavs = uavs
            self._update_statistics()
            self.init_system_model()
        else:
            # 空初始化
            pass

    def _load_from_files(self, user_file: str, uav_file: str):
        """
        从CSV文件加载用户和UAV数据

        Args:
            user_file: 用户CSV文件路径
            uav_file: UAV CSV文件路径
        """
        print(f"Loading data from files...")
        print(f"  User file: {user_file}")
        print(f"  UAV file: {uav_file}")

        # 读取CSV文件
        try:
            user_df = pd.read_csv(user_file)
            uav_df = pd.read_csv(uav_file)
        except Exception as e:
            print(f"Error reading CSV files: {e}")
            raise

            # 【新增】如果是经纬度坐标，计算全局原点
        has_lonlat_user = 'longitude' in user_df.columns
        has_lonlat_uav = 'longitude' in uav_df.columns

        if has_lonlat_user and has_lonlat_uav:
            # 合并所有经纬度找到全局最小值
            all_lons = pd.concat([user_df['longitude'], uav_df['longitude']])
            all_lats = pd.concat([user_df['latitude'], uav_df['latitude']])
            self.min_lon = all_lons.min()
            self.min_lat = all_lats.min()
            print(f"Global Origin: ({self.min_lon:.6f}, {self.min_lat:.6f})")

        # 解析用户数据
        self._parse_users(user_df)

        # 解析UAV数据
        self._parse_uavs(uav_df)

        # 更新统计信息
        self._update_statistics()

        # 初始化信道矩阵
        self.init_system_model()

        print(f"System initialized from files.")
        print(f"Users: {len(self.users)} (Hard: {self.n1}, Elastic: {self.n2})")
        print(f"UAVs: {len(self.uavs)}")

    def _parse_users(self, user_df: pd.DataFrame):
        """
        解析用户DataFrame，创建User对象

        用户CSV格式：
        - id: 用户ID
        - type: 用户类型 (1=HARD, 2=ELASTIC)
        - weight: 权重
        - x, y, z: 坐标 (或 lon, lat)
        - user_requirement_1: 最小速率要求 (Mbps)
        - req2: 中断概率 (对于HARD用户) 或其他参数

        Args:
            user_df: 用户数据的DataFrame
        """
        self.users = []

        # 检查是否需要坐标转换（经纬度 -> 米）
        has_lonlat = 'longitude' in user_df.columns and 'latitude' in user_df.columns
        has_lonlat_short = 'lon' in user_df.columns and 'lat' in user_df.columns
        has_xyz = 'x' in user_df.columns and 'y' in user_df.columns

        if has_lonlat or has_lonlat_short:
            # 需要经纬度转换
            self.users = self._parse_users_with_lonlat(user_df)
        elif has_xyz:
            # 直接使用XYZ坐标
            self.users = self._parse_users_with_xyz(user_df)
        else:
            raise ValueError("User CSV must have either (longitude, latitude) or (lon, lat) or (x, y) columns")

    def _parse_users_with_xyz(self, user_df: pd.DataFrame) -> List[User]:
        """解析使用XYZ坐标的用户数据"""
        users = []

        # 分别收集hard和elastic用户
        hard_users = []
        elastic_users = []

        for idx, row in user_df.iterrows():
            # 用户类型：支持多种列名格式
            if 'user_type' in row:
                utype_str = str(row['user_type']).lower()
                utype = HARD_UTILITY if utype_str == 'hard' else ELASTIC_UTILITY
            elif 'type' in row:
                utype = int(row['type'])
            else:
                utype = HARD_UTILITY

            # 用户权重
            if 'user_weight' in row:
                weight = float(row['user_weight'])
            elif 'weight' in row:
                weight = float(row['weight'])
            else:
                weight = 1.0

            x = float(row.get('x', 0.0))
            y = float(row.get('y', 0.0))
            z = float(row.get('z', 0.0))

            # 最小速率要求（user_requirement_1）
            # 注意：需要从Kbps转换为Mbps
            if 'user_requirement_1' in row:
                r_min_kbps = float(row['user_requirement_1'])
                r_min = r_min_kbps / 1000.0  # Kbps -> Mbps
            elif 'r_min' in row:
                r_min = float(row['r_min'])
            else:
                r_min = 1.0

            # 中断概率（user_requirement_2，仅对hard用户有意义）
            if utype == HARD_UTILITY:
                # Hard用户：从user_requirement_2读取中断概率
                if 'user_requirement_2' in row:
                    p_out = float(row['user_requirement_2'])
                elif 'req2' in row:
                    p_out = float(row['req2'])
                elif 'p_out' in row:
                    p_out = float(row['p_out'])
                else:
                    p_out = 1e-3  # 默认值
            else:
                # Elastic用户：user_requirement_2不表示中断概率，使用默认值
                p_out = 1e-2  # Elastic用户的默认中断概率

            # 创建用户对象（忽略文件中的ID，后面统一重新编号）
            if utype == HARD_UTILITY:
                # 硬性用户权重乘以10（与C++代码保持一致）
                user = User(0, utype, weight, x, y, z, r_min, p_out)
                hard_users.append(user)
            else:
                user = User(0, utype, weight, x, y, z, r_min, p_out)
                elastic_users.append(user)

        # 按照hard用户在前，elastic用户在后的顺序合并
        users = hard_users + elastic_users

        # 重新设置ID（从0开始，确保连续）
        for new_id, user in enumerate(users):
            user.ID = new_id

        return users

    def _parse_users_with_lonlat(self, user_df: pd.DataFrame) -> List[User]:
        """
        解析使用经纬度坐标的用户数据，并转换为米制坐标

        Args:
            user_df: 用户数据的DataFrame

        Returns:
            用户对象列表
        """
        users = []

        # 找到最小经纬度作为原点
        min_lon = self.min_lon
        min_lat = self.min_lat

        # 计算转换系数（经纬度 -> 米）
        lat_to_meter = sys_config.EARTH_RADIUS * math.pi / 180.0
        avg_lat = user_df['latitude'].mean()
        lon_to_meter = sys_config.EARTH_RADIUS * math.pi / 180.0 * math.cos(min_lat * math.pi / 180.0)

        # 分别收集hard和elastic用户
        hard_users = []
        elastic_users = []

        for idx, row in user_df.iterrows():
            # 用户类型：支持多种列名格式
            if 'user_type' in row:
                utype_str = str(row['user_type']).lower()
                utype = HARD_UTILITY if utype_str == 'hard' else ELASTIC_UTILITY
            elif 'type' in row:
                utype = int(row['type'])
            else:
                utype = HARD_UTILITY

            # 用户权重
            if 'user_weight' in row:
                weight = float(row['user_weight'])
            elif 'weight' in row:
                weight = float(row['weight'])
            else:
                weight = 1.0

            # 经纬度转换为相对坐标（米）
            lon = float(row['longitude'])
            lat = float(row['latitude'])
            x = (lon - min_lon) * lon_to_meter
            y = (lat - min_lat) * lat_to_meter
            z = float(row.get('z', 0.0))

            # 最小速率要求（user_requirement_1）
            # 注意：需要从Kbps转换为Mbps
            if 'user_requirement_1' in row:
                r_min_kbps = float(row['user_requirement_1'])
                r_min = r_min_kbps / 1000.0  # Kbps -> Mbps
            elif 'r_min' in row:
                r_min = float(row['r_min'])
            else:
                r_min = 1.0

            # 中断概率（user_requirement_2，仅对hard用户有意义）
            if utype == HARD_UTILITY:
                # Hard用户：从user_requirement_2读取中断概率
                if 'user_requirement_2' in row:
                    p_out = float(row['user_requirement_2'])
                elif 'req2' in row:
                    p_out = float(row['req2'])
                elif 'p_out' in row:
                    p_out = float(row['p_out'])
                else:
                    p_out = 1e-3  # 默认值
            else:
                # Elastic用户：user_requirement_2不表示中断概率，使用默认值
                p_out = 1e-2  # Elastic用户的默认中断概率

            # 创建用户对象（忽略文件中的ID，后面统一重新编号）
            if utype == HARD_UTILITY:
                user = User(0, utype, weight, x, y, z, r_min, p_out)
                hard_users.append(user)
            else:
                user = User(0, utype, weight, x, y, z, r_min, p_out)
                elastic_users.append(user)

        # 合并并重新编号（从0开始）
        users = hard_users + elastic_users
        for new_id, user in enumerate(users):
            user.ID = new_id

        return users

    def _parse_uavs(self, uav_df: pd.DataFrame):
        """
        解析UAV DataFrame，创建Uav对象

        UAV CSV格式：
        - id: UAV ID
        - x, y, z: 坐标 (或 lon, lat, altitude)
        - bandwidth: 总带宽 (MHz)

        Args:
            uav_df: UAV数据的DataFrame
        """
        self.uavs = []

        # 检查坐标格式
        has_lonlat = 'longitude' in uav_df.columns and 'latitude' in uav_df.columns
        has_lonlat_short = 'lon' in uav_df.columns and 'lat' in uav_df.columns
        has_xyz = 'x' in uav_df.columns and 'y' in uav_df.columns

        if has_lonlat or has_lonlat_short:
            self.uavs = self._parse_uavs_with_lonlat(uav_df)
        elif has_xyz:
            self.uavs = self._parse_uavs_with_xyz(uav_df)
        else:
            raise ValueError("UAV CSV must have either (longitude, latitude) or (lon, lat) or (x, y) columns")

    def _parse_uavs_with_xyz(self, uav_df: pd.DataFrame) -> List[Uav]:
        """解析使用XYZ坐标的UAV数据"""
        uavs = []

        for idx, row in uav_df.iterrows():
            uav_id = int(row.get('id', idx))

            x = float(row.get('x', 0.0))
            y = float(row.get('y', 0.0))

            # Z坐标：优先使用altitude，否则使用z，最后使用config中的默认高度
            if 'altitude' in row:
                z = float(row['altitude'])
            elif 'z' in row:
                z = float(row['z'])
            else:
                z = sys_config.uav_alt

            # 带宽
            bandwidth = float(row.get('bandwidth', 20.0))

            # 创建UAV对象
            uav = Uav(uav_id, x, y, z, bandwidth, sys_config.uav_trans_power)
            uavs.append(uav)

        return uavs

    def _parse_uavs_with_lonlat(self, uav_df: pd.DataFrame) -> List[Uav]:
        """解析使用经纬度坐标的UAV数据，并转换为米制坐标"""
        uavs = []

        # 确定经纬度列名
        if 'longitude' in uav_df.columns:
            lon_col, lat_col = 'longitude', 'latitude'
        else:
            lon_col, lat_col = 'lon', 'lat'

        # 使用与用户相同的原点（需要从用户数据获取）
        # 这里假设已经计算过，或者重新计算
        if len(self.users) > 0:
            # 如果已经有用户数据，从第一个用户推算原点
            # 这是一个简化方案，实际应该保存原点信息
            print("Warning: UAV coordinate conversion may be inconsistent with users")

        # 找到最小经纬度
        min_lon = self.min_lon
        min_lat = self.min_lat

        # 计算转换系数
        lat_to_meter = sys_config.EARTH_RADIUS * math.pi / 180.0
        avg_lat = uav_df[lat_col].mean()
        lon_to_meter = sys_config.EARTH_RADIUS * math.pi / 180.0 * math.cos(min_lat * math.pi / 180.0)

        for idx, row in uav_df.iterrows():
            # 忽略文件中的ID，按顺序重新编号
            uav_id = idx

            lon = float(row[lon_col])
            lat = float(row[lat_col])
            x = (lon - min_lon) * lon_to_meter
            y = (lat - min_lat) * lat_to_meter

            # 高度
            if 'altitude' in row:
                z = float(row['altitude'])
            else:
                z = sys_config.uav_alt

            # 带宽
            bandwidth = float(row.get('bandwidth', 20.0))

            # 创建UAV对象
            uav = Uav(uav_id, x, y, z, bandwidth, sys_config.uav_trans_power)
            uavs.append(uav)

        # 重新编号（从0开始）
        for new_id, uav in enumerate(uavs):
            uav.ID = new_id

        return uavs

    def _update_statistics(self):
        """更新系统统计信息"""
        self.n1 = sum(1 for u in self.users if u.uType == HARD_UTILITY)
        self.n2 = sum(1 for u in self.users if u.uType == ELASTIC_UTILITY)
        self.m = len(self.uavs)

        # 计算最大用户权重
        if len(self.users) > 0:
            self.max_user_utility = max(u.weight for u in self.users)
        else:
            self.max_user_utility = 0.0

    def init_system_model(self):
        """
        初始化系统模型
        计算所有UAV与用户之间的信道参数矩阵
        """
        n_total = self.n1 + self.n2

        # 初始化矩阵
        self.dis_list = np.zeros((self.m, n_total))
        self.SNRave_list = np.zeros((self.m, n_total))
        self.SNRth_list = np.zeros((self.m, n_total))
        self.M_list = np.zeros((self.m, n_total))
        self.cap_list = np.zeros((self.m, n_total))
        self.Bth_list = np.zeros((self.m, self.n1))

        print(f"Initializing channel matrices ({self.m} UAVs x {n_total} users)...")

        # 计算每对UAV-用户的信道参数
        for i in range(self.m):
            for j in range(n_total):
                ch = Channel(self.uavs[i], self.users[j])

                self.dis_list[i, j] = ch.d
                self.SNRave_list[i, j] = ch.SNRa_dB
                self.SNRth_list[i, j] = ch.SNRt_dB
                self.M_list[i, j] = ch.M
                self.cap_list[i, j] = ch.channel_capacity

        # 计算硬性用户的最小带宽需求
        for i in range(self.m):
            for j in range(self.n1):
                cap = self.cap_list[i, j]
                if cap > EPS:
                    self.Bth_list[i, j] = self.users[j].rMin / cap
                else:
                    self.Bth_list[i, j] = INFINITE

        print("System model initialized successfully.")

    def print_system_info(self, n: int = 100):
        """
        打印系统信息

        Args:
            n: 打印前n个用户的信息，-1表示全部
        """
        if n < 0 or n > self.n1 + self.n2:
            n = self.n1 + self.n2

        print("=" * 80)
        print("System Model Information")
        print("=" * 80)

        self.print_all_uavs()
        self.print_all_users(n)
        self.print_dis_list(n)
        self.print_SNRa_list(n)
        self.print_SNRt_list(n)
        self.print_M_list(n)
        self.print_cap_list(n)
        self.print_min_bw_list(min(n, self.n1))

    def print_all_users(self, n: int = 100):
        """打印用户信息"""
        print("\n" + "=" * 80)
        print(f"All Users (showing first {min(n, len(self.users))})")
        print("=" * 80)

        for i, user in enumerate(self.users[:n]):
            user.print_user()

    def print_all_uavs(self):
        """打印UAV信息"""
        print("\n" + "=" * 80)
        print("All UAVs")
        print("=" * 80)

        for uav in self.uavs:
            uav.print_UAV()

    def print_dis_list(self, n: int = 100):
        """打印距离矩阵"""
        print("\n" + "=" * 80)
        print(f"Distance Matrix (m) - First {min(n, self.n1 + self.n2)} users")
        print("=" * 80)

        for i in range(self.m):
            print(f"UAV {i}: ", end="")
            for j in range(min(n, self.n1 + self.n2)):
                print(f"{self.dis_list[i, j]:8.2f} ", end="")
                if (j + 1) % 10 == 0:
                    print()
                    print(" " * 8, end="")
            print()

    def print_SNRa_list(self, n: int = 100):
        """打印平均信噪比矩阵"""
        print("\n" + "=" * 80)
        print(f"Average SNR Matrix (dB) - First {min(n, self.n1 + self.n2)} users")
        print("=" * 80)

        for i in range(self.m):
            print(f"UAV {i}: ", end="")
            for j in range(min(n, self.n1 + self.n2)):
                print(f"{self.SNRave_list[i, j]:8.2f} ", end="")
                if (j + 1) % 10 == 0:
                    print()
                    print(" " * 8, end="")
            print()

    def print_SNRt_list(self, n: int = 100):
        """打印信噪比阈值矩阵"""
        print("\n" + "=" * 80)
        print(f"SNR Threshold Matrix (dB) - First {min(n, self.n1 + self.n2)} users")
        print("=" * 80)

        for i in range(self.m):
            print(f"UAV {i}: ", end="")
            for j in range(min(n, self.n1 + self.n2)):
                print(f"{self.SNRth_list[i, j]:8.2f} ", end="")
                if (j + 1) % 10 == 0:
                    print()
                    print(" " * 8, end="")
            print()

    def print_M_list(self, n: int = 100):
        """打印Nakagami-M参数矩阵"""
        print("\n" + "=" * 80)
        print(f"Nakagami-M Parameter Matrix - First {min(n, self.n1 + self.n2)} users")
        print("=" * 80)

        for i in range(self.m):
            print(f"UAV {i}: ", end="")
            for j in range(min(n, self.n1 + self.n2)):
                print(f"{self.M_list[i, j]:8.4f} ", end="")
                if (j + 1) % 10 == 0:
                    print()
                    print(" " * 8, end="")
            print()

    def print_cap_list(self, n: int = 100):
        """打印信道容量矩阵"""
        print("\n" + "=" * 80)
        print(f"Channel Capacity Matrix (bit/s/Hz) - First {min(n, self.n1 + self.n2)} users")
        print("=" * 80)

        for i in range(self.m):
            print(f"UAV {i}: ", end="")
            for j in range(min(n, self.n1 + self.n2)):
                print(f"{self.cap_list[i, j]:8.4f} ", end="")
                if (j + 1) % 10 == 0:
                    print()
                    print(" " * 8, end="")
            print()

    def print_min_bw_list(self, n: int = 100):
        """打印硬性用户最小带宽需求矩阵"""
        if n > self.n1:
            n = self.n1

        print("\n" + "=" * 80)
        print(f"Minimum Bandwidth Requirement Matrix (MHz) - First {n} hard users")
        print("=" * 80)

        for i in range(self.m):
            print(f"UAV {i}: ", end="")
            for j in range(n):
                if self.Bth_list[i, j] == INFINITE:
                    print(f"     INF ", end="")
                else:
                    print(f"{self.Bth_list[i, j]:8.4f} ", end="")
                if (j + 1) % 10 == 0:
                    print()
                    print(" " * 8, end="")
            print()

    def __repr__(self):
        return (f"SystemMd(UAVs={self.m}, Users={self.n1+self.n2} "
                f"[Hard={self.n1}, Elastic={self.n2}])")