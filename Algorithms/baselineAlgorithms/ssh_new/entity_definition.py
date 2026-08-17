"""
UAV Bandwidth Allocation System - Entity Definitions
无人机带宽分配系统 - 实体定义

包含系统中的基础实体类：Point, User, Uav, Channel等
"""
import sys
# 插入SystemModel目录到搜索路径最前面（对应错误日志里的目录）
sys.path.insert(0, r'../SystemModel')

import math
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from scipy import special
from scipy.optimize import brentq
from SystemModel import sys_config

# 常量定义
HARD_UTILITY = 1  # 硬性QoS用户类型
ELASTIC_UTILITY = 2  # 弹性效用用户类型
HALFSOFT_UTILITY = 3  # 半软效用用户类型
INFINITE = 1e9  # 无穷大
EPS = 1e-9  # 浮点数比较误差
BANDWIDTH = 20  # 每个无人机的总带宽 20MHz


@dataclass
class KnapsackResult:
    """
    背包问题（带宽分配）的结果结构
    包含选中的用户列表及其对应的分配值、总价值、总重量等
    """
    allocated_list: List[int] = field(default_factory=list)  # 选中的用户ID列表
    allocated_bandwidth: Dict[int, float] = field(default_factory=dict)  # 用户对应的分配带宽
    allocated_value: Dict[int, float] = field(default_factory=dict)  # 用户对应的价值
    total_value: float = 0.0  # 总价值
    total_weight: float = 0.0  # 总重量
    elastic_value: float = 0.0  # 仅连续部分的总价值
    hard_value: float = 0.0  # 仅离散部分的总价值
    elastic_weight: float = 0.0  # 仅连续部分的总重量
    hard_weight: float = 0.0  # 仅离散部分的总重量

    def clear(self):
        """清空结果"""
        self.allocated_list.clear()
        self.allocated_bandwidth.clear()
        self.allocated_value.clear()
        self.total_value = 0.0
        self.total_weight = 0.0
        self.elastic_value = 0.0
        self.hard_value = 0.0
        self.elastic_weight = 0.0
        self.hard_weight = 0.0


@dataclass
class UserResult:
    """单个用户的分配结果"""
    uav_id: int = -1  # 分配的UAV ID
    allocated_bandwidth: float = 0.0  # 分配的带宽
    utility: float = 0.0  # 用户效用


@dataclass
class KKTParameters:
    """
    KKT条件参数
    用于注水算法和FPTAS算法
    """
    user_id: int = -1  # 用户下标
    W_sum: float = 0.0  # 当前下标之前所有elastic用户的权重之和
    C_inv_sum: float = 0.0  # 当前下标之前所有elastic用户的信道容量倒数之和
    efficient: float = 0.0  # 当前elastic用户在未分配带宽时的边际效用
    B_elastic_sum: float = 0.0  # 当前下标之前所有elastic用户的分配带宽之和
    B_hard_sum: float = 0.0  # 当前下标之前所有hard用户的最小带宽需求之和


class Point:
    """
    三维空间坐标点类
    提供距离计算等基础功能
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, id: int = -1):
        """
        初始化Point对象

        Args:
            x: X坐标
            y: Y坐标
            z: Z坐标
            id: 点的ID
        """
        self.X = x
        self.Y = y
        self.Z = z
        self.ID = id

    @staticmethod
    def cal_distance(s: 'Point', t: 'Point') -> float:
        """
        计算点s与点t之间的直线距离

        Args:
            s: 起点
            t: 终点

        Returns:
            两点之间的欧氏距离
        """
        return math.sqrt((s.X - t.X) ** 2 + (s.Y - t.Y) ** 2 + (s.Z - t.Z) ** 2)

    @staticmethod
    def cal_horizontal_distance(s: 'Point', t: 'Point') -> float:
        """
        计算点s与点t之间的水平距离（忽略Z坐标）

        Args:
            s: 起点
            t: 终点

        Returns:
            两点之间的水平距离
        """
        return math.sqrt((s.X - t.X) ** 2 + (s.Y - t.Y) ** 2)

    def set_location(self, lx: float, ly: float, lz: float):
        """设置坐标位置"""
        self.X = lx
        self.Y = ly
        self.Z = lz

    def get_index(self) -> int:
        """获取索引ID"""
        return self.ID

    def __lt__(self, other: 'Point') -> bool:
        """用于排序比较"""
        return self.ID < other.ID

    def __repr__(self):
        return f"Point(ID={self.ID}, X={self.X:.2f}, Y={self.Y:.2f}, Z={self.Z:.2f})"


class User(Point):
    """
    用户类，继承自Point
    包含用户的通信需求、效用函数等
    """

    def __init__(self, id: int, utype: int, weight: float,
                 x: float, y: float, z: float,
                 r_min: float, p_out: float, r_data: float = 0.0):
        """
        初始化User对象

        Args:
            id: 用户ID
            utype: 用户效用类型 (HARD_UTILITY, ELASTIC_UTILITY, HALFSOFT_UTILITY)
            weight: 用户权重
            x, y, z: 三维坐标
            r_min: 需求的最小数据速率 (Mbps)
            p_out: 需求的最大中断概率
            r_data: 需求的最小数据量（可选）
        """
        super().__init__(x, y, z, id)
        self.uType = utype
        self.weight = weight
        self.rData = r_data
        self.rMin = r_min
        self.pOut = p_out

        # 带宽参数
        self.B = 0.18  # 180 KHz
        self.BSub = 0.18  # 子载波带宽 180 KHz

    def set_communication_requirements(self, r_min: float, p_out: float, r_data: float = 0.0):
        """设置通信需求参数"""
        self.rMin = r_min
        self.pOut = p_out
        if r_data > 0:
            self.rData = r_data

    def hard_utility(self, bandwidth: float = 0.0, capacity: float = 0.0,
                     outage: Optional[float] = None) -> float:
        """
        计算硬性QoS用户的效用

        Args:
            bandwidth: 分配的带宽
            capacity: 信道容量
            outage: 中断概率（如果提供则基于中断概率计算）

        Returns:
            效用值
        """
        if outage is not None:
            # 基于中断概率的效用
            if outage <= self.pOut:
                return self.weight * math.log2(1 + self.rMin)
            else:
                return 0.0
        else:
            # 基于数据速率的效用
            data_rate = bandwidth * capacity
            if data_rate >= self.rMin - EPS:
                return self.weight * math.log2(1 + self.rMin)
            else:
                return 0.0

    def elastic_utility(self, bandwidth: float, capacity: float) -> float:
        """
        计算弹性用户的效用

        Args:
            bandwidth: 分配的带宽
            capacity: 信道容量

        Returns:
            效用值
        """
        r = bandwidth * capacity
        return self.weight * math.log2(1 + r)



    def utility(self, bandwidth: float, capacity: float) -> float:
        """
        根据用户类型计算效用

        Args:
            bandwidth: 分配的带宽
            capacity: 信道容量

        Returns:
            效用值
        """
        if self.uType == HARD_UTILITY:
            return self.hard_utility(bandwidth, capacity)
        elif self.uType == ELASTIC_UTILITY:
            return self.elastic_utility(bandwidth, capacity)
        else:
            raise ValueError(f"Undefined user utility type: {self.uType}")


    def utility_derivative(self, bandwidth: float, capacity: float) -> float:
        """
        计算效用函数的导数（用于优化算法）

        Args:
            bandwidth: 分配的带宽
            capacity: 信道容量（对于elastic用户）或带宽阈值（对于hard用户）

        Returns:
            效用函数的导数值
        """
        if self.uType == HARD_UTILITY:
            # 对于hard用户，capacity表示其需求的带宽阈值
            uti = self.weight * math.log2(1 + self.rMin)
            B_th = capacity
            if B_th > 1e-9:
                return uti / B_th
            return float('inf')
        else:
            # 对于elastic用户，capacity表示信道容量，即log(1+SNR)
            return (self.weight * capacity) / ((bandwidth * capacity + 1) * math.log(2))

    def print_user(self):
        """打印用户信息"""
        utype_str = {HARD_UTILITY: "HARD", ELASTIC_UTILITY: "ELASTIC",
                     HALFSOFT_UTILITY: "HALFSOFT"}.get(self.uType, "UNKNOWN")
        print(f"User ID: {self.ID}, Type: {utype_str}, Weight: {self.weight:.2f}, "
              f"Location: ({self.X:.2f}, {self.Y:.2f}, {self.Z:.2f}), "
              f"rMin: {self.rMin:.2f} Mbps, pOut: {self.pOut:.6f}")

    def __repr__(self):
        utype_str = {HARD_UTILITY: "HARD", ELASTIC_UTILITY: "ELASTIC",
                     HALFSOFT_UTILITY: "HALFSOFT"}.get(self.uType, "UNKNOWN")
        return (f"User(ID={self.ID}, Type={utype_str}, Weight={self.weight:.2f}, "
                f"Pos=({self.X:.1f},{self.Y:.1f},{self.Z:.1f}))")


class Uav(Point):
    """
    无人机类，继承自Point
    包含无人机的发射功率、带宽等参数
    """

    def __init__(self, id: int, x: float, y: float, z: float,
                 total_bandwidth: float = 20.0, p_trans: float = 1.0):
        """
        初始化Uav对象

        Args:
            id: UAV的ID
            x, y, z: 三维坐标
            total_bandwidth: 总带宽 (MHz)
            p_trans: 发射功率 (W)
        """
        super().__init__(x, y, z, id)
        self.total_bandwidth = total_bandwidth  # MHz
        self.pTrans = p_trans  # 发射功率 (W)
        self.max_coverage_distance = 500

    def print_UAV(self):
        """打印UAV信息"""
        print(f"UAV ID: {self.ID}, Location: ({self.X:.2f}, {self.Y:.2f}, {self.Z:.2f}), "
              f"Bandwidth: {self.total_bandwidth:.2f} MHz, pTrans: {self.pTrans:.2f} W")

    def __repr__(self):
        return (f"Uav(ID={self.ID}, Pos=({self.X:.1f},{self.Y:.1f},{self.Z:.1f}), "
                f"BW={self.total_bandwidth:.1f}MHz)")


# ============================================
# 辅助函数
# ============================================

def clean_knapsack_result(result: KnapsackResult):
    """清空背包结果"""
    result.clear()


def add_knapsack_result(result: KnapsackResult, user: User,
                        bandwidth: float, value: float):
    """
    向背包结果中添加一个用户的分配

    Args:
        result: KnapsackResult对象
        user: 用户对象
        bandwidth: 分配的带宽
        value: 对应的价值
    """
    user_id = user.ID
    result.allocated_list.append(user_id)
    result.allocated_bandwidth[user_id] = bandwidth
    result.allocated_value[user_id] = value
    result.total_value += value
    result.total_weight += bandwidth

    if user.uType == HARD_UTILITY:
        result.hard_value += value
        result.hard_weight += bandwidth
    else:
        result.elastic_value += value
        result.elastic_weight += bandwidth


def remove_first(vec: List, value) -> bool:
    """
    删除列表中第一个等于value的元素

    Args:
        vec: 列表
        value: 要删除的值

    Returns:
        是否成功删除
    """
    try:
        vec.remove(value)
        return True
    except ValueError:
        return False


def remove_all(vec: List, value) -> int:
    """
    删除列表中所有等于value的元素

    Args:
        vec: 列表
        value: 要删除的值

    Returns:
        删除的数量
    """
    count = 0
    while value in vec:
        vec.remove(value)
        count += 1
    return count


def remove_if_pred(vec: List, predicate) -> int:
    """
    按谓词删除列表中的元素

    Args:
        vec: 列表
        predicate: 谓词函数

    Returns:
        删除的数量
    """
    original_len = len(vec)
    vec[:] = [x for x in vec if not predicate(x)]
    return original_len - len(vec)


"""
UAV Bandwidth Allocation System - Channel Model
信道模型

包含UAV与用户之间的信道参数计算
基于Nakagami-M衰落模型
"""

class Channel:
    """
    信道类
    计算发送者(UAV)到接收者(User)之间的各种信道参数
    """

    def __init__(self, p_tr: Uav, p_re: User):
        """
        初始化Channel对象
        计算发送者p_tr到接收者p_re之间的各种信道参数

        Args:
            p_tr: 信号发送者，一般指无人机
            p_re: 信号接收者，一般指用户
        """
        self.P_tr = p_tr  # 发送者 (UAV)
        self.P_re = p_re  # 接收者 (User)

        # 计算各项参数
        self.d = Point.cal_distance(p_tr, p_re)  # 距离
        self.theta = self.cal_theta()  # 仰角
        self.P_LoS = self.cal_P_LoS()  # 视距概率
        self.P_NLoS = 1 - self.P_LoS  # 非视距概率
        self.M = self.cal_Nakagami_M()  # Nakagami-M参数
        self.L_LoS = self.cal_L_LoS()  # LoS路径损耗
        self.L_NLoS = self.cal_L_NLoS()  # NLoS路径损耗
        self.PL = self.cal_PL()  # 大尺度衰减系数
        self.SNRa_dB = self.cal_average_SNR()  # 平均信噪比
        self.SNRt_dB = self.cal_SNR_th()  # 信噪比阈值
        self.channel_capacity = self.cal_capacity()  # 信道容量



    def cal_theta(self) -> float:
        """
        计算低点P_re到高点P_tr的仰角
        theta = arctan((z1 - z2) / sqrt((x1 - x2)^2 + (y1 - y2)^2))

        Returns:
            仰角，单位为度
        """
        horizontal_dist = Point.cal_horizontal_distance(self.P_tr, self.P_re)

        if horizontal_dist < 1e-9:  # 避免除零
            return 90.0

        theta_rad = math.atan((self.P_tr.Z - self.P_re.Z) / horizontal_dist)
        theta_deg = math.degrees(theta_rad)

        return theta_deg

    def cal_P_LoS(self) -> float:
        """
        计算发送者P_tr到接收者P_re之间的视距概率
        P_LoS = 1 / (1 + param_a * exp(-param_b * (theta - param_a)))

        Returns:
            视距概率, 0-1之间
        """
        # 使用全局配置参数
        a = sys_config.param_a
        b = sys_config.param_b

        p_los = 1.0 / (1.0 + a * math.exp(-b * (self.theta - a)))

        return p_los

    def cal_L_LoS(self) -> float:
        """
        计算发送者P_tr到接收者P_re之间的视距自由空间路径损耗L_LoS
        L_LoS = 20*log10(4*pi*f*d/c) + η_LoS

        Returns:
            视距路径损耗，单位为dB
        """
        # 自由空间路径损耗
        fspl = 20 * math.log10(4 * sys_config.PI * sys_config.freq_hz * self.d / sys_config.speed_light)

        # 加上额外的LoS损耗
        l_los = fspl + sys_config.los_loss_db

        return l_los

    def cal_L_NLoS(self) -> float:
        """
        计算发送者P_tr到接收者P_re之间的非视距自由空间路径损耗L_NLoS
        L_NLoS = 20*log10(4*pi*f*d/c) + η_NLoS

        Returns:
            非视距路径损耗，单位为dB
        """
        # 自由空间路径损耗
        fspl = 20 * math.log10(4 * sys_config.PI * sys_config.freq_hz * self.d / sys_config.speed_light)

        # 加上额外的NLoS损耗
        l_nlos = fspl + sys_config.nlos_loss_db

        return l_nlos

    def cal_Nakagami_M(self) -> float:
        """
        计算小尺度Nakagami-M衰落的参数M
        M = (K + 1)^2 / (2K + 1)，其中K = P_LoS / P_NLoS

        Returns:
            Nakagami-M参数
        """
        if self.P_NLoS < 1e-9:  # 避免除零
            return 1.0  # 纯LoS情况

        K = self.P_LoS / self.P_NLoS
        M = (K + 1) ** 2 / (2 * K + 1)

        return M

    def cal_PL(self) -> float:
        """
        计算发送者P_tr到接收者P_re之间的大尺度衰减系数PL
        PL = P_LoS * L_LoS + P_NLoS * L_NLoS

        Returns:
            大尺度衰减系数，单位dB
        """
        linear_L_LoS = pow(10, self.L_LoS / 10.0)
        linear_L_NLoS = pow(10, self.L_NLoS / 10.0)
        pl = self.P_LoS * linear_L_LoS + self.P_NLoS * linear_L_NLoS
        pl = 10 * math.log10(pl)

        return pl

    def cal_average_SNR(self) -> float:
        """
        计算发送者P_tr到接收者P_re之间的平均信噪比SNR
        SNR = P_tx * G / (N * PL)

        Returns:
            平均信噪比，单位为dB
        """
        # 发射功率 (W转dBm)
        p_tx_dbm = 10 * math.log10(self.P_tr.pTrans * 1000 )

        # 天线增益
        gain_total_db = sys_config.gain_uav_db + sys_config.gain_bs_db

        # 接收功率 (dBm)
        p_rx_dbm = p_tx_dbm + gain_total_db - self.PL
        # 信噪比 (dB)
        snr_db = p_rx_dbm - sys_config.noise_dbm

        # if self.P_re.ID < 20:
        #     print("p_tx_dbm: " + str(p_tx_dbm) + " gain_total_db: " + str(gain_total_db) + " PL: " + str(
        #         self.PL) + " noise_dbm: " + str(config.noise_dbm) + " snr_db: " + str(snr_db))

        return snr_db

    def cal_SNR_th(self) -> float:
        """
        根据用户的最大中断概率pOut，计算P_tr满足P_re中概率断需求的信噪比阈值SNR_th
        基于Nakagami衰落中的中断概率公式，通过求解反函数得到SNR_th

        中断概率公式:
        pOut = 1/Γ(M) * γ(M, (M * SNR_th) / SNR_avg)

        其中 γ(M, x) 是下不完全伽马函数

        Returns:
            信噪比阈值，单位为dB；如果无法满足需求则返回负无穷
        """
        # 将平均SNR从dB转换为线性值
        snr_avg_linear = 10 ** (self.SNRa_dB / 10)

        # 目标中断概率
        p_out_target = self.P_re.pOut

        # 如果平均SNR太低，可能无法满足QoS要求
        if snr_avg_linear < 1e-9:
            return float('-inf')

        try:
            # 定义中断概率函数
            def outage_prob(snr_th_linear):
                """计算给定SNR阈值下的中断概率"""
                if snr_th_linear <= 0:
                    return 0.0

                x = self.M * snr_th_linear / snr_avg_linear

                # 使用scipy的不完全伽马函数
                # gammainc(a, x) = γ(a,x) / Γ(a)
                # 所以 γ(a,x) = gammainc(a, x) * Γ(a)
                # pOut = γ(M, x) / Γ(M) = gammainc(M, x)
                p_out = special.gammainc(self.M, x)

                return p_out

            # 定义目标函数：outage_prob - p_out_target = 0
            def target_func(snr_th_linear):
                return outage_prob(snr_th_linear) - p_out_target

            # 搜索SNR阈值的范围
            # 下界：接近0
            # 上界：远大于平均SNR（例如100倍）
            lower_bound = 1e-6
            upper_bound = snr_avg_linear * 100

            # 检查边界条件
            if target_func(lower_bound) > 0:
                # 即使SNR阈值很小，中断概率也大于目标值，无法满足
                return float('-inf')

            if target_func(upper_bound) < 0:
                # 即使SNR阈值很大，中断概率仍小于目标值
                # 使用上界作为阈值
                snr_th_linear = upper_bound
            else:
                # 使用二分法求解
                snr_th_linear = brentq(target_func, lower_bound, upper_bound)



            # 转换为dB
            snr_th_db = 10 * math.log10(snr_th_linear)
            # print("user" + str(self.P_re.ID) + " uav" + str(self.P_tr.ID) + ": snr_avg_linear = " + str(snr_avg_linear)
            #       + " snr_avg_db = " + str(self.SNRa_dB) + " p_out_target = " + str(
            #     p_out_target) + " snr_th_linear = " + str(snr_th_linear) + " snr_th_db = " + str(snr_th_db))
            return snr_th_db

        except Exception as e:
            print(f"Warning: Failed to calculate SNR threshold: {e}")
            return float('-inf')

    def cal_capacity(self) -> float:
        """
        计算发送者P_tr到接收者P_re之间的信道容量
        公式: Capacity = (1 - pOut) * log2(1 + SNR_th) hard用户
            Capacity = (1 - pOut) * log2(1 + SNR_a)

        Returns:
            信道容量，单位为bit/s/Hz
        """
        if self.P_re.uType == HARD_UTILITY:
            if self.SNRt_dB == float('-inf'):
                return 0.0

            # 将SNR阈值从dB转换为线性值
            snr_th_linear = 10 ** (self.SNRt_dB / 10)

            # 计算信道容量
            capacity = (1 - self.P_re.pOut) * math.log2(1 + snr_th_linear)
        else:
            if self.SNRa_dB == float('-inf'):
                return 0.0
            snr_a_linear = 10 ** (self.SNRa_dB / 10)
            capacity = math.log2(1 + snr_a_linear)

        return capacity

    def print_all(self):
        """打印所有信道参数"""
        print(f"===== Channel: UAV {self.P_tr.ID} -> User {self.P_re.ID} =====")
        print(f"距离: {self.d:.2f} m")
        print(f"仰角: {self.theta:.2f} 度")
        print(f"LoS概率: {self.P_LoS:.4f}")
        print(f"NLoS概率: {self.P_NLoS:.4f}")
        print(f"Nakagami-M: {self.M:.4f}")
        print(f"LoS路径损耗: {self.L_LoS:.2f} dB")
        print(f"NLoS路径损耗: {self.L_NLoS:.2f} dB")
        print(f"平均路径损耗: {self.PL:.2f} dB")
        print(f"平均SNR: {self.SNRa_dB:.2f} dB")
        print(f"SNR阈值: {self.SNRt_dB:.2f} dB")
        print(f"信道容量: {self.channel_capacity:.4f} bit/s/Hz")
        print("=" * 50)

    def __repr__(self):
        return (f"Channel(UAV{self.P_tr.ID}->User{self.P_re.ID}, "
                f"d={self.d:.1f}m, SNR={self.SNRa_dB:.1f}dB, "
                f"C={self.channel_capacity:.2f}bit/s/Hz)")