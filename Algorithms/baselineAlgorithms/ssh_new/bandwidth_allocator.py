"""
Bandwidth Allocator Module (Worker)
带宽分配模块

功能：
1. 给定用户关联矩阵 X，计算最优带宽分配 B
2. 分两步：先满足 Hard 用户，再优化 Elastic 用户
3. 计算系统总效用和约束违反情况
"""
import sys
# 插入SystemModel目录到搜索路径最前面（对应错误日志里的目录）
sys.path.insert(0, r'..\SystemModel')


import numpy as np
import math
from typing import Tuple, Dict, List
from SystemModel.entity_definition import HARD_UTILITY, ELASTIC_UTILITY, EPS
from SystemModel.water_filling import WaterFillingAllocator


class BandwidthAllocator:
    """
    带宽分配器（Worker模块）
    """

    def __init__(self, system, use_water_filling: bool = True):
        """
        初始化分配器

        Args:
            system: SystemMd对象
            use_water_filling: 是否使用Water-filling算法（默认True）
        """
        self.system = system
        self.m = system.m  # UAV数量
        self.n1 = system.n1  # Hard用户数量
        self.n2 = system.n2  # Elastic用户数量
        self.n_total = self.n1 + self.n2
        self.use_water_filling = use_water_filling

        # 创建Water-filling分配器
        if use_water_filling:
            self.wf_allocator = WaterFillingAllocator(system)

    def allocate(self, X: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        给定关联矩阵 X，计算最优带宽分配

        Args:
            X: 关联矩阵，shape (m, n_total)，X[k,i] ∈ {0,1}

        Returns:
            B: 带宽分配矩阵，shape (m, n_total)
            info: 字典，包含以下信息：
                - 'utility_total': 总效用
                - 'utility_hard': Hard用户效用
                - 'utility_elastic': Elastic用户效用
                - 'constraint_violations': 约束违反情况（每个UAV的带宽溢出量）
                - 'is_feasible': 是否可行解
        """
        # 初始化带宽分配矩阵
        B = np.zeros((self.m, self.n_total), dtype=np.float32)

        # 统计信息
        utility_hard = 0.0
        utility_elastic = 0.0
        constraint_violations = np.zeros(self.m)  # 每个UAV的带宽溢出量

        # 对每个UAV独立处理
        for k in range(self.m):
            # 获取连接到该UAV的用户
            connected_users = np.where(X[k, :] > 0.5)[0]

            if len(connected_users) == 0:
                continue  # 该UAV没有连接用户

            # 分离Hard和Elastic用户
            hard_users = [i for i in connected_users if i < self.n1]
            elastic_users = [i for i in connected_users if i >= self.n1]

            # Step 1: 分配Hard用户的最小带宽
            B_hard_required = 0.0
            for i in hard_users:
                # 计算满足QoS需求的最小带宽
                capacity = self.system.cap_list[k, i]
                if capacity > EPS:
                    B_min = self.system.users[i].rMin / capacity
                else:
                    B_min = 1e9  # 无穷大（实际上不可行）

                B[k, i] = B_min
                B_hard_required += B_min

                # 计算Hard用户效用（如果满足QoS）
                if B_min <= self.system.uavs[k].total_bandwidth:
                    utility_hard += self.system.users[i].weight * \
                                   math.log2(1 + self.system.users[i].rMin)

            # 计算剩余带宽
            B_UAV = self.system.uavs[k].total_bandwidth
            remaining_bandwidth = B_UAV - B_hard_required
            # print("remaining_bandwidth = " + str(remaining_bandwidth))
            # 检查Hard用户是否可行
            if remaining_bandwidth < 0:
                # 不可行：记录溢出量
                constraint_violations[k] = -remaining_bandwidth
                # 不分配Elastic用户
                continue

            # Step 2: $U_E$ 注水分配 (Water-filling)
            if len(elastic_users) > 0 and remaining_bandwidth > EPS:
                if self.use_water_filling:
                    # 使用Water-filling算法
                    elastic_user_objs = [self.system.users[i] for i in elastic_users]

                    # 创建一个虚拟UAV，容量为剩余带宽
                    from SystemModel.entity_definition import Uav as UavClass
                    virtual_uav = UavClass(
                        id=k,
                        x=self.system.uavs[k].X,
                        y=self.system.uavs[k].Y,
                        z=self.system.uavs[k].Z,
                        total_bandwidth=remaining_bandwidth,
                        p_trans=self.system.uavs[k].pTrans
                    )

                    wf_result = self.wf_allocator.water_filling_single_uav(
                        virtual_uav,
                        elastic_user_objs,
                        is_rounding=False  # 只需要连续解
                    )

                    # 提取分配结果
                    for user_id in elastic_users:
                        if user_id in wf_result.allocated_bandwidth:
                            B[k, user_id] = wf_result.allocated_bandwidth[user_id]

                            # 计算效用
                            capacity = self.system.cap_list[k, user_id]
                            rate = B[k, user_id] * capacity
                            utility_elastic += self.system.users[user_id].weight * \
                                              math.log2(1 + rate)
                else:
                    # 临时实现：简单均分
                    B_per_user = remaining_bandwidth / len(elastic_users)

                    for i in elastic_users:
                        B[k, i] = B_per_user

                        # 计算Elastic用户效用
                        capacity = self.system.cap_list[k, i]
                        rate = B_per_user * capacity
                        utility_elastic += self.system.users[i].weight * \
                                          math.log2(1 + rate)

        # 汇总信息
        utility_total = utility_hard + utility_elastic
        is_feasible = np.all(constraint_violations < EPS)

        info = {
            'utility_total': utility_total,
            'utility_hard': utility_hard,
            'utility_elastic': utility_elastic,
            'constraint_violations': constraint_violations,
            'total_violation': np.sum(constraint_violations),
            'is_feasible': is_feasible
        }

        return B, info

    def compute_utility(self, X: np.ndarray, B: np.ndarray) -> float:
        """
        计算给定 (X, B) 的系统总效用

        Args:
            X: 关联矩阵，shape (m, n_total)
            B: 带宽分配矩阵，shape (m, n_total)

        Returns:
            总效用值
        """
        utility_total = 0.0

        for k in range(self.m):
            for i in range(self.n_total):
                if X[k, i] > 0.5:  # 用户i连接到UAV k
                    user = self.system.users[i]
                    capacity = self.system.cap_list[k, i]

                    if user.uType == HARD_UTILITY:
                        # Hard用户：阶跃效用
                        rate = B[k, i] * capacity
                        if rate >= user.rMin - EPS:
                            utility_total += user.weight * math.log2(1 + user.rMin)
                    else:
                        # Elastic用户：对数效用
                        rate = B[k, i] * capacity
                        utility_total += user.weight * math.log2(1 + rate)

        return utility_total

    def check_constraints(self, X: np.ndarray, B: np.ndarray) -> Dict:
        """
        检查所有约束是否满足

        Args:
            X: 关联矩阵
            B: 带宽分配矩阵

        Returns:
            约束检查结果字典
        """
        violations = {
            'user_association': [],  # 用户关联唯一性违反
            'bandwidth_coupling': [],  # 带宽耦合约束违反
            'uav_capacity': [],  # UAV容量约束违反
            'qos_hard': [],  # Hard用户QoS约束违反
            'bandwidth_nonneg': []  # 带宽非负约束违反
        }

        # 1. 用户关联唯一性：sum_k X[k,i] <= 1
        for i in range(self.n_total):
            if np.sum(X[:, i]) > 1 + EPS:
                violations['user_association'].append((i, np.sum(X[:, i])))

        # 2. 带宽耦合：B[k,i] <= X[k,i] * B_UAV
        for k in range(self.m):
            for i in range(self.n_total):
                B_UAV = self.system.uavs[k].total_bandwidth
                if B[k, i] > X[k, i] * B_UAV + EPS:
                    violations['bandwidth_coupling'].append((k, i, B[k, i], X[k, i] * B_UAV))

        # 3. UAV容量约束：sum_i B[k,i] <= B_UAV
        for k in range(self.m):
            B_UAV = self.system.uavs[k].total_bandwidth
            total_allocated = np.sum(B[k, :])
            if total_allocated > B_UAV + EPS:
                violations['uav_capacity'].append((k, total_allocated, B_UAV))

        # 4. Hard用户QoS约束：B[k,i] * capacity >= X[k,i] * r_min
        for k in range(self.m):
            for i in range(self.n1):
                if X[k, i] > 0.5:
                    capacity = self.system.cap_list[k, i]
                    rate = B[k, i] * capacity
                    r_min = self.system.users[i].rMin
                    if rate < r_min - EPS:
                        violations['qos_hard'].append((k, i, rate, r_min))

        # 5. 带宽非负：B[k,i] >= 0
        negative_indices = np.where(B < -EPS)
        if len(negative_indices[0]) > 0:
            for k, i in zip(negative_indices[0], negative_indices[1]):
                violations['bandwidth_nonneg'].append((k, i, B[k, i]))

        return violations

    def print_allocation_summary(self, X: np.ndarray, B: np.ndarray, info: Dict):
        """打印分配结果摘要"""
        print("\n" + "="*80)
        print("Bandwidth Allocation Summary")
        print("="*80)

        # 统计连接情况
        num_connected_users = np.sum(np.sum(X, axis=0) > 0.5)
        num_connected_hard = np.sum(np.sum(X[:, :self.n1], axis=0) > 0.5)
        num_connected_elastic = np.sum(np.sum(X[:, self.n1:], axis=0) > 0.5)

        print(f"Connected users: {num_connected_users} / {self.n_total}")
        print(f"  - Hard: {num_connected_hard} / {self.n1}")
        print(f"  - Elastic: {num_connected_elastic} / {self.n2}")

        # 效用统计
        print(f"\nUtility:")
        print(f"  Total: {info['utility_total']:.4f}")
        print(f"  Hard: {info['utility_hard']:.4f}")
        print(f"  Elastic: {info['utility_elastic']:.4f}")

        # 约束违反情况
        print(f"\nConstraint Violations:")
        print(f"  Total violation: {info['total_violation']:.4f} MHz")
        print(f"  Feasible: {info['is_feasible']}")

        if not info['is_feasible']:
            print(f"  Violated UAVs:")
            for k, violation in enumerate(info['constraint_violations']):
                if violation > EPS:
                    print(f"    UAV {k}: overflow {violation:.4f} MHz")

        # 每个UAV的负载
        print(f"\nUAV Load:")
        for k in range(self.m):
            total_allocated = np.sum(B[k, :])
            B_UAV = self.system.uavs[k].total_bandwidth
            utilization = 100 * total_allocated / B_UAV
            print(f"  UAV {k}: {total_allocated:.4f} / {B_UAV:.4f} MHz ({utilization:.1f}%)")

        print("="*80)


# 测试代码
if __name__ == "__main__":
    from system_model import SystemMd
    import sys

    print("Testing Bandwidth Allocator...")
    system = SystemMd(
        user_file=r"../data/train_data/1_5000users_data_230717.csv",
        uav_file=f"../data/train_data/1_20uavs_loc_230717.csv",
        config_file="def_config.json"
    )

    # 创建分配器
    allocator = BandwidthAllocator(system)

    # 测试：随机关联矩阵
    print("\n--- Test 1: Random Association ---")
    X_random = np.zeros((system.m, system.n1 + system.n2))
    for i in range(system.n1 + system.n2):
        # 每个用户随机连接到一个UAV
        if np.random.rand() > 0.3:  # 70%的用户连接
            k = np.random.randint(0, system.m)
            X_random[k, i] = 1.0

    B, info = allocator.allocate(X_random)
    allocator.print_allocation_summary(X_random, B, info)

    # 检查约束
    violations = allocator.check_constraints(X_random, B)
    if any(len(v) > 0 for v in violations.values()):
        print("\nConstraint Violations Detected:")
        for constraint_type, violation_list in violations.items():
            if len(violation_list) > 0:
                print(f"  {constraint_type}: {len(violation_list)} violations")