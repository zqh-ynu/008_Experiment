import numpy as np
import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field
from SystemModel.sys_config import MAX_COVERAGE, USERS_PER_CLUSTER
from SystemModel.entity_definition import HARD_UTILITY, ELASTIC_UTILITY, EPS
from SystemModel.system_model import SystemMd

@dataclass
class UAVState:
    """单个UAV的动态状态"""
    uav_id: int
    total_bandwidth: float
    remain_bandwidth: float
    num_served_users: int = 0
    served_user_ids: List[int] = field(default_factory=list)

@dataclass
class FeatureConfig:
    """特征配置"""
    # 距离归一化分母 (假设地图大概是 2000x2000 或类似量级，用于缩放)
    # 如果能从 system 动态获取地图大小更好，这里给一个经验值
    map_scale: float = 1000.0 
    
    # 特征维度配置
    cluster_feat_dim: int = 4  # [权重密度, Hard需求密度, 用户数占比, 进度]
    uav_feat_dim: int = 5      # [距离, 覆盖标识, 平均容量, 剩余带宽比, 匹配度]
    global_feat_dim: int = 2   # [全局剩余带宽比, 剩余簇占比]

class FeatureBuilder:
    """
    基于簇(Cluster)的特征构建器
    支持 Scale Invariance (尺度不变性)
    """

    def __init__(self, system: SystemMd, config: Optional[FeatureConfig] = None):
        self.system = system
        self.config = config if config is not None else FeatureConfig()
        
        self.m = system.m
        self.n_total = system.n1 + system.n2
        
        # 预计算归一化常量
        self._init_normalization_constants()
        
        # 状态维度
        self.state_dim = (self.config.cluster_feat_dim + 
                          self.m * self.config.uav_feat_dim + 
                          self.config.global_feat_dim)
        
        # 动作空间: 0=不连接, 1~m=连接对应UAV
        self.action_dim = self.m + 1

    def _init_normalization_constants(self):
        """初始化归一化常数，确保特征数值在 0~1 之间"""
        # 1. 权重归一化: 估算一个簇的最大可能权重
        # 假设最大权重用户是100，一个簇最多100人 -> 10000
        max_user_w = max([u.weight for u in self.system.users]) if self.system.users else 1.0
        self.norm_cluster_weight = max_user_w * USERS_PER_CLUSTER 
        
        hard_ratio: float = self.system.n1 / (self.system.n1 + self.system.n2)
        print(f"hard_ratio = {hard_ratio}")
        # 2. 带宽需求归一化
        # 假设一个簇最大的Hard需求总和
        hard_users = [u for u in self.system.users if u.uType == HARD_UTILITY]
        if hard_users:
            max_r_min = max([u.rMin for u in hard_users])
            self.norm_cluster_demand = max_r_min * USERS_PER_CLUSTER * hard_ratio  # 假设簇内有50个Hard用户
        else:
            self.norm_cluster_demand = 1.0

        # 3. 带宽容量归一化
        self.max_cap = np.max(self.system.cap_list) if self.system.cap_list is not None else 1.0

    def init_uav_states(self) -> List[UAVState]:
        states = []
        for k in range(self.m):
            uav = self.system.uavs[k]
            states.append(UAVState(
                uav_id=k,
                total_bandwidth=uav.total_bandwidth, # SystemModel中属性是bandwidth
                remain_bandwidth=uav.total_bandwidth
            ))
        return states

    def build_state(self, cluster_users: List[int], uav_states: List[UAVState], 
                    step: int, total_steps: int) -> np.ndarray:
        """
        构建状态向量
        Args:
            cluster_users: 当前簇包含的用户ID列表
            uav_states: UAV状态
            step: 当前是第几个簇 (0-based)
            total_steps: 总簇数 (用于归一化进度)
        """
        features = []
        
        # 0. 预计算簇中心
        center_x, center_y = self._get_cluster_center(cluster_users)
        
        # --- A. 簇特征 (Cluster Features) ---
        f_cluster = self._build_cluster_features(cluster_users, step, total_steps)
        features.append(f_cluster)
        
        # --- B. UAV交互特征 (UAV-Cluster Features) ---
        for k in range(self.m):
            f_uav = self._build_uav_features(k, center_x, center_y, cluster_users, uav_states[k])
            features.append(f_uav)
            
        # --- C. 全局特征 (Global Features) ---
        f_global = self._build_global_features(uav_states, step, total_steps)
        features.append(f_global)
        
        return np.concatenate(features).astype(np.float32)

    def _get_cluster_center(self, user_ids: List[int]) -> Tuple[float, float]:
        if not user_ids: return 0.0, 0.0
        # 从SystemModel的用户列表中获取坐标
        # 注意：这里假设User对象有 x, y 属性
        xs = [self.system.users[uid].X for uid in user_ids]
        ys = [self.system.users[uid].Y for uid in user_ids]
        return float(np.mean(xs)), float(np.mean(ys))

    def _build_cluster_features(self, user_ids: List[int], step: int, total_steps: int):
        """[总权重, 总Hard需求, 用户数归一化, 进度]"""
        total_weight = 0.0
        total_demand = 0.0
        
        for uid in user_ids:
            u = self.system.users[uid]
            total_weight += u.weight
            if u.uType == HARD_UTILITY:
                total_demand += u.rMin
                
        feat = np.zeros(self.config.cluster_feat_dim, dtype=np.float32)
        feat[0] = total_weight / self.norm_cluster_weight
        feat[1] = total_demand / self.norm_cluster_demand
        # 归一化用户数：假设平均每个簇50人，除以100归一化
        feat[2] = len(user_ids) / 100.0 
        feat[3] = step / max(total_steps, 1) # 进度
        return feat

    def _build_uav_features(self, uav_idx: int, cx: float, cy: float, 
                            cluster_users: List[int], uav_state: UAVState):
        """[归一化距离, 覆盖Flag, 平均容量, 剩余带宽比, 供需比]"""
        uav = self.system.uavs[uav_idx]
        feat = np.zeros(self.config.uav_feat_dim, dtype=np.float32)
        
        # 1. 距离 (使用欧氏距离)
        dist = math.sqrt((uav.X - cx)**2 + (uav.Y - cy)**2)
        feat[0] = dist / self.config.map_scale
        
        # 2. 覆盖标识 (只要中心在覆盖范围内)
        is_covered = 1.0 if dist <= MAX_COVERAGE else 0.0
        feat[1] = is_covered
        
        # 3. 平均信道容量 (利用SystemModel预计算的矩阵)
        # 这是一个很强的特征，告诉Agent这个簇连这个UAV信号好不好
        if cluster_users:
            avg_cap = np.mean(self.system.cap_list[uav_idx, cluster_users])
            feat[2] = avg_cap / self.max_cap
        
        # 4. 剩余带宽比例
        if uav_state.total_bandwidth > EPS:
            feat[3] = uav_state.remain_bandwidth / uav_state.total_bandwidth
        
        # 5. 供需匹配度 (Supply/Demand)
        # 衡量UAV剩余带宽能否满足该簇的Hard需求
        # 如果需求为0，设为1.0(完全满足)
        # 计算该簇的总需求
        cluster_hard_demand = sum(self.system.users[uid].rMin 
                                  for uid in cluster_users 
                                  if self.system.users[uid].uType == HARD_UTILITY)
        
        if cluster_hard_demand < EPS:
            feat[4] = 1.0
        elif uav_state.remain_bandwidth < EPS:
            feat[4] = 0.0
        else:
            # 限制在 0~2 之间，避免数值过大
            feat[4] = min(uav_state.remain_bandwidth / cluster_hard_demand, 2.0)
            
        return feat

    def _build_global_features(self, uav_states: List[UAVState], step: int, total_steps: int):
        """[全局平均剩余带宽率, 剩余任务比例]"""
        feat = np.zeros(self.config.global_feat_dim, dtype=np.float32)
        
        total_rem = sum(s.remain_bandwidth for s in uav_states)
        total_bw = sum(s.total_bandwidth for s in uav_states)
        
        if total_bw > EPS:
            feat[0] = total_rem / total_bw
            
        feat[1] = 1.0 - (step / max(total_steps, 1))
        return feat

    def compute_action_mask(self, cluster_users: List[int], uav_states: List[UAVState]) -> np.ndarray:
        """
        计算动作掩码
        对于簇决策，只要UAV还有带宽，并且距离不是极其离谱，就允许连接。
        具体的分配可行性由Step函数内部处理。
        """
        mask = np.ones(self.action_dim, dtype=np.float32)
        # Action 0 (不连接) 总是合法的
        
        cx, cy = self._get_cluster_center(cluster_users)
        
        for k in range(self.m):
            uav = self.system.uavs[k]
            state = uav_states[k]
            
            # 1. 检查物理距离是否过远 (例如超过覆盖半径的1.5倍)
            # 允许一定容忍度，因为簇中心在外面但部分用户可能在里面
            dist = math.sqrt((uav.X - cx)**2 + (uav.Y - cy)**2)
            if dist > MAX_COVERAGE:
                mask[k+1] = 0.0
                continue
                
            # 2. 检查UAV是否已满 (剩余带宽接近0)
            if state.remain_bandwidth < EPS:
                mask[k+1] = 0.0
                
        return mask

    def get_state_dim(self): return self.state_dim
    def get_action_dim(self): return self.action_dim