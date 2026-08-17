import sys
import numpy as np
import math
from typing import Dict, Tuple, Optional, List
from sklearn.cluster import KMeans

# 引用你的模块
from KmeansDRL.feature_builder import FeatureBuilder, FeatureConfig, UAVState
from SystemModel.bandwidth_allocator import BandwidthAllocator
from SystemModel.entity_definition import HARD_UTILITY, ELASTIC_UTILITY, EPS
from SystemModel.sys_config import MAX_COVERAGE, USERS_PER_CLUSTER
from SystemModel.system_model import SystemMd

class SequentialEnv:
    """
    Cluster-based Sequential Environment
    聚类/分层顺序决策环境
    """
    
    def __init__(self, system: SystemMd, config=None):
        self.system = system
        # 使用简单的Config字典或对象，这里沿用之前的结构
        self.config = config 
        self.m = system.m
        
        # 初始化 FeatureBuilder
        self.feature_builder = FeatureBuilder(system)
        
        # 初始化 Allocator (用于最终奖励修正)
        # 假设 BandwidthAllocator 接口没变
        self.allocator = BandwidthAllocator(system, use_water_filling=True)
        
        # 期望每个簇包含的用户数 (用于动态计算K)
        self.target_users_per_cluster = USERS_PER_CLUSTER
        
        # 状态记录
        self.clusters: List[List[int]] = [] # [[u1, u2...], [u3, u4...]]
        self.uav_states: List[UAVState] = []
        self.associations: Dict[int, int] = {} # user_id -> uav_id
        
        self.current_step = 0
        self.episode_len = 0
        self.reset()

    def reset(self) -> Dict:
        """重置环境，执行K-Means聚类"""
        self.current_step = 0
        self.associations = {}
        self.cumulative_utility = 0.0
        
        # 1. 动态执行聚类 (Scale Invariance 的关键)
        self._perform_clustering()
        
        # 2. 初始化UAV状态
        self.uav_states = self.feature_builder.init_uav_states()
        
        return self._get_current_state()

    def _perform_clustering(self):
        """对所有用户执行 K-Means 聚类"""
        users = self.system.users
        n_users = len(users)
        
        # 提取坐标
        
        coords = np.array([[u.X, u.Y] for u in users])
        
        # 动态计算 K
        k = max(1, int(np.ceil(n_users / self.target_users_per_cluster)))
        self.episode_len = k # Episode长度 = 簇的数量

        # print(f"簇的数量为：{self.episode_len}")
        
        # 执行 K-Means
        kmeans = KMeans(n_clusters=k, random_state=42, n_init=10)
        labels = kmeans.fit_predict(coords)
        
        # 整理簇
        self.clusters = [[] for _ in range(k)]
        
        # 计算每个簇的“优先级权重”以便排序
        # 策略：按簇的总权重降序排列，让RL先处理高价值簇
        cluster_weights = np.zeros(k)
        
        for uid, label in enumerate(labels):
            self.clusters[label].append(uid)
            cluster_weights[label] += users[uid].weight
            
        # 对簇进行排序 (可选，但推荐，有助于训练稳定性)
        sorted_indices = np.argsort(cluster_weights)[::-1]
        self.clusters = [self.clusters[i] for i in sorted_indices]
        
        # 打印调试信息
        # print(f"Clustering done: {n_users} users -> {k} clusters.")

    def step(self, action: int) -> Tuple[Dict, float, bool, Dict]:
        """
        处理一个簇的分配
        Action: 0=不分配, k=分配给UAV k-1
        """
        cluster_users = self.clusters[self.current_step]
        step_reward = 0.0
        
        assigned_count = 0
        
        if action > 0:
            uav_idx = action - 1
            uav_state = self.uav_states[uav_idx]
            
            # 遍历簇内用户进行贪婪分配 (Greedy Assignment within Cluster)
            # 按照 Hard 优先，权重高优先的顺序
            # 简单的内部排序
            sorted_users = sorted(cluster_users, 
                                key=lambda uid: (self.system.users[uid].uType == HARD_UTILITY, 
                                                 self.system.users[uid].weight), 
                                reverse=True)
            
            for uid in sorted_users:
                user = self.system.users[uid]
                
                # 1. 检查覆盖 (精确物理检查)
                dist = self.system.dis_list[uav_idx, uid]
                if dist > MAX_COVERAGE:
                    continue 
                
                # 2. 尝试分配
                if user.uType == HARD_UTILITY:
                    # Hard 用户：必须满足带宽需求
                    bth = self.system.Bth_list[uav_idx, uid]
                    if uav_state.remain_bandwidth >= bth:
                        # 分配成功
                        self.associations[uid] = uav_idx
                        uav_state.remain_bandwidth -= bth
                        uav_state.num_served_users += 1
                        uav_state.served_user_ids.append(uid)
                        
                        # 估算奖励 (对数效用)
                        step_reward += user.weight * math.log2(1 + user.rMin)
                        assigned_count += 1
                else:
                    # Elastic 用户：只要能连上就行
                    # 但为了不把Hard用户的带宽挤占光，可以设置一个简单的策略
                    # 比如：Elastic用户暂时不扣减 remain_bandwidth，或者扣减一个极小值
                    # 真正的带宽分配由最后的 Allocator 决定
                    self.associations[uid] = uav_idx
                    uav_state.num_served_users += 1
                    uav_state.served_user_ids.append(uid)
                    assigned_count += 1
                    
                    # 估算 Elastic 奖励 (启发式)
                    # 假设分到 1 MHz
                    cap = self.system.cap_list[uav_idx, uid]
                    step_reward += user.weight * math.log2(1 + 1.0 * cap)

        # 步进
        self.current_step += 1
        done = self.current_step >= self.episode_len
        self.cumulative_utility += step_reward
        
        # 归一化 Step Reward (关键！防止数值过大导致梯度爆炸)
        # 假设一个簇满打满算效用是 5000，将其缩放到 0~5 左右
        reward_scale = 1.0 / 1000.0 
        normalized_reward = step_reward * reward_scale
        
        # Episode 结束时的修正 (Final Correction)
        final_correction = 0.0
        info = {}
        
        if done:
            # 构建关联矩阵 X
            X = np.zeros((self.m, self.system.n1 + self.system.n2))
            for uid, uav_idx in self.associations.items():
                X[uav_idx, uid] = 1.0
            
            # 调用 Worker 计算精确效用
            # 注意：Allocator 内部会处理具体的带宽切分
            res_bw, res_info = self.allocator.allocate(X)
            
            precise_utility = res_info['utility_total']
            
            # 计算修正值 (Precise - Estimated)
            # 同样需要缩放
            correction = (precise_utility - self.cumulative_utility) * reward_scale
            # print(f"correction = {correction}")
            # 将修正值加到最后一步的奖励上
            # 如果修正值过大，可以截断 (Clipping)
            normalized_reward += correction
            
            info = {
                'utility_total': precise_utility,      # 修改键名: precise_utility -> utility_total
                'estimated_utility': self.cumulative_utility,
                'final_correction': correction,        # 修改键名: correction -> final_correction (Trainer中其实用的是correction，但统计类用的final_correction，建议统一)
                'num_connected_users': len(self.associations), # 修改键名: connected_users -> num_connected_users
                'feasible': precise_utility > 0        # [新增] 添加 feasible 标志
            }

        return self._get_current_state(), normalized_reward, done, info

    def _get_current_state(self):
        if self.current_step >= self.episode_len:
            # Dummy state for termination
            s_dim = self.feature_builder.get_state_dim()
            a_dim = self.feature_builder.get_action_dim()
            return {'state': np.zeros(s_dim), 'action_mask': np.ones(a_dim), 'done': True}
            
        cluster_users = self.clusters[self.current_step]
        
        state = self.feature_builder.build_state(
            cluster_users, 
            self.uav_states, 
            self.current_step, 
            self.episode_len
        )
        
        mask = self.feature_builder.compute_action_mask(cluster_users, self.uav_states)
        
        return {
            'state': state,
            'action_mask': mask,
            'done': False
        }
    
    # 兼容接口
    def get_state_dim(self): return self.feature_builder.get_state_dim()
    def get_action_dim(self): return self.feature_builder.get_action_dim()
    def get_episode_length(self): return self.episode_len