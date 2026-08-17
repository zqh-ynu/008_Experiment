"""
Sequential Dueling DQN Network
顺序决策 Dueling DQN 网络

特点：
1. 轻量级DNN，参数量与用户数无关
2. Dueling架构（Value + Advantage分离）
3. Double DQN训练
4. 优先经验回放（PER）
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
import random
from collections import namedtuple


# ===========================
# 配置
# ===========================

@dataclass
class DQNConfig:
    """DQN配置"""
    # 网络结构
    hidden_dim: int = 512
    num_hidden_layers: int = 3
    dropout: float = 0.1

    # 训练参数
    learning_rate: float = 2e-4
    gamma: float = 0.99
    epsilon_start: float = 0.95
    epsilon_end: float = 0.01
    epsilon_decay_steps: int = 20000   # 原来是 60000 (约600 eps)，改为 10000 (约100 eps) 或 20000 (约200 eps)

    # 经验回放
    buffer_size: int = 200000
    batch_size: int = 256
    min_buffer_size: int = 500        # 开始训练的最小buffer大小

    # 目标网络
    target_update_freq: int = 4       # 每N步更新目标网络
    tau: float = 0.005                 # [新增] soft update系数
    # 训练频率
    train_every_n_steps: int = 4       # [新增] 每4步训练一次

    # PER
    use_per: bool = True
    per_alpha: float = 0.6
    per_beta_start: float = 0.4
    per_beta_end: float = 1.0
    per_beta_steps: int = 100000

    # 梯度裁剪
    max_grad_norm: float = 10.0


# ===========================
# Dueling DQN 网络
# ===========================

class DuelingDQN(nn.Module):
    """
    Dueling DQN网络

    Q(s, a) = V(s) + A(s, a) - mean(A(s, ·))

    输入: 状态向量 (batch_size, state_dim)
    输出: Q值 (batch_size, action_dim)
    """

    def __init__(self, state_dim: int, action_dim: int, config: DQNConfig):
        super().__init__()

        self.state_dim = state_dim
        self.action_dim = action_dim

        # 共享特征提取层
        layers = []
        in_dim = state_dim
        for i in range(config.num_hidden_layers):
            layers.append(nn.Linear(in_dim, config.hidden_dim))
            layers.append(nn.ReLU())
            if config.dropout > 0:
                layers.append(nn.Dropout(config.dropout))
            in_dim = config.hidden_dim
        self.feature_net = nn.Sequential(*layers)

        # Value stream: 输出标量 V(s)
        self.value_stream = nn.Sequential(
            nn.Linear(config.hidden_dim, config.hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(config.hidden_dim // 2, 1)
        )

        # Advantage stream: 输出每个动作的优势 A(s, a)
        self.advantage_stream = nn.Sequential(
            nn.Linear(config.hidden_dim, config.hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(config.hidden_dim // 2, action_dim)
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """
        前向传播

        Args:
            state: (batch_size, state_dim)
        Returns:
            q_values: (batch_size, action_dim)
        """
        features = self.feature_net(state)

        value = self.value_stream(features)           # (batch, 1)
        advantage = self.advantage_stream(features)   # (batch, action_dim)

        # Dueling: Q = V + A - mean(A)
        q_values = value + advantage - advantage.mean(dim=1, keepdim=True)

        return q_values


# ===========================
# 优先经验回放
# ===========================

Transition = namedtuple('Transition',
                        ['state', 'action', 'reward', 'next_state',
                         'done', 'mask', 'next_mask'])


class SumTree:
    """Sum Tree用于高效PER采样"""

    def __init__(self, capacity: int):
        self.capacity = capacity
        self.tree = np.zeros(2 * capacity - 1)
        self.data = [None] * capacity
        self.write_pos = 0
        self.size = 0

    def _propagate(self, idx: int, change: float):
        parent = (idx - 1) // 2
        self.tree[parent] += change
        if parent != 0:
            self._propagate(parent, change)

    def _retrieve(self, idx: int, s: float) -> int:
        left = 2 * idx + 1
        right = left + 1

        if left >= len(self.tree):
            return idx

        if s <= self.tree[left]:
            return self._retrieve(left, s)
        else:
            return self._retrieve(right, s - self.tree[left])

    def total(self) -> float:
        return self.tree[0]

    def add(self, priority: float, data):
        idx = self.write_pos + self.capacity - 1
        self.data[self.write_pos] = data
        self.update(idx, priority)
        self.write_pos = (self.write_pos + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def update(self, idx: int, priority: float):
        change = priority - self.tree[idx]
        self.tree[idx] = priority
        self._propagate(idx, change)

    def get(self, s: float):
        idx = self._retrieve(0, s)
        data_idx = idx - self.capacity + 1
        return idx, self.tree[idx], self.data[data_idx]


class PrioritizedReplayBuffer:
    """优先经验回放缓冲区"""

    def __init__(self, capacity: int, alpha: float = 0.6):
        self.tree = SumTree(capacity)
        self.capacity = capacity
        self.alpha = alpha
        self.max_priority = 1.0
        self.eps = 1e-6

    def push(self, transition: Transition):
        priority = self.max_priority ** self.alpha
        self.tree.add(priority, transition)

    def sample(self, batch_size: int, beta: float = 0.4):
        batch = []
        indices = []
        priorities = []

        segment = self.tree.total() / batch_size

        for i in range(batch_size):
            a = segment * i
            b = segment * (i + 1)
            s = random.uniform(a, b)

            idx, priority, data = self.tree.get(s)

            if data is None:
                # 如果数据为None，重新采样
                s = random.uniform(0, self.tree.total())
                idx, priority, data = self.tree.get(s)

            batch.append(data)
            indices.append(idx)
            priorities.append(priority)

        # 计算重要性采样权重
        total = self.tree.total()
        probs = np.array(priorities) / (total + self.eps)
        min_prob = probs.min()

        weights = (self.tree.size * probs) ** (-beta)
        max_weight = (self.tree.size * min_prob) ** (-beta)
        weights /= (max_weight + self.eps)

        return batch, np.array(indices), np.array(weights, dtype=np.float32)

    def update_priorities(self, indices: np.ndarray, priorities: np.ndarray):
        for idx, priority in zip(indices, priorities):
            priority = max(priority, self.eps)
            self.max_priority = max(self.max_priority, priority)
            self.tree.update(idx, priority ** self.alpha)

    def __len__(self):
        return self.tree.size


class UniformReplayBuffer:
    """均匀采样经验回放缓冲区"""

    def __init__(self, capacity: int):
        self.capacity = capacity
        self.buffer = []
        self.position = 0

    def push(self, transition: Transition):
        if len(self.buffer) < self.capacity:
            self.buffer.append(transition)
        else:
            self.buffer[self.position] = transition
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size: int, beta: float = 0.4):
        indices = np.random.choice(len(self.buffer), batch_size, replace=False)
        batch = [self.buffer[i] for i in indices]
        weights = np.ones(batch_size, dtype=np.float32)
        return batch, indices, weights

    def update_priorities(self, indices, priorities):
        pass  # 均匀采样不需要优先级

    def __len__(self):
        return len(self.buffer)


# ===========================
# DQN Agent
# ===========================

class SequentialDQNAgent:
    """
    Sequential DQN智能体

    整合网络、经验回放、训练逻辑
    """

    def __init__(self, state_dim: int, action_dim: int,
                 config: Optional[DQNConfig] = None):
        self.config = config if config is not None else DQNConfig()
        self.state_dim = state_dim
        self.action_dim = action_dim

        # 设备
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # 策略网络和目标网络
        self.policy_net = DuelingDQN(state_dim, action_dim, self.config).to(self.device)
        self.target_net = DuelingDQN(state_dim, action_dim, self.config).to(self.device)
        self.update_target_network()

        # 优化器
        self.optimizer = optim.Adam(
            self.policy_net.parameters(),
            lr=self.config.learning_rate
        )

        # 经验回放
        if self.config.use_per:
            self.replay_buffer = PrioritizedReplayBuffer(
                self.config.buffer_size,
                alpha=self.config.per_alpha
            )
        else:
            self.replay_buffer = UniformReplayBuffer(self.config.buffer_size)

        # Epsilon
        self.epsilon = self.config.epsilon_start
        self.train_steps = 0
        self.env_steps = 0             # 环境交互步数，用于控制训练频率


        # PER beta
        self.per_beta = self.config.per_beta_start

        # 统计
        param_count = sum(p.numel() for p in self.policy_net.parameters())
        print(f"Sequential DQN Agent initialized on {self.device}")
        print(f"  State dim: {state_dim}, Action dim: {action_dim}")
        print(f"  Parameters: {param_count:,}")
        print(f"  Buffer: {'PER' if self.config.use_per else 'Uniform'} "
              f"(capacity={self.config.buffer_size})")

    def select_action(self, state: np.ndarray, mask: np.ndarray,
                      epsilon: Optional[float] = None) -> int:
        """
        ε-greedy动作选择

        Args:
            state: 状态向量
            mask: 动作掩码
            epsilon: 探索率（None则使用self.epsilon）

        Returns:
            action: 选择的动作
        """
        if epsilon is None:
            epsilon = self.epsilon

        valid_actions = np.where(mask > 0.5)[0]
        if len(valid_actions) == 0:
            return 0  # 安全回退

        if random.random() < epsilon:
            # 探索
            return int(np.random.choice(valid_actions))
        else:
            # 利用
            self.policy_net.eval()
            with torch.no_grad():
                state_t = torch.FloatTensor(state).unsqueeze(0).to(self.device)
                q_values = self.policy_net(state_t).cpu().numpy()[0]

            # 掩码
            q_values[mask < 0.5] = -1e9
            return int(np.argmax(q_values))

    def select_action_remove_reselect(self, state: np.ndarray,
                                       mask: np.ndarray,
                                       feasibility_check=None) -> int:
        """
        Remove-Reselect动作选择（执行阶段）

        Args:
            state: 状态向量
            mask: 动作掩码
            feasibility_check: 可行性检查函数 action -> bool

        Returns:
            action: 可行的最优动作
        """
        self.policy_net.eval()
        with torch.no_grad():
            state_t = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            q_values = self.policy_net(state_t).cpu().numpy()[0]

        # 掩码
        q_masked = q_values.copy()
        q_masked[mask < 0.5] = -1e9

        # 按Q值降序尝试
        sorted_actions = np.argsort(q_masked)[::-1]

        for action in sorted_actions:
            if mask[action] < 0.5:
                continue

            if feasibility_check is None or feasibility_check(int(action)):
                return int(action)

        # 所有动作都不可行，返回"不连接"
        return 0

    def store_transition(self, state: np.ndarray, action: int, reward: float,
                         next_state: np.ndarray, done: bool,
                         mask: np.ndarray, next_mask: np.ndarray):
        """存储经验"""
        transition = Transition(
            state=state,
            action=action,
            reward=reward,
            next_state=next_state,
            done=done,
            mask=mask,
            next_mask=next_mask
        )
        self.replay_buffer.push(transition)

    def train_step(self) -> Optional[Dict]:
        """
        执行一步训练

        Returns:
            stats: 训练统计，如果buffer不够大返回None
        """
        # [改] 递增环境步数计数器
        self.env_steps += 1

        # [改] 每 train_every_n_steps 步才真正训练一次
        if self.env_steps % self.config.train_every_n_steps != 0:
            return None

        if len(self.replay_buffer) < self.config.min_buffer_size:
            return None

        self.train_steps += 1
        self.policy_net.train()

        # 采样
        batch, indices, is_weights = self.replay_buffer.sample(
            self.config.batch_size, beta=self.per_beta
        )

        # 拆分batch
        states = torch.FloatTensor(
            np.array([t.state for t in batch])
        ).to(self.device)
        actions = torch.LongTensor(
            [t.action for t in batch]
        ).to(self.device)
        rewards = torch.FloatTensor(
            [t.reward for t in batch]
        ).to(self.device)
        next_states = torch.FloatTensor(
            np.array([t.next_state for t in batch])
        ).to(self.device)
        dones = torch.FloatTensor(
            [float(t.done) for t in batch]
        ).to(self.device)
        next_masks = torch.FloatTensor(
            np.array([t.next_mask for t in batch])
        ).to(self.device)
        is_weights_t = torch.FloatTensor(is_weights).to(self.device)

        # === 当前Q值 ===
        q_values = self.policy_net(states)  # (batch, action_dim)
        q_selected = q_values.gather(1, actions.unsqueeze(1)).squeeze(1)  # (batch,)

        # === 目标Q值 (Double DQN) ===
        with torch.no_grad():
            # 策略网络选择动作
            next_q_policy = self.policy_net(next_states)
            next_q_policy[next_masks < 0.5] = -1e9
            best_next_actions = next_q_policy.argmax(dim=1)  # (batch,)

            # 目标网络评估
            next_q_target = self.target_net(next_states)
            next_q_selected = next_q_target.gather(
                1, best_next_actions.unsqueeze(1)
            ).squeeze(1)

            # TD目标
            q_target = rewards + self.config.gamma * next_q_selected * (1 - dones)

        # === 损失 ===
        td_errors = q_selected - q_target
        loss = (is_weights_t * F.smooth_l1_loss(
            q_selected, q_target, reduction='none'
        )).mean()

        # 反向传播
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(
            self.policy_net.parameters(),
            self.config.max_grad_norm
        )
        self.optimizer.step()

        # 更新PER优先级
        priorities = np.abs(td_errors.detach().cpu().numpy()) + 1e-6
        self.replay_buffer.update_priorities(indices, priorities)

        # [改] 每步都做 soft update，取代原来的硬更新
        self.soft_update_target_network()

        # 更新epsilon（线性衰减）
        self._update_epsilon()

        # 更新PER beta
        self._update_per_beta()

        return {
            'loss': loss.item(),
            'q_mean': q_selected.mean().item(),
            'q_std': q_selected.std().item(),
            'td_error_mean': td_errors.abs().mean().item(),
            'epsilon': self.epsilon,
            'train_steps': self.train_steps
        }
    

    def soft_update_target_network(self):
        """[新增] Soft update: target = τ * policy + (1-τ) * target"""
        tau = self.config.tau
        for target_param, policy_param in zip(
            self.target_net.parameters(), self.policy_net.parameters()
        ):
            target_param.data.copy_(
                tau * policy_param.data + (1.0 - tau) * target_param.data
            )
            
    def _update_epsilon(self):
        """线性衰减epsilon"""
        progress = min(self.train_steps / max(self.config.epsilon_decay_steps, 1), 1.0)
        self.epsilon = self.config.epsilon_start + progress * (
            self.config.epsilon_end - self.config.epsilon_start
        )

    def _update_per_beta(self):
        """线性增长PER beta"""
        if self.config.use_per:
            progress = min(self.train_steps / max(self.config.per_beta_steps, 1), 1.0)
            self.per_beta = self.config.per_beta_start + progress * (
                self.config.per_beta_end - self.config.per_beta_start
            )

    def update_target_network(self):
        """将策略网络参数复制到目标网络"""
        self.target_net.load_state_dict(self.policy_net.state_dict())

    def save(self, filepath: str):
        """保存模型"""
        torch.save({
            'policy_state_dict': self.policy_net.state_dict(),
            'target_state_dict': self.target_net.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'epsilon': self.epsilon,
            'train_steps': self.train_steps,
            'config': self.config
        }, filepath)
        print(f"Model saved to {filepath}")

    def load(self, filepath: str):
        """加载模型"""
        checkpoint = torch.load(filepath, map_location=self.device,
                                weights_only=False)
        self.policy_net.load_state_dict(checkpoint['policy_state_dict'])
        self.target_net.load_state_dict(checkpoint['target_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.epsilon = checkpoint['epsilon']
        self.train_steps = checkpoint['train_steps']
        print(f"Model loaded from {filepath}")


# ===========================
# 测试
# ===========================

if __name__ == "__main__":
    print("Testing Sequential DQN Agent...")

    config = DQNConfig(hidden_dim=128, num_hidden_layers=2)
    agent = SequentialDQNAgent(
        state_dim=32, action_dim=5, config=config
    )

    # 测试动作选择
    state = np.random.randn(32).astype(np.float32)
    mask = np.array([1, 1, 0, 1, 1], dtype=np.float32)

    action = agent.select_action(state, mask, epsilon=0.5)
    print(f"Selected action: {action}")

    # 测试存储和训练
    for i in range(300):
        s = np.random.randn(32).astype(np.float32)
        a = np.random.randint(0, 5)
        r = np.random.randn()
        s_next = np.random.randn(32).astype(np.float32)
        m = np.ones(5, dtype=np.float32)
        agent.store_transition(s, a, r, s_next, False, m, m)

    stats = agent.train_step()
    if stats:
        print(f"Train stats: loss={stats['loss']:.4f}, "
              f"q_mean={stats['q_mean']:.4f}")

    print("\n✅ Sequential DQN Agent test completed!")