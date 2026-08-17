"""
Sequential DQN Trainer
顺序决策DQN训练器

包含：
1. 训练循环（逐episode、逐step）
2. 评估
3. 日志和统计
4. 检查点保存
"""

import sys
sys.path.insert(0, '..')

from SystemModel.system_model import SystemMd
import numpy as np
import time
import os
from typing import Dict, List, Optional
from dataclasses import dataclass, field

from KmeansDRL.sequential_env import SequentialEnv
from KmeansDRL.sequential_dqn_network import SequentialDQNAgent, DQNConfig

os.environ["CUDA_VISIBLE_DEVICES"] = "1"
MODEL_SAVE_DIR = f"./modelData/checkpoints/test_ins230717"
STATS_SAVE_DIR = f"./modelData/stats/test_ins230717"

#MODEL_SAVE_DIR = f"./modelData/checkpoints/Kmeans"
#STATS_SAVE_DIR = f"./modelData/stats/Kmeans"

@dataclass
class TrainConfig:
    """训练配置"""
    num_episodes: int = 5000
    log_freq: int = 10
    eval_freq: int = 50
    save_freq: int = 200
    num_eval_episodes: int = 20
    eval_epsilon: float = 0.02

    # 早停
    early_stopping_patience: int = 1000
    early_stopping_min_episodes: int = 500

    # 路径
    model_save_dir: str = MODEL_SAVE_DIR
    stats_save_dir: str = STATS_SAVE_DIR

@dataclass
class TrainStats:
    """训练统计"""
    # Episode级别
    episode_rewards: List[float] = field(default_factory=list)
    episode_utilities: List[float] = field(default_factory=list)
    episode_feasible: List[bool] = field(default_factory=list)
    episode_connected: List[int] = field(default_factory=list)
    episode_corrections: List[float] = field(default_factory=list)

    # 评估
    eval_rewards: List[float] = field(default_factory=list)
    eval_utilities: List[float] = field(default_factory=list)
    eval_feasible_rates: List[float] = field(default_factory=list)

    # 训练
    training_losses: List[float] = field(default_factory=list)
    q_values: List[float] = field(default_factory=list)
    epsilons: List[float] = field(default_factory=list)

    # 最佳
    best_eval_utility: float = -float('inf')
    best_episode: int = 0


class SequentialTrainer:
    """
    Sequential DQN训练器
    """

    def __init__(self, env: SequentialEnv, agent: SequentialDQNAgent,
                 config: Optional[TrainConfig] = None):
        self.env = env
        self.agent = agent
        self.config = config if config is not None else TrainConfig()
        self.stats = TrainStats()

        # 创建保存目录
        os.makedirs(self.config.model_save_dir, exist_ok=True)
        os.makedirs(self.config.stats_save_dir, exist_ok=True)

        print("Trainer initialized:")
        print(f"  Episodes: {self.config.num_episodes}")
        print(f"  Episode length: {env.get_episode_length()} steps")
        print(f"  Total steps: ~{self.config.num_episodes * env.get_episode_length()}")

    def train(self) -> TrainStats:
        """主训练循环"""
        print("\n" + "=" * 70)
        print("Starting Training")
        print("=" * 70)

        start_time = time.time()
        no_improvement = 0

        for episode in range(1, self.config.num_episodes + 1):
            # 训练一个episode
            ep_reward, ep_info, ep_train_stats = self._train_episode()

            # 记录统计
            self.stats.episode_rewards.append(ep_reward)
            self.stats.epsilons.append(self.agent.epsilon)

            if ep_info is not None and 'utility_total' in ep_info:
                self.stats.episode_utilities.append(ep_info['utility_total'])
                self.stats.episode_feasible.append(ep_info.get('feasible', False))
                self.stats.episode_connected.append(
                    ep_info.get('num_connected_users', 0)
                )
                self.stats.episode_corrections.append(
                    ep_info.get('final_correction', 0.0)
                )
            else:
                self.stats.episode_utilities.append(0.0)
                self.stats.episode_feasible.append(False)
                self.stats.episode_connected.append(0)
                self.stats.episode_corrections.append(0.0)

            if ep_train_stats:
                self.stats.training_losses.append(ep_train_stats['avg_loss'])
                self.stats.q_values.append(ep_train_stats['avg_q'])

            # 日志
            if episode % self.config.log_freq == 0:
                self._log_episode(episode, ep_reward, ep_info)

            # 评估
            if episode % self.config.eval_freq == 0:
                eval_result = self._evaluate()
                self.stats.eval_rewards.append(eval_result['mean_reward'])
                self.stats.eval_utilities.append(eval_result['mean_utility'])
                self.stats.eval_feasible_rates.append(eval_result['feasible_rate'])

                print(f"  [Eval] R={eval_result['mean_reward']:.4f} | "
                      f"U={eval_result['mean_utility']:.2f} | "
                      f"F={eval_result['feasible_rate']*100:.1f}% | "
                      f"Conn={eval_result['mean_connected']:.1f}")

                # 更新最佳
                if eval_result['mean_utility'] > self.stats.best_eval_utility:
                    self.stats.best_eval_utility = eval_result['mean_utility']
                    self.stats.best_episode = episode
                    no_improvement = 0
                    # 保存最佳模型
                    self.agent.save(os.path.join(
                        self.config.model_save_dir, "best_model.pth"
                    ))
                else:
                    no_improvement += self.config.eval_freq

            # 定期保存
            if episode % self.config.save_freq == 0:
                self._save_checkpoint(episode)

            # 早停
            # if (episode > self.config.early_stopping_min_episodes and
            #         no_improvement >= self.config.early_stopping_patience):
            #     print(f"\nEarly stopping at episode {episode} "
            #           f"(no improvement for {no_improvement} episodes)")
            #     break

        # 训练结束
        elapsed = time.time() - start_time
        print("\n" + "=" * 70)
        print("Training Completed")
        print("=" * 70)
        print(f"Time: {elapsed:.1f}s ({elapsed/60:.1f}min)")
        print(f"Episodes: {len(self.stats.episode_rewards)}")
        print(f"Best eval utility: {self.stats.best_eval_utility:.4f} "
              f"(episode {self.stats.best_episode})")

        # 保存最终统计
        self._save_stats()

        return self.stats

    def _train_episode(self):
        """
        训练一个episode

        Returns:
            total_reward: episode总奖励
            last_info: 最后一步的info（包含Worker结果）
            train_stats: 训练统计
        """
        state_dict = self.env.reset()
        total_reward = 0.0
        last_info = None
        losses = []
        q_means = []

        while not state_dict.get('done', False):
            state = state_dict['state']
            mask = state_dict['action_mask']

            # 选择动作
            action = self.agent.select_action(state, mask)

            # 执行
            next_state_dict, reward, done, info = self.env.step(action)

            next_state = next_state_dict['state']
            next_mask = next_state_dict['action_mask']

            # 存储经验
            self.agent.store_transition(
                state, action, reward, next_state, done, mask, next_mask
            )

            # 训练一步
            # 只有当 buffer 里的数据量足够采样时，才进行训练
            # 这里的 batch_size 应该从 agent 的 config 中获取
            if len(self.agent.replay_buffer) >= self.agent.config.batch_size:
                train_result = self.agent.train_step()
                if train_result is not None:
                    losses.append(train_result['loss'])
                    q_means.append(train_result['q_mean'])
            else:
                train_result = None

            total_reward += reward
            last_info = info
            state_dict = next_state_dict

        # 汇总训练统计
        train_stats = None
        if losses:
            train_stats = {
                'avg_loss': np.mean(losses),
                'avg_q': np.mean(q_means)
            }

        return total_reward, last_info, train_stats

    def _evaluate(self, num_episodes: Optional[int] = None) -> Dict:
        """评估当前策略"""
        if num_episodes is None:
            num_episodes = self.config.num_eval_episodes

        rewards = []
        utilities = []
        feasible = []
        connected = []

        for _ in range(num_episodes):
            state_dict = self.env.reset()
            ep_reward = 0.0

            while not state_dict.get('done', False):
                state = state_dict['state']
                mask = state_dict['action_mask']

                action = self.agent.select_action(
                    state, mask, epsilon=self.config.eval_epsilon
                )
                state_dict, reward, done, info = self.env.step(action)
                ep_reward += reward

            rewards.append(ep_reward)
            if 'utility_total' in info:
                utilities.append(info['utility_total'])
                feasible.append(info.get('feasible', False))
                connected.append(info.get('num_connected_users', 0))

        return {
            'mean_reward': np.mean(rewards),
            'std_reward': np.std(rewards),
            'mean_utility': np.mean(utilities) if utilities else 0.0,
            'feasible_rate': np.mean(feasible) if feasible else 0.0,
            'mean_connected': np.mean(connected) if connected else 0.0
        }

    def _log_episode(self, episode: int, reward: float, info: Optional[Dict]):
        """打印训练进度"""
        window = 100
        recent_rewards = self.stats.episode_rewards[-window:]
        recent_utils = self.stats.episode_utilities[-window:]
        recent_feasible = self.stats.episode_feasible[-window:]

        feasible_str = "✅" if info and info.get('feasible', False) else "❌"
        utility = info.get('utility_total', 0.0) if info else 0.0
        connected = info.get('num_connected_users', 0) if info else 0

        loss_str = ""
        if self.stats.training_losses:
            loss_str = f"L={self.stats.training_losses[-1]:.4f} | "

        print(f"Ep {episode:5d} | "
              f"R={reward:7.4f} | "
              f"U={utility:7.2f} | "
              f"{feasible_str} C={connected:3d} | "
              f"ε={self.agent.epsilon:.3f} | "
              f"{loss_str}"
              f"Avg{window}: R={np.mean(recent_rewards):7.4f} "
              f"F={np.mean(recent_feasible)*100:5.1f}%")

    def _save_checkpoint(self, episode: int):
        """保存检查点"""
        path = os.path.join(self.config.model_save_dir, f"model_ep{episode}.pth")
        self.agent.save(path)

    def _save_stats(self):
        """保存训练统计"""
        path = os.path.join(self.config.stats_save_dir, "training_stats.npz")
        np.savez(
            path,
            episode_rewards=np.array(self.stats.episode_rewards),
            episode_utilities=np.array(self.stats.episode_utilities),
            episode_feasible=np.array(self.stats.episode_feasible),
            episode_connected=np.array(self.stats.episode_connected),
            episode_corrections=np.array(self.stats.episode_corrections),
            eval_rewards=np.array(self.stats.eval_rewards),
            eval_utilities=np.array(self.stats.eval_utilities),
            eval_feasible_rates=np.array(self.stats.eval_feasible_rates),
            training_losses=np.array(self.stats.training_losses),
            q_values=np.array(self.stats.q_values),
            epsilons=np.array(self.stats.epsilons)
        )
        print(f"Stats saved to {path}")


# ===========================
# 便捷启动函数
# ===========================

def run_training(system, num_episodes: int = 2000, 
                 verbose: int = 0) -> TrainStats:
    
    # 1. 环境 (Cluster-based)
    env = SequentialEnv(system)
    
    # 2. 智能体配置
    # 由于 Step 变少了 (3000 -> 60)，Gamma 可以设置得更激进
    dqn_config = DQNConfig(
        hidden_dim=512,
        num_hidden_layers=3, # 稍微加深一点，因为Cluster特征比User特征复杂
        
        # [关键] Gamma
        # 60步的 Episode, 0.99^60 ~= 0.54, 依然能传递奖励
        gamma=0.99, 
        
        # [关键] Buffer Size
        # 现在一个 Episode 只有 ~60 个 transition
        # 100,000 的 buffer 可以存 1600 个完整的 episodes，这非常充足！
        # 之前 50000 只能存 16 个。这是质的飞跃。
        buffer_size=200000, 
        min_buffer_size=1000,
        
        batch_size=256,
        learning_rate=3e-4,
        
        # 探索衰减
        # 2000 episodes * 60 steps = 120,000 steps total
        epsilon_decay_steps=60000 # 在一半的时候停止衰减
    )
    
    agent = SequentialDQNAgent(
        state_dim=env.get_state_dim(),
        action_dim=env.get_action_dim(),
        config=dqn_config
    )
    
    # 3. 训练器
    trainer = SequentialTrainer(env, agent, TrainConfig(
        num_episodes=num_episodes,
        eval_freq=50,
        save_freq=200
    ))
    
    stats = trainer.train()
    return stats


# ===========================
# 测试
# ===========================

if __name__ == "__main__":
    sys.path.insert(0, '..')
    from SystemModel.test_phase1 import create_test_scenario
    print("Testing Sequential Trainer...")
    np.random.seed(42)

    # 创建测试系统
    #system = create_test_scenario(num_users=500, num_uavs=10, hard_ratio=0.6)

    system = SystemMd(
        user_file=r"./data/train_data/1_5000users_data_230717.csv",
        uav_file=f"./data/train_data/1_20uavs_loc_230717.csv",
        config_file="./SystemModel/def_config.json"
    )

    # 快速训练测试
    stats = run_training(system, num_episodes=5000, verbose=0)

    print(f"\nTraining completed:")
    print(f"  Episodes: {len(stats.episode_rewards)}")
    print(f"  Best eval utility: {stats.best_eval_utility:.4f}")
    if stats.episode_rewards:
        print(f"  Final avg reward (last 20): "
              f"{np.mean(stats.episode_rewards[-20:]):.4f}")
    if stats.episode_feasible:
        print(f"  Final feasible rate (last 20): "
              f"{np.mean(stats.episode_feasible[-20:])*100:.1f}%")

    print("\n✅ Sequential Trainer test completed!")