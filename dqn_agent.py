import numpy as np
import tensorflow as tf
import tensorflow.keras as keras
import json
import sys
import time
import os
import random


class DQNAgent:
    def __init__(self, state_dim, action_dim):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.memory = []
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.batch_size = 32
        self.memory_size = 10000  # 经验回放缓冲区大小
        
        # 训练相关参数
        self.target_update_frequency = 10
        self.last_target_update = 0
        self.total_steps = 0
        self.episode_rewards = []  # 记录每个episode的总奖励
        self.episode_lengths = []  # 记录每个episode的长度
        
        # 网络
        self.main_network = self._build_model()
        self.target_network = self._build_model()
        self.update_target_network()
        
        print("DQNAgent initialized with target update frequency:", self.target_update_frequency)

    def _build_model(self):
        model = keras.Sequential([
            keras.layers.Dense(24, input_dim=self.state_dim, activation='relu', name='dense_1'),
            keras.layers.Dense(24, activation='relu', name='dense_2'),
            keras.layers.Dense(self.action_dim, activation='linear', name='dense_3')
        ])
        model.compile(loss='mse', optimizer=keras.optimizers.Adam(learning_rate=self.learning_rate))
        return model

    def update_target_network(self):
        print("Updating target network...")
        self.target_network.set_weights(self.main_network.get_weights())
        print("Target network updated successfully")

    def get_action(self, state):
        if np.random.rand() <= self.epsilon:
            return np.random.randint(self.action_dim)
        act_values = self.main_network.predict(state.reshape(1, -1), verbose=0)
        return np.argmax(act_values[0])

    def train(self, state, action, reward, next_state, done, current_step, episode, episode_step):
        self.total_steps = current_step
        
        # 存储经验
        if len(self.memory) >= self.memory_size:
            self.memory.pop(0)  # 如果内存满了，移除最旧的经验
        self.memory.append((state, action, reward, next_state, done))
        
        # 训练主网络
        if len(self.memory) > self.batch_size:
            self._replay()
        
        # 检查是否需要更新目标网络
        should_update_target = (current_step - self.last_target_update) >= self.target_update_frequency
        
        if should_update_target:
            print(f"Step {current_step}: Updating target network")
            self.update_target_network()
            self.last_target_update = current_step
        
        # 如果是episode结束，记录统计信息
        if done:
            episode_reward = sum([exp[2] for exp in self.memory[-episode_step:]])
            self.episode_rewards.append(episode_reward)
            self.episode_lengths.append(episode_step)
            
            # 保存训练统计信息
            self._save_training_stats(episode, episode_reward, episode_step)
        
        return should_update_target

    def _save_training_stats(self, episode, reward, length):
        """保存训练统计信息"""
        stats = {
            'episode': episode,
            'reward': reward,
            'length': length,
            'epsilon': self.epsilon,
            'total_steps': self.total_steps
        }
        
        # 保存到JSON文件
        filename = f'training_stats_episode_{episode}.json'
        with open(filename, 'w') as f:
            json.dump(stats, f)
        
        # 计算并打印平均奖励
        if len(self.episode_rewards) >= 10:
            avg_reward = np.mean(self.episode_rewards[-10:])
            print(f"Episode {episode} finished. Average reward over last 10 episodes: {avg_reward:.2f}")

    def _replay(self):
        """经验回放训练"""
        minibatch = random.sample(self.memory, self.batch_size)
        
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = reward + self.gamma * np.amax(
                    self.target_network.predict(next_state.reshape(1, -1), verbose=0)[0]
                )
            
            target_f = self.main_network.predict(state.reshape(1, -1), verbose=0)
            target_f[0][action] = target
            self.main_network.fit(state.reshape(1, -1), target_f, epochs=1, verbose=0)
        
        # 衰减探索率
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def get_weights(self):
        weights = self.main_network.get_weights()
        # 将所有权重转换为float32类型并展平
        weights_list = []
        for w in weights:
            # 确保权重是float32类型
            w_float32 = w.astype(np.float32)
            weights_list.extend(w_float32.flatten().tolist())
        return weights_list


# 创建全局agent实例
agent = DQNAgent(state_dim=4, action_dim=6)


def process_step(data):
    try:
        print("Processing step data:", data)
        
        # 解析数据
        state = np.array(data[0:4], dtype=np.float32)
        action = int(data[4])
        reward = float(data[5])
        done = bool(data[6])
        episode = int(data[7])
        episode_step = int(data[8])
        current_step = agent.total_steps + 1
        
        print(f"Episode {episode}, Step {episode_step} - State: {state.tolist()}, "
              f"Action: {action}, Reward: {reward}, Done: {done}")
        
        # 训练网络
        should_update_target = agent.train(state, action, reward, state, done, 
                                         current_step, episode, episode_step)
        
        # 获取下一个动作
        next_action = int(agent.get_action(state))
        print(f"Next action: {next_action}")
        
        # 获取网络权重
        weights = agent.get_weights()
        
        # 保存结果
        result = {
            'action': next_action,
            'weights': weights,
            'should_update_target': should_update_target,
            'current_step': current_step,
            'episode': episode,
            'episode_step': episode_step,
            'epsilon': agent.epsilon
        }
        
        # 将结果写到脚本目录下的 dqn_result.json（使用绝对路径）
        result_file = os.path.join(script_dir, 'dqn_result.json')
        with open(result_file, 'w') as f:
            json.dump(result, f)
        
    except Exception as e:
        print(f"Error in process_step: {e}", file=sys.stderr)
        print(f"Error details: {str(e)}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    try:
        print("Starting Python script")
        print("Current working directory:", os.getcwd())
        
        # 获取脚本所在目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        print(f"Script directory: {script_dir}")
        
        # 切换到脚本所在目录
        os.chdir(script_dir)
        print(f"Changed working directory to: {os.getcwd()}")

        # 使用脚本所在目录的绝对路径来读取 step_data.json，避免工作目录不一致的问题
        step_file = os.path.join(script_dir, 'step_data.json')
        # 等待 step_data.json 出现，处理可能的文件系统延迟或 race condition
        max_wait = 5.0
        waited = 0.0
        wait_step = 0.05
        while not os.path.exists(step_file) and waited < max_wait:
            time.sleep(wait_step)
            waited += wait_step

        if not os.path.exists(step_file):
            print(f"Error: {step_file} not found after waiting {max_wait} seconds", file=sys.stderr)
            sys.exit(1)

        print(f"Reading step_data.json from: {step_file}")
        with open(step_file, 'r') as f:
            data = json.load(f)
        print(f"Loaded data: {data}")

        # 确保数据是Python原生类型，并处理episode字段
        state_data = [float(x) for x in data['state']]
        action_data = int(data['action'])
        reward_data = float(data['reward'])
        done_data = bool(data['done'])
        
        # 处理episode字段，确保它是整数
        if isinstance(data['episode'], list):
            episode_data = 1  # 如果是列表，使用默认值1
        else:
            try:
                episode_data = int(float(data['episode']))  # 先转换为float再转为int
            except (ValueError, TypeError):
                episode_data = 1  # 如果转换失败，使用默认值1
                
        episode_step_data = int(float(data['episode_step']))  # 确保是整数

        # 打印处理后的数据
        print(f"Processed data:")
        print(f"State: {state_data}")
        print(f"Action: {action_data}")
        print(f"Reward: {reward_data}")
        print(f"Done: {done_data}")
        print(f"Episode: {episode_data}")
        print(f"Episode step: {episode_step_data}")

        process_step(state_data + [action_data, reward_data, done_data, episode_data, episode_step_data])
        print("Script completed successfully")

    except Exception as e:
        print(f"Error in main: {e}", file=sys.stderr)
        print(f"Error details: {str(e)}", file=sys.stderr)
        sys.exit(1)