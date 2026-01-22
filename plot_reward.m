clc,clear
% 读取训练数据
step_data = csvread('training_data.csv');
episode_data = csvread('episode_rewards.csv', 1, 0);  % 跳过标题行

% 提取数据
steps = step_data(:,end);
step_rewards = step_data(:,7);
episodes = episode_data(:,1);
episode_steps = episode_data(:,2);
episode_rewards = episode_data(:,3);
episode_avg_rewards = episode_data(:,4);

% 创建两个子图
figure('Position', [100, 100, 1200, 500]);

% 子图1：步数奖励
subplot(1,2,1);
window_size = 100;
moving_avg = movmean(step_rewards, window_size);
plot(steps, step_rewards, 'b.', 'MarkerSize', 1);  % 原始奖励点
hold on;
plot(steps, moving_avg, 'r-', 'LineWidth', 2);  % 移动平均线
xlabel('Steps');
ylabel('Reward');
title('Step Rewards');
legend('Raw Rewards', 'Moving Average');
grid on;

% 子图2：Episode奖励
subplot(1,2,2);
window_size = 10;
episode_moving_avg = movmean(episode_rewards, window_size);
plot(episodes, episode_rewards, 'b.', 'MarkerSize', 10);  % 原始奖励点
hold on;
plot(episodes, episode_moving_avg, 'r-', 'LineWidth', 2);  % 移动平均线
xlabel('Episodes');
ylabel('Total Reward');
title('Episode Rewards');
legend('Raw Rewards', 'Moving Average');
grid on;

% 保存图像
saveas(gcf, 'training_curves.png');

% 打印统计信息
fprintf('Training Statistics:\n');
fprintf('Total Episodes: %d\n', length(episodes));
fprintf('Total Steps: %d\n', max(steps));
fprintf('Average Episode Reward: %.2f\n', mean(episode_rewards));
fprintf('Best Episode Reward: %.2f\n', max(episode_rewards));
fprintf('Average Episode Length: %.2f steps\n', mean(episode_steps));