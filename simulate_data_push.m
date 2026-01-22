%% 模拟数据推送测试脚本
% 用于测试网页显示效果，模拟机械臂实时数据

clc;
clear;

fprintf('========================================\n');
fprintf('  模拟机械臂数据推送测试\n');
fprintf('========================================\n\n');

%% 配置
BACKEND_URL = 'http://localhost:8000';  % 修改为您的实际IP
fprintf('后端地址: %s\n', BACKEND_URL);
fprintf('按 Ctrl+C 停止推送\n\n');

%% 检查连接
try
    response = webread([BACKEND_URL '/health']);
    fprintf('✓ 后端服务器连接成功\n\n');
catch ME
    fprintf('✗ 无法连接到后端服务器\n');
    fprintf('  请确保 server.py 正在运行\n\n');
    return;
end

%% 模拟训练数据
episode = 1;
step = 0;
base_reward = 0;

fprintf('开始推送模拟数据...\n\n');

while true
    step = step + 1;
    
    % 模拟关节角度变化（0-1000范围）
    joint1 = 500 + 100 * sin(step * 0.1);
    joint2 = 500 + 150 * cos(step * 0.15);
    joint3 = 500 + 80 * sin(step * 0.2);
    joint4 = 500 + 50 * cos(step * 0.25);
    
    state = [joint1, joint2, joint3, joint4];
    
    % 模拟奖励值变化（逐渐增加，带随机波动）
    base_reward = base_reward + 0.01;
    reward = base_reward + randn() * 0.5;
    
    % 推送数据
    try
        push_to_backend(state, reward, 'training', episode, step);
        
        % 每10步打印一次
        if mod(step, 10) == 0
            fprintf('[Episode %d, Step %3d] 关节1: %.1f°, 关节2: %.1f°, 奖励: %.2f\n', ...
                episode, step, joint1, joint2, reward);
        end
        
    catch ME
        fprintf('推送失败: %s\n', ME.message);
        break;
    end
    
    % 每100步切换到下一个episode
    if mod(step, 100) == 0
        episode = episode + 1;
        step = 0;
        base_reward = 0;
        fprintf('\n--- Episode %d 开始 ---\n\n', episode);
    end
    
    % 推送频率：每秒10次
    pause(0.1);
end

fprintf('\n推送已停止\n');
