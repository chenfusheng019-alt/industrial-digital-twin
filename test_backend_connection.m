%% 测试后端连接脚本
% 用于验证 MATLAB 到后端服务器的数据传输

clc;
clear;

fprintf('========================================\n');
fprintf('  机械臂数字孪生 - 后端连接测试\n');
fprintf('========================================\n\n');

%% 配置
BACKEND_URL = 'http://localhost:8000';  % 修改为您的实际IP
fprintf('后端地址: %s\n\n', BACKEND_URL);

%% 测试 1: 健康检查
fprintf('[测试 1/5] 健康检查...\n');
try
    response = webread([BACKEND_URL '/health']);
    fprintf('  ✓ 后端服务器运行正常\n');
    fprintf('    连接数: %d\n', response.connections);
    fprintf('    时间戳: %.2f\n\n', response.timestamp);
    test1_pass = true;
catch ME
    fprintf('  ✗ 连接失败: %s\n\n', ME.message);
    fprintf('  请检查:\n');
    fprintf('  1. server.py 是否正在运行\n');
    fprintf('  2. 防火墙是否允许端口 8000\n');
    fprintf('  3. IP 地址是否正确\n\n');
    test1_pass = false;
    return;
end

%% 测试 2: 获取当前状态
fprintf('[测试 2/5] 获取当前状态...\n');
try
    state = webread([BACKEND_URL '/state']);
    fprintf('  ✓ 状态获取成功\n');
    fprintf('    模式: %s\n', state.mode);
    fprintf('    奖励: %.2f\n', state.reward);
    fprintf('    关节数: %d\n\n', length(state.joints));
    test2_pass = true;
catch ME
    fprintf('  ✗ 获取状态失败: %s\n\n', ME.message);
    test2_pass = false;
end

%% 测试 3: 推送数据
fprintf('[测试 3/5] 推送测试数据...\n');
try
    % 构造测试数据
    test_state = [100, 200, 300, 400];  % 4个关节
    test_reward = 0.85;
    test_mode = 'testing';
    test_episode = 1;
    test_step = 10;
    
    % 调用推送函数
    push_to_backend(test_state, test_reward, test_mode, test_episode, test_step);
    
    % 等待一下
    pause(0.5);
    
    % 验证数据是否更新
    state = webread([BACKEND_URL '/state']);
    
    if strcmp(state.mode, test_mode) && abs(state.reward - test_reward) < 0.01
        fprintf('  ✓ 数据推送成功\n');
        fprintf('    验证: 模式=%s, 奖励=%.2f\n\n', state.mode, state.reward);
        test3_pass = true;
    else
        fprintf('  ⚠ 数据推送但验证失败\n');
        fprintf('    期望: 模式=%s, 奖励=%.2f\n', test_mode, test_reward);
        fprintf('    实际: 模式=%s, 奖励=%.2f\n\n', state.mode, state.reward);
        test3_pass = false;
    end
catch ME
    fprintf('  ✗ 推送失败: %s\n\n', ME.message);
    test3_pass = false;
end

%% 测试 4: 推送图像
fprintf('[测试 4/5] 推送测试图像...\n');
try
    % 创建测试图像
    test_image = uint8(randi([0, 255], 480, 640, 3));
    
    % 推送图像
    push_image_to_backend(test_image);
    
    fprintf('  ✓ 图像推送成功\n\n');
    test4_pass = true;
catch ME
    fprintf('  ✗ 图像推送失败: %s\n\n', ME.message);
    test4_pass = false;
end

%% 测试 5: 连续推送（模拟实时数据）
fprintf('[测试 5/5] 连续推送测试（10次）...\n');
try
    success_count = 0;
    for i = 1:10
        % 生成随机数据
        state = [100, 200, 300, 400] + randn(1, 4) * 10;
        reward = 0.5 + randn() * 0.2;
        
        % 推送
        push_to_backend(state, reward, 'testing', 1, i);
        
        success_count = success_count + 1;
        fprintf('  第 %2d 次推送成功\n', i);
        
        pause(0.1);  % 100ms 间隔
    end
    
    fprintf('  ✓ 连续推送测试完成 (%d/10)\n\n', success_count);
    test5_pass = (success_count == 10);
catch ME
    fprintf('  ✗ 连续推送失败: %s\n\n', ME.message);
    test5_pass = false;
end

%% 测试总结
fprintf('========================================\n');
fprintf('  测试总结\n');
fprintf('========================================\n');

if test1_pass
    fprintf('  [✓] 测试 1: 健康检查\n');
else
    fprintf('  [✗] 测试 1: 健康检查\n');
end

if test2_pass
    fprintf('  [✓] 测试 2: 获取状态\n');
else
    fprintf('  [✗] 测试 2: 获取状态\n');
end

if test3_pass
    fprintf('  [✓] 测试 3: 推送数据\n');
else
    fprintf('  [✗] 测试 3: 推送数据\n');
end

if test4_pass
    fprintf('  [✓] 测试 4: 推送图像\n');
else
    fprintf('  [✗] 测试 4: 推送图像\n');
end

if test5_pass
    fprintf('  [✓] 测试 5: 连续推送\n');
else
    fprintf('  [✗] 测试 5: 连续推送\n');
end

fprintf('========================================\n\n');

total_pass = test1_pass + test2_pass + test3_pass + test4_pass + test5_pass;
fprintf('总计: %d/5 测试通过\n\n', total_pass);

if total_pass == 5
    fprintf('🎉 所有测试通过！系统运行正常。\n\n');
    fprintf('下一步:\n');
    fprintf('  1. 打开网页: file:///d:/matlab/代码/Github/index.html\n');
    fprintf('  2. 检查 WebSocket 连接状态\n');
    fprintf('  3. 运行 receive_and_control.m 开始实时传输\n\n');
else
    fprintf('⚠️  部分测试失败，请检查配置。\n\n');
    fprintf('故障排查:\n');
    fprintf('  1. 确保 server.py 正在运行\n');
    fprintf('  2. 检查 push_to_backend.m 中的 URL 配置\n');
    fprintf('  3. 检查防火墙设置\n');
    fprintf('  4. 查看后端服务器日志\n\n');
end

fprintf('========================================\n');
