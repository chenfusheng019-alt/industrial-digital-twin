% 测试修复后的 call_python_dqn 函数
clc; clear;

disp('=== 测试 DQN 路径修复 ===');
disp(['当前工作目录: ' pwd]);

% 创建测试数据
test_data = struct();
test_data.state = [500, 500, 500, 100];  % 关节位置和距离
test_data.action = 2;
test_data.reward = 0.5;
test_data.done = false;
test_data.episode = 1;
test_data.episode_step = 5;

disp('测试数据:');
disp(test_data);

try
    disp('调用 call_python_dqn...');
    [action, weights, should_update] = call_python_dqn(test_data, 5);
    
    disp('✓ 调用成功!');
    disp(['  下一个动作: ' num2str(action)]);
    disp(['  权重数量: ' num2str(length(weights))]);
    disp(['  是否更新目标网络: ' num2str(should_update)]);
    
catch ME
    disp('✗ 调用失败!');
    disp(['  错误信息: ' ME.message]);
    disp('  错误堆栈:');
    for i = 1:length(ME.stack)
        disp(['    ' ME.stack(i).file ' (行 ' num2str(ME.stack(i).line) ')']);
    end
end

disp('=== 测试完成 ===');
