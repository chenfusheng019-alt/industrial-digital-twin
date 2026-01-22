%% 机械臂数字孪生端到端测试脚本
% 测试从MATLAB到网页的完整数据流

clear; clc;

%% 1. 配置后端地址
base_url = 'http://localhost:8000';
fprintf('=== 机械臂数字孪生端到端测试 ===\n');
fprintf('后端地址: %s\n', base_url);

%% 2. 测试连接
fprintf('\n[1] 测试后端连接...\n');
try
    options = weboptions('Timeout', 5);
    response = webread([base_url '/health'], options);
    fprintf('  ✓ 后端连接正常\n');
    fprintf('    状态: %s\n', response.status);
    fprintf('    连接数: %d\n', response.connections);
catch ME
    fprintf('  ✗ 连接失败: %s\n', ME.message);
    fprintf('  请检查:\n');
    fprintf('  1. server.py是否在运行\n');
    fprintf('  2. 防火墙是否允许端口8000\n');
    return;
end

%% 3. 初始化测试数据
fprintf('\n[2] 准备测试数据...\n');

% 模拟6个关节的角度（弧度）
test_joints = [
    0.1,  % 关节1
    0.2,  % 关节2
    0.3,  % 关节3
    0.4,  % 关节4
    0.5,  % 关节5
    0.6   % 关节6
];

% 模拟奖励值
test_reward = 0.85;

% 运行模式
test_mode = 'testing';

% 训练信息
episode = 1;
step = 0;
success_rate = 0.0;

fprintf('  关节角度: [%s]\n', sprintf('%.2f ', test_joints));
fprintf('  奖励值: %.2f\n', test_reward);
fprintf('  模式: %s\n', test_mode);

%% 4. 推送数据到后端
fprintf('\n[3] 推送数据到后端...\n');

% 构建JSON数据
data = struct();
data.joints = test_joints;
data.reward = test_reward;
data.mode = test_mode;
data.episode = episode;
data.step = step;
data.success_rate = success_rate;
data.timestamp = posixtime(datetime('now'));

% 发送POST请求
try
    options = weboptions(...
        'RequestMethod', 'post', ...
        'MediaType', 'application/json', ...
        'Timeout', 5, ...
        'HeaderFields', {'Content-Type' 'application/json'});
    
    response = webwrite([base_url '/update'], data, options);
    fprintf('  ✓ 数据推送成功\n');
    fprintf('    状态: %s\n', response.status);
    fprintf('    消息: %s\n', response.message);
catch ME
    fprintf('  ✗ 数据推送失败: %s\n', ME.message);
end

%% 5. 验证数据是否已更新
fprintf('\n[4] 验证后端数据...\n');
pause(1);  % 等待1秒让后端处理

try
    current_state = webread([base_url '/state']);
    fprintf('  ✓ 获取状态成功\n');
    fprintf('    当前模式: %s\n', current_state.mode);
    fprintf('    当前奖励: %.2f\n', current_state.reward);
    fprintf('    关节角度: [%s]\n', sprintf('%.2f ', current_state.joints));
    
    % 验证数据是否匹配
    if all(abs(current_state.joints - test_joints) < 0.01)
        fprintf('  ✓ 数据验证通过\n');
    else
        fprintf('  ⚠ 数据轻微不匹配（可能是其他更新）\n');
    end
catch ME
    fprintf('  ✗ 获取状态失败: %s\n', ME.message);
end

%% 6. 测试图像上传（可选）
fprintf('\n[5] 测试图像上传...\n');

% 创建测试图像
test_image = uint8(randi([0, 255], 480, 640, 3));

% 保存为临时文件
imwrite(test_image, 'test_image.jpg');

% 读取图像并转换为base64
fid = fopen('test_image.jpg', 'rb');
image_data = fread(fid, inf, 'uint8=>uint8');
fclose(fid);
image_base64 = matlab.net.base64encode(image_data);

% 构建图像数据
image_payload = struct();
image_payload.image_base64 = char(image_base64);
image_payload.timestamp = posixtime(datetime('now'));

try
    % 使用upload_image_base64接口
    img_response = webwrite([base_url '/upload_image_base64'], image_payload, options);
    fprintf('  ✓ 图像上传成功\n');
    fprintf('    状态: %s\n', img_response.status);
catch ME
    fprintf('  ⚠ 图像上传失败（可能是接口未实现）: %s\n', ME.message);
end

% 清理临时文件
if exist('test_image.jpg', 'file')
    delete('test_image.jpg');
end

%% 7. 测试命令轮询（模拟网页控制）
fprintf('\n[6] 测试命令轮询...\n');

% 先检查是否有待处理命令
try
    cmd_response = webread([base_url '/command/latest']);
    if cmd_response.has_command
        fprintf('  ⚠ 发现待处理命令: %s\n', cmd_response.command.action);
    else
        fprintf('  ✓ 当前无待处理命令\n');
    end
catch ME
    fprintf('  ⚠ 命令检查失败: %s\n', ME.message);
end

%% 8. 连续推送测试（模拟实时数据流）
fprintf('\n[7] 开始连续数据推送测试（10次）...\n');
fprintf('  按 Ctrl+C 中断测试\n\n');

for i = 1:10
    try
        % 更新数据
        data.joints = test_joints + randn(1,6) * 0.05;  % 添加随机变化
        data.reward = test_reward + randn() * 0.1;
        data.step = i;
        data.timestamp = posixtime(datetime('now'));
        
        % 推送
        webwrite([base_url '/update'], data, options);
        
        % 获取状态验证
        current = webread([base_url '/state']);
        
        fprintf('  第%2d次: 关节1=%.2f, 奖励=%.2f, 模式=%s\n', ...
            i, current.joints(1), current.reward, current.mode);
        
        pause(0.5);  % 0.5秒间隔
    catch ME
        fprintf('  第%2d次失败: %s\n', i, ME.message);
        break;
    end
end

%% 9. 测试完成
fprintf('\n=== 测试完成 ===\n');
fprintf('请打开浏览器访问:\n');
fprintf('  1. http://localhost:8000/state   查看当前状态\n');
fprintf('  2. http://localhost:8000/docs    API文档\n');
fprintf('  3. 如果部署了网页，访问你的GitHub Pages页面\n');

fprintf('\n下一步建议:\n');
fprintf('  1. 在网页上查看数据是否实时更新\n');
fprintf('  2. 在网页上尝试发送控制命令\n');
fprintf('  3. 在MATLAB中轮询并执行命令\n');