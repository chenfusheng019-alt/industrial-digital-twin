%% 测试图像推送脚本
% 用于测试网页图像显示功能

clc;
clear;

fprintf('========================================\n');
fprintf('  测试图像推送\n');
fprintf('========================================\n\n');

%% 配置
BACKEND_URL = 'http://localhost:8000';
fprintf('后端地址: %s\n\n', BACKEND_URL);

%% 检查连接
try
    response = webread([BACKEND_URL '/health']);
    fprintf('✓ 后端服务器连接成功\n\n');
catch ME
    fprintf('✗ 无法连接到后端服务器\n');
    fprintf('  请确保 server.py 正在运行\n\n');
    return;
end

%% 方法1：推送现有图像文件
fprintf('[方法1] 推送现有图像文件...\n');

% 检查是否有机械臂图片
if exist('机械臂.png', 'file')
    try
        img = imread('机械臂.png');
        push_image_to_backend(img);
        fprintf('  ✓ 机械臂图片推送成功\n\n');
    catch ME
        fprintf('  ✗ 推送失败: %s\n\n', ME.message);
    end
else
    fprintf('  ⚠ 未找到机械臂.png文件\n\n');
end

%% 方法2：生成测试图像
fprintf('[方法2] 生成并推送测试图像...\n');

try
    % 创建一个带文字的测试图像
    test_img = uint8(ones(480, 640, 3) * 200);  % 浅灰色背景
    
    % 添加一些图形元素
    test_img(100:380, 100:540, 1) = 100;  % 红色矩形
    test_img(100:380, 100:540, 2) = 150;
    test_img(100:380, 100:540, 3) = 200;
    
    % 保存为临时文件
    imwrite(test_img, 'test_robot_image.jpg');
    
    % 推送图像
    push_image_to_backend(test_img);
    
    fprintf('  ✓ 测试图像推送成功\n');
    fprintf('  提示: 请在网页上查看右侧的"实时图像"区域\n\n');
    
catch ME
    fprintf('  ✗ 推送失败: %s\n\n', ME.message);
end

%% 方法3：持续推送动态图像
fprintf('[方法3] 持续推送动态图像（10次）...\n');
fprintf('  按 Ctrl+C 可以提前停止\n\n');

try
    for i = 1:10
        % 创建动态变化的图像
        img = uint8(randi([50, 200], 480, 640, 3));
        
        % 添加一个移动的方块
        x_pos = mod(i * 50, 540) + 50;
        y_pos = 200;
        img(y_pos:y_pos+80, x_pos:x_pos+80, 1) = 255;  % 红色方块
        img(y_pos:y_pos+80, x_pos:x_pos+80, 2) = 0;
        img(y_pos:y_pos+80, x_pos:x_pos+80, 3) = 0;
        
        % 推送图像
        push_image_to_backend(img);
        
        fprintf('  第 %2d 次推送完成\n', i);
        
        pause(0.5);  % 每0.5秒推送一次
    end
    
    fprintf('\n  ✓ 动态图像推送测试完成\n\n');
    
catch ME
    fprintf('\n  ✗ 推送失败: %s\n\n', ME.message);
end

%% 总结
fprintf('========================================\n');
fprintf('  测试完成\n');
fprintf('========================================\n');
fprintf('请在网页上检查:\n');
fprintf('  1. 右侧"实时图像"区域是否显示图像\n');
fprintf('  2. 图像是否随推送而更新\n\n');
fprintf('如果图像未显示，请:\n');
fprintf('  1. 按F12打开浏览器控制台查看错误\n');
fprintf('  2. 检查后端服务器日志\n');
fprintf('  3. 确认 push_image_to_backend.m 中的URL正确\n\n');
