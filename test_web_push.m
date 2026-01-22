% 快速測試網頁數據推送
clc; clear;

disp('========================================');
disp('  網頁數據推送測試');
disp('========================================');
disp(' ');

% 測試 1: 檢查函數是否存在
disp('[測試 1/3] 檢查必要函數...');
if exist('push_to_backend.m', 'file')
    disp('  ✓ push_to_backend.m 存在');
else
    disp('  ✗ push_to_backend.m 不存在');
    return;
end

if exist('push_image_to_backend.m', 'file')
    disp('  ✓ push_image_to_backend.m 存在');
else
    disp('  ✗ push_image_to_backend.m 不存在');
    return;
end
disp(' ');

% 測試 2: 推送測試數據
disp('[測試 2/3] 推送測試數據到後端...');
try
    test_state = [500, 450, 550, 100];  % 關節位置和距離
    test_reward = 0.75;
    test_mode = 'TESTING';
    test_episode = 1;
    test_step = 10;
    
    push_to_backend(test_state, test_reward, test_mode, test_episode, test_step);
    disp('  ✓ 數據推送成功');
    disp(['    狀態: [' num2str(test_state) ']']);
    disp(['    獎勵: ' num2str(test_reward)]);
    disp(['    模式: ' test_mode]);
catch ME
    disp(['  ✗ 數據推送失敗: ' ME.message]);
end
disp(' ');

% 測試 3: 推送測試圖像
disp('[測試 3/3] 推送測試圖像到後端...');
try
    % 創建一個簡單的測試圖像（彩色漸變）
    test_image = uint8(zeros(200, 300, 3));
    for i = 1:200
        for j = 1:300
            test_image(i, j, 1) = uint8(i * 255 / 200);  % 紅色通道
            test_image(i, j, 2) = uint8(j * 255 / 300);  % 綠色通道
            test_image(i, j, 3) = 128;                   % 藍色通道
        end
    end
    
    push_image_to_backend(test_image);
    disp('  ✓ 圖像推送成功');
    disp(['    圖像尺寸: ' num2str(size(test_image, 1)) 'x' num2str(size(test_image, 2))]);
catch ME
    disp(['  ✗ 圖像推送失敗: ' ME.message]);
end
disp(' ');

% 總結
disp('========================================');
disp('  測試完成');
disp('========================================');
disp(' ');
disp('下一步:');
disp('  1. 打開網頁檢查數據是否顯示');
disp('  2. 確認關節狀態已更新');
disp('  3. 確認圖像已顯示');
disp('  4. 運行 ArmControlApp 開始實時傳輸');
disp(' ');
disp('網頁地址: http://192.168.110.19:8000');
disp('或: file:///d:/matlab/代码/Github/index.html');
disp('========================================');
