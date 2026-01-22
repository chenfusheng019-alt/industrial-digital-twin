% 診斷後端推送問題
clc; clear;

disp('========================================');
disp('  後端推送診斷工具');
disp('========================================');
disp(' ');

% 測試數據（模擬 APP 接收到的數據）
test_state = [70, 815, 665, 1];  % 從您的截圖中獲取
test_reward = -5.05;
test_mode = 'TRAINING';
test_episode = 1;
test_step = 10;

disp('測試數據:');
disp(['  狀態: [' num2str(test_state) ']']);
disp(['  獎勵: ' num2str(test_reward)]);
disp(['  模式: ' test_mode]);
disp(['  Episode: ' num2str(test_episode)]);
disp(['  Step: ' num2str(test_step)]);
disp(' ');

% 測試 1: 檢查後端連接
disp('[測試 1/4] 檢查後端服務器...');
try
    response = webread('http://192.168.110.19:8000/health');
    disp('  ✓ 後端服務器正在運行');
    disp(['    連接數: ' num2str(response.connections)]);
catch ME
    disp('  ✗ 後端服務器無法連接');
    disp(['    錯誤: ' ME.message]);
    disp(' ');
    disp('請確認:');
    disp('  1. 後端服務器是否已啟動 (python app.py)');
    disp('  2. IP 地址是否正確 (192.168.110.19:8000)');
    disp('  3. 防火牆是否允許連接');
    return;
end
disp(' ');

% 測試 2: 推送數據
disp('[測試 2/4] 推送測試數據...');
try
    push_to_backend(test_state, test_reward, test_mode, test_episode, test_step);
    disp('  ✓ 數據推送成功');
    
    % 驗證數據是否被接收
    pause(0.5);
    response = webread('http://192.168.110.19:8000/state');
    disp('  驗證後端狀態:');
    disp(['    模式: ' response.mode]);
    disp(['    獎勵: ' num2str(response.reward)]);
    disp(['    關節數: ' num2str(length(response.joints))]);
    
    if strcmp(response.mode, test_mode) && abs(response.reward - test_reward) < 0.01
        disp('  ✓ 數據驗證成功');
    else
        disp('  ✗ 數據驗證失敗（後端數據與發送數據不匹配）');
    end
catch ME
    disp('  ✗ 數據推送失敗');
    disp(['    錯誤: ' ME.message]);
end
disp(' ');

% 測試 3: 推送圖像
disp('[測試 3/4] 推送測試圖像...');
try
    % 創建測試圖像（簡單的漸變）
    test_image = uint8(zeros(200, 300, 3));
    for i = 1:200
        for j = 1:300
            test_image(i, j, 1) = uint8(i * 255 / 200);
            test_image(i, j, 2) = uint8(j * 255 / 300);
            test_image(i, j, 3) = 128;
        end
    end
    
    push_image_to_backend(test_image);
    disp('  ✓ 圖像推送成功');
catch ME
    disp('  ✗ 圖像推送失敗');
    disp(['    錯誤: ' ME.message]);
end
disp(' ');

% 測試 4: 檢查 WebSocket 連接
disp('[測試 4/4] 檢查 WebSocket 狀態...');
try
    response = webread('http://192.168.110.19:8000/health');
    if response.connections > 0
        disp(['  ✓ WebSocket 已連接 (' num2str(response.connections) ' 個客戶端)']);
    else
        disp('  ⚠ WebSocket 未連接（網頁可能未打開）');
        disp('    請在瀏覽器中打開: http://192.168.110.19:8000');
    end
catch ME
    disp('  ✗ 無法檢查 WebSocket 狀態');
end
disp(' ');

% 總結
disp('========================================');
disp('  診斷完成');
disp('========================================');
disp(' ');
disp('如果所有測試都通過，但網頁仍無數據：');
disp('  1. 檢查瀏覽器控制台（F12）是否有錯誤');
disp('  2. 確認網頁的 WebSocket 連接狀態');
disp('  3. 重新啟動 ArmControlApp');
disp('  4. 檢查 APP 日誌是否顯示「✓ 数据已推送到后端」');
disp(' ');
disp('如果測試失敗：');
disp('  1. 確認後端服務器正在運行');
disp('  2. 檢查 IP 地址和端口配置');
disp('  3. 檢查防火牆設置');
disp('========================================');
