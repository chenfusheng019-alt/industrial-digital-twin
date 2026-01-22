% 測試實時數據推送到後端
% 模擬 APP 運行時的數據推送，檢查網頁是否實時更新

fprintf('========================================\n');
fprintf('  實時數據推送測試\n');
fprintf('========================================\n\n');

% 後端地址
backend_url = 'http://192.168.110.19:8000';

fprintf('後端地址: %s\n\n', backend_url);

% 測試連接
fprintf('[測試 1/3] 檢查後端連接...\n');
try
    response = webread([backend_url '/health']);
    fprintf('  ✓ 後端連接正常\n');
    fprintf('    連接數: %d\n\n', response.connections);
catch ME
    fprintf('  ✗ 後端連接失敗: %s\n', ME.message);
    return;
end

% 模擬訓練數據推送（連續推送 20 次，模擬真實訓練）
fprintf('[測試 2/3] 模擬訓練數據推送（20次）...\n');
fprintf('  請觀察網頁上的獎勵曲線是否實時變化\n\n');

for i = 1:20
    % 模擬變化的數據
    state = [80 + randi([-10, 10]), ...
             815 + randi([-20, 20]), ...
             665 + randi([-20, 20]), ...
             1];
    
    % 獎勵值逐漸變化（從 -5 到 -3）
    reward = -5.0 + (i / 20) * 2.0 + randn() * 0.1;
    
    mode = 'TRAINING';
    episode = 1;
    step = i;
    
    % 推送數據
    try
        push_to_backend(state, reward, mode, episode, step);
        fprintf('  [%2d/20] Step %2d, 獎勵: %6.2f ✓\n', i, step, reward);
    catch ME
        fprintf('  [%2d/20] 推送失敗: %s\n', i, ME.message);
    end
    
    % 每次推送間隔 0.5 秒
    pause(0.5);
end

fprintf('\n[測試 3/3] 推送測試圖像...\n');
try
    % 創建一個測試圖像（彩色漸變）
    test_img = uint8(zeros(400, 400, 3));
    for i = 1:400
        for j = 1:400
            test_img(i, j, 1) = uint8(i / 400 * 255);  % 紅色通道
            test_img(i, j, 2) = uint8(j / 400 * 255);  % 綠色通道
            test_img(i, j, 3) = uint8((i+j) / 800 * 255);  % 藍色通道
        end
    end
    
    push_image_to_backend(test_img);
    fprintf('  ✓ 測試圖像推送成功\n\n');
catch ME
    fprintf('  ✗ 圖像推送失敗: %s\n\n', ME.message);
end

fprintf('========================================\n');
fprintf('  測試完成\n');
fprintf('========================================\n\n');

fprintf('請檢查網頁:\n');
fprintf('  1. 獎勵值是否從 -5.00 逐漸變化到 -3.00\n');
fprintf('  2. 獎勵曲線是否顯示上升趨勢\n');
fprintf('  3. 實時圖像是否顯示彩色漸變\n');
fprintf('  4. 關節角度是否有小幅度變化\n\n');

fprintf('如果網頁沒有變化，請:\n');
fprintf('  1. 按 F12 打開瀏覽器控制台，查看是否有錯誤\n');
fprintf('  2. 檢查 WebSocket 連接狀態（應該顯示綠色"已連接"）\n');
fprintf('  3. 按 Ctrl+F5 強制刷新網頁\n');
fprintf('========================================\n');
