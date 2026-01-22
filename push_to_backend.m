function push_to_backend(state, reward, mode, episode, step)
    % 后端地址（请修改为您的实际IP地址）
    url = 'http://192.168.110.19:8000/update';
    
    % 如果 state 是 4 维，扩展为 6 维（补零）
    if length(state) == 4
        joints = [state(1), state(2), state(3), 0, 0, 0];
    else
        joints = state(:)';
    end
    
    % 构建数据
    data = struct();
    data.joints = joints;
    data.reward = reward;
    data.mode = mode;
    data.episode = episode;
    data.step = step;
    data.timestamp = posixtime(datetime('now'));
    
    % 发送请求
    options = weboptions(...
        'RequestMethod', 'post', ...
        'MediaType', 'application/json', ...
        'Timeout', 3);
    
    try
        response = webwrite(url, data, options);
        % 成功时不打印，避免刷屏
        % fprintf('[%s] 推送成功\n', datestr(now, 'HH:MM:SS'));
    catch ME
        % 只在失败时打印警告
        fprintf('[%s] 推送失败: %s\n', ...
            datestr(now, 'HH:MM:SS'), ME.message);
    end
end