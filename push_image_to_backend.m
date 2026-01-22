function push_image_to_backend(image_matrix)
    % 推送图像到后端（base64编码）
    
    base_url = 'http://192.168.110.19:8000';
    
    % 获取脚本所在目录
    script_dir = fileparts(mfilename('fullpath'));
    temp_file = fullfile(script_dir, 'temp_image.jpg');
    
    % 将图像转换为base64
    imwrite(image_matrix, temp_file);
    fid = fopen(temp_file, 'rb');
    image_bytes = fread(fid, inf, 'uint8=>uint8');
    fclose(fid);
    image_base64 = matlab.net.base64encode(image_bytes);
    
    % 构建数据
    data = struct();
    data.image_base64 = char(image_base64);
    data.timestamp = posixtime(datetime('now'));
    
    % 发送请求
    options = weboptions(...
        'RequestMethod', 'post', ...
        'MediaType', 'application/json', ...
        'Timeout', 10);
    
    try
        response = webwrite([base_url '/upload_image_base64'], data, options);
        fprintf('[%s] 图像推送成功\n', datestr(now, 'HH:MM:SS'));
    catch ME
        fprintf('[%s] 图像推送失败: %s\n', ...
            datestr(now, 'HH:MM:SS'), ME.message);
    end
    
    % 清理临时文件
    if exist(temp_file, 'file')
        delete(temp_file);
    end
end