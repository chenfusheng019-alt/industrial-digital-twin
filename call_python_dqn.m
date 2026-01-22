function [action, weights, should_update_target] = call_python_dqn(data_struct, current_step)
    % 获取当前函数所在目录（而不是工作目录）
    script_dir = fileparts(mfilename('fullpath'));
    
    % 将数据保存为JSON文件（保存到脚本所在目录）
    json_file = fullfile(script_dir, 'step_data.json');
    fid = fopen(json_file, 'w');
    if fid == -1
        error('无法创建step_data.json文件');
    end
    fprintf(fid, jsonencode(data_struct));
    fclose(fid);
    
    % 使用完整路径调用Python脚本
    python_path = 'C:\Users\86156\.conda\envs\RL_ROS\python.exe';
    script_path = fullfile(script_dir, 'dqn_agent.py');
    
    % 构建完整的命令：切换到脚本目录再运行 Python，避免工作目录导致的文件找不到
    cmd = sprintf('cd /d "%s" && "%s" "%s"', script_dir, python_path, script_path);
    
    % 调用Python脚本并获取详细输出
    [status, result] = system(cmd);
    disp('Python script output:');
    disp(result);
    
    if status ~= 0
        error(['Python脚本执行失败: ', result]);
    end
    
    % 等待Python脚本生成结果文件（在脚本所在目录）
    result_file = fullfile(script_dir, 'dqn_result.json');
    max_wait = 10; % 最大等待秒数
    wait_time = 0;
    while ~exist(result_file, 'file') && wait_time < max_wait
        pause(0.1);
        wait_time = wait_time + 0.1;
    end
    
    if ~exist(result_file, 'file')
        error(['等待Python结果文件超时。Python输出: ', result]);
    end
    
    % 读取Python返回的结果
    try
        fid = fopen(result_file, 'r');
        if fid == -1
            error(['无法打开dqn_result.json文件。文件是否存在: ', num2str(exist(result_file, 'file'))]);
        end
        result_data = jsondecode(fread(fid, '*char')');
        fclose(fid);
        
        % 确保返回的数据类型正确
        action = double(result_data.action);
        weights = double(result_data.weights);
        should_update_target = logical(result_data.should_update_target);
        
        % 删除临时文件
        delete(json_file);
        delete(result_file);
    catch e
        disp('Error reading result file:');
        disp(e.message);
        if exist(result_file, 'file')
            disp('File exists but may be corrupted. Contents:');
            type(result_file);
        end
        rethrow(e);
    end
end