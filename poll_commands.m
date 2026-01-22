function cmd = poll_commands()
    % 轮询后端获取控制命令
    
    base_url = 'http://localhost:8000';
    
    try
        response = webread([base_url '/command/latest']);
        
        if response.has_command
            cmd = response.command;
            fprintf('[%s] 收到命令: %s\n', ...
                datestr(now, 'HH:MM:SS'), cmd.action);
            
            % 根据命令执行相应动作
            execute_command(cmd);
        else
            cmd = [];
        end
    catch ME
        fprintf('[%s] 轮询命令失败: %s\n', ...
            datestr(now, 'HH:MM:SS'), ME.message);
        cmd = [];
    end
end

function execute_command(cmd)
    % 执行接收到的命令
    
    switch cmd.action
        case 'start_training'
            % 开始DQN训练
            start_dqn_training();
            
        case 'stop'
            % 停止当前动作
            stop_motion();
            
        case 'reset'
            % 重置机械臂
            reset_robot();
            
        case 'emergency_stop'
            % 紧急停止
            emergency_stop();
            
        case 'home'
            % 回到零位
            go_home();
            
        case 'move_joint'
            % 移动单个关节
            if isfield(cmd, 'joint_index') && isfield(cmd, 'target_angle')
                move_joint(cmd.joint_index, cmd.target_angle, cmd.speed);
            end
            
        otherwise
            fprintf('未知命令: %s\n', cmd.action);
    end
end