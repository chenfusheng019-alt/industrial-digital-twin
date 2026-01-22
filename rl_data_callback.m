function rl_data_callback(~, msg)
    global last_state last_action last_reward is_first_step rl_action_pub network_update_pub
    global target_update_counter target_update_frequency last_target_update total_steps
    global current_episode current_episode_step episode_reward_sum
    
    % 初始化变量（如果第一次运行）
    if isempty(target_update_counter)
        target_update_counter = 0;
        target_update_frequency = 10;  % 与Python端保持一致
        last_target_update = 0;
        total_steps = 0;
        current_episode = 1;  % 从1开始
        current_episode_step = 0;
        episode_reward_sum = 0;
        
        % 创建episode奖励记录文件（如果不存在）
        if ~exist('episode_rewards.csv', 'file')
            fid = fopen('episode_rewards.csv', 'w');
            fprintf(fid, 'episode,steps,total_reward,avg_reward\n');
            fclose(fid);
        end
        
        % 创建训练数据文件（如果不存在）
        if ~exist('training_data.csv', 'file')
            fid = fopen('training_data.csv', 'w');
            fprintf(fid, 'step,state1,state2,state3,state4,action,reward,done,episode,episode_step\n');
            fclose(fid);
        end
    end

    % 解析接收到的数据并确保转换为正确的类型
    data = msg.Data;
    if iscell(data)
        data = cell2mat(data);
    end
    data = double(data);
    
    % 检查数据长度
    if length(data) < 9
        disp('Warning: Received incomplete data');
        disp(['Data length: ', num2str(length(data))]);
        disp('Data:');
        disp(data);
        return;
    end
    
    current_state = data(1:4);
    current_action = data(5);
    current_reward = data(6);
    is_done = logical(data(7));
    current_episode = int32(data(8));  % 从ROS消息中获取episode
    current_episode_step = int32(data(9));  % 从ROS消息中获取episode_step
    
    % 检查是否是重置信号（action = -1）
    if current_action == -1
        % 重置所有计数器
        current_episode = 1;
        current_episode_step = 0;
        episode_reward_sum = 0;
        total_steps = 0;
        target_update_counter = 0;
        last_target_update = 0;
        is_first_step = true;
        
        disp('Environment reset complete. Starting new training session.');
        
        % 更新状态并继续训练
        last_state = current_state;
        last_action = current_action;
        last_reward = current_reward;
        
        % 准备发送给Python的数据
        data_struct = struct();
        data_struct.state = double(last_state');  % 确保是double类型
        data_struct.action = double(last_action);  % 确保是double类型
        data_struct.reward = double(last_reward);  % 确保是double类型
        data_struct.done = false;  % 重置时done为false
        data_struct.episode = double(current_episode);  % 确保是double类型
        data_struct.episode_step = double(current_episode_step);  % 确保是double类型
        
        % 保存数据到JSON文件
        fid = fopen('step_data.json', 'w');
        if fid == -1
            error('无法创建step_data.json文件');
        end
        fprintf(fid, jsonencode(data_struct));
        fclose(fid);
        
        % 调用Python DQN进行训练和获取动作
        [next_action, weights, should_update_target] = call_python_dqn(data_struct, total_steps);

        % 发送动作到机械臂
        action_msg = rosmessage('std_msgs/Int32');
        action_msg.Data = int32(next_action);
        send(rl_action_pub, action_msg);
        disp('Action sent after reset:');
        disp(next_action);
        
        return;  % 重置后返回，等待下一个状态
    end

    % 累计episode奖励
    episode_reward_sum = episode_reward_sum + current_reward;
    
    % 如果episode结束，保存奖励数据
    if is_done
        % 保存当前episode的奖励数据
        fid = fopen('episode_rewards.csv', 'a');
        if fid ~= -1
            fprintf(fid, '%d,%d,%.4f,%.4f\n', ...
                current_episode, ...
                current_episode_step, ...
                episode_reward_sum, ...
                episode_reward_sum/current_episode_step);
            fclose(fid);
            disp(['Saved episode reward data: Episode ', num2str(current_episode), ...
                  ', Steps: ', num2str(current_episode_step), ...
                  ', Total Reward: ', num2str(episode_reward_sum)]);
        else
            disp('Error: Could not open episode_rewards.csv for writing');
        end
        
        % 重置episode相关变量
        episode_reward_sum = 0;
        
        disp(['Episode ', num2str(current_episode), ' finished. Starting episode ', num2str(current_episode + 1)]);
    end

    % 打印调试信息
    disp('Callback triggered!');
    disp(['Episode: ', num2str(current_episode), ', Step: ', num2str(current_episode_step)]);
    disp(['Total steps: ', num2str(total_steps)]);
    disp(['Episode reward sum: ', num2str(episode_reward_sum)]);
    disp('State:');
    disp(current_state);
    disp('Action:');
    disp(current_action);
    disp('Reward:');
    disp(current_reward);
    disp('Done:');
    disp(is_done);

    % 记录每一步的数据到文件
    try
        fid = fopen('training_data.csv', 'a');
        if fid ~= -1
            fprintf(fid, '%d,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%d,%d,%d\n', ...
                total_steps, ...
                current_state(1), current_state(2), current_state(3), current_state(4), ...
                current_action, ...
                current_reward, ...
                is_done, ...
                current_episode, ...
                current_episode_step);
            fclose(fid);
            disp(['Saved training data for step ', num2str(total_steps)]);
        else
            disp('Error: Could not open training_data.csv for writing');
        end
    catch e
        disp('Error saving training data:');
        disp(e.message);
    end
    
    % 推送数据到后端服务器（实时同步到网页）
    try
        push_to_backend(current_state, current_reward, 'training', current_episode, current_episode_step);
    catch e
        disp(['Warning: Failed to push to backend: ', e.message]);
    end

    % 如果是第一步，只保存数据
    if is_first_step
        last_state = current_state;
        last_action = current_action;
        last_reward = current_reward;
        is_first_step = false;
        disp('First step completed');
        return;
    end

    try
        % 确保数据是double类型
        if iscell(last_state)
            last_state = cell2mat(last_state);
        end
        if iscell(last_action)
            last_action = cell2mat(last_action);
        end
        if iscell(last_reward)
            last_reward = cell2mat(last_reward);
        end
        
        % 准备发送给Python的数据
        data_struct = struct();
        data_struct.state = double(last_state');  % 确保是double类型
        data_struct.action = double(last_action);  % 确保是double类型
        data_struct.reward = double(last_reward);  % 确保是double类型
        data_struct.done = logical(is_done);  % 确保是logical类型
        data_struct.episode = double(current_episode);  % 确保是double类型
        data_struct.episode_step = double(current_episode_step);  % 确保是double类型
        
        % 打印调试信息
        disp('Sending data to Python:');
        disp(['Episode: ', num2str(data_struct.episode)]);
        disp(['Episode step: ', num2str(data_struct.episode_step)]);
        
        % 保存数据到JSON文件
        fid = fopen('step_data.json', 'w');
        if fid == -1
            error('无法创建step_data.json文件');
        end
        fprintf(fid, jsonencode(data_struct));
        fclose(fid);
        
        % 调用Python DQN进行训练和获取动作
        [next_action, weights, should_update_target] = call_python_dqn(data_struct, total_steps);

        % 发送动作到机械臂
        action_msg = rosmessage('std_msgs/Int32');
        action_msg.Data = int32(next_action);
        send(rl_action_pub, action_msg);
        disp('Action sent:');
        disp(next_action);

        % 只在需要时发送网络权重更新
        if should_update_target
            weights_msg = rosmessage('std_msgs/Float32MultiArray');
            if iscell(weights)
                weights = cell2mat(weights);
            end
            weights_msg.Data = double(weights);
            send(network_update_pub, weights_msg);
            disp(['Target network updated at step ', num2str(total_steps)]);
            last_target_update = total_steps;
        else
            disp(['Skipping target network update at step ', num2str(total_steps)]);
        end

        % 更新状态
        last_state = current_state;
        last_action = current_action;
        last_reward = current_reward;
        
        % 更新步数
        total_steps = total_steps + 1;
        
    catch e
        disp('Error in callback:');
        disp(e.message);
        disp('Error details:');
        disp(e);
    end
end