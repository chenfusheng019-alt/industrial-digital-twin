rosshutdown
clc,clear

% 设置Python环境
% 先检查当前Python环境状态
disp('检查当前Python环境...');
current_pe = pyenv;
disp(current_pe);

% 尝试设置Python环境
python_path = 'C:\Users\86156\.conda\envs\RL_ROS\python.exe';
disp(['尝试加载Python: ' python_path]);

% 检查Python可执行文件是否存在
if ~isfile(python_path)
    error('Python可执行文件不存在: %s\n请检查路径是否正确', python_path);
end

try
    % 设置 Python 环境
    pe = pyenv('Version', python_path);
    disp('Python环境配置:');
    disp(pe);
    
    % 注意: NotLoaded 状态是正常的，Python 是懒加载的
    % 只有在第一次执行 Python 命令时才会真正加载
    
    % 测试Python是否可以执行（这会触发实际加载）
    disp('测试 Python 执行能力...');
    test_result = py.sys.version;
    disp(['✓ Python 加载成功! 版本: ' char(test_result)]);
    
    % 再次检查状态
    pe_loaded = pyenv;
    if pe_loaded.Status == "Loaded"
        disp('✓ Python 环境已激活');
    end
    
catch ME
    fprintf('\n✗ Python环境加载失败!\n');
    fprintf('错误信息: %s\n\n', ME.message);
    fprintf('请尝试以下解决方案:\n');
    fprintf('1. 在 MATLAB 命令窗口运行: terminate(pyenv); clear classes\n');
    fprintf('2. 检查 Python 版本兼容性: pyenv\n');
    fprintf('3. 确认 conda 环境完整性: conda list -n RL_ROS\n');
    fprintf('4. 运行诊断脚本: test_python_env\n\n');
    rethrow(ME);
end

% 初始化ROS节点
rosinit('192.168.110.46');

% 全局变量声明
global rl_action_pub network_update_pub last_state last_action last_reward is_first_step

% 创建订阅者和发布者
rl_data_sub = rossubscriber('/rl_data', 'std_msgs/Float32MultiArray', @rl_data_callback);
rl_action_pub = rospublisher('/rl_action', 'std_msgs/Int32');
network_update_pub = rospublisher('/network_update', 'std_msgs/Float32MultiArray');

last_state = [];
last_action = 0;
last_reward = 0;
is_first_step = true;

% 等待发布者连接
disp('Waiting for subscribers to connect...');
while rl_action_pub.NumSubscribers == 0
    pause(0.1);
end
disp('Subscribers connected!');

disp('Starting main loop...');
while true
    pause(0.1);
end