% 测试 Python 环境的诊断脚本
clc, clear

disp('=== Python 环境诊断 ===');
disp(' ');

% 步骤 1: 检查当前 Python 环境
disp('步骤 1: 检查当前 Python 环境');
pe = pyenv;
disp(pe);
disp(' ');

% 步骤 2: 设置 Python 路径
python_path = 'C:\Users\28902\.conda\envs\RL_ROS\python.exe';
disp(['步骤 2: 设置 Python 路径为: ' python_path]);

% 检查文件是否存在
if isfile(python_path)
    disp('✓ Python 可执行文件存在');
else
    error('✗ Python 可执行文件不存在');
end

% 检查 DLL 是否存在
dll_path = 'C:\Users\28902\.conda\envs\RL_ROS\python39.dll';
if isfile(dll_path)
    disp('✓ Python DLL 文件存在');
else
    warning('✗ Python DLL 文件不存在: %s', dll_path);
end
disp(' ');

% 步骤 3: 尝试设置环境（如果需要）
disp('步骤 3: 配置 Python 环境');
if pe.Version ~= "3.9" || pe.Executable ~= python_path
    disp('需要重新配置 Python 环境...');
    pe = pyenv('Version', python_path);
end
disp(pe);
disp(' ');

% 步骤 4: 测试 Python 是否可以执行（这会触发实际加载）
disp('步骤 4: 测试 Python 执行能力');
try
    % 执行一个简单的 Python 命令
    result = py.sys.version;
    disp('✓ Python 成功执行命令!');
    disp(['  Python 版本信息: ' char(result)]);
    
    % 再次检查状态
    pe_after = pyenv;
    disp(' ');
    disp('执行命令后的 Python 环境状态:');
    disp(pe_after);
    
    if pe_after.Status == "Loaded"
        disp('✓ Python 环境已成功加载!');
    else
        warning('Python 状态仍然是: %s', pe_after.Status);
    end
    
catch ME
    disp('✗ Python 执行失败!');
    disp(['  错误类型: ' ME.identifier]);
    disp(['  错误信息: ' ME.message]);
    disp(' ');
    
    % 提供详细的诊断信息
    disp('可能的原因:');
    disp('1. Python 环境损坏 - 尝试重新创建 conda 环境');
    disp('2. 缺少依赖的 DLL - 检查 conda 环境的 Library\bin 目录');
    disp('3. MATLAB 和 Python 位数不匹配 - 检查都是 64 位');
    disp('4. 防病毒软件阻止 - 暂时禁用试试');
    disp(' ');
    
    % 显示完整错误堆栈
    disp('完整错误堆栈:');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end

disp(' ');
disp('=== 诊断完成 ===');

