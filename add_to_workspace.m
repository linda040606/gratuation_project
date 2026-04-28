%% 添加To Workspace模块到模型
clear; clc;

fprintf('=== 添加To Workspace模块 ===\n\n');

model = 'missile_closed_loop';
projectDir = 'C:\Users\25516\Desktop\graduationproject\simulink\test2';
cd(projectDir);

% 加载模型
if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('模型已打开\n\n');

%% 检查现有的输出端口
fprintf('检查输出端口:\n');

% 检查Water_Dynamics的输出端口
wd_theta = [model '/Water_Dynamics/theta_out'];
wd_omega = [model '/Water_Dynamics/omega_out'];

% 使用 get_param 检查模块是否存在
try
    get_param(wd_theta, 'Position');
    fprintf('  ✓ Water_Dynamics/theta_out 存在\n');
catch
    fprintf('  ✗ Water_Dynamics/theta_out 不存在\n');
end

try
    get_param(wd_omega, 'Position');
    fprintf('  ✓ Water_Dynamics/omega_out 存在\n');
catch
    fprintf('  ✗ Water_Dynamics/omega_out 不存在\n');
end

% 检查Model_Out
model_out = [model '/Model_Out'];
try
    get_param(model_out, 'Position');
    fprintf('  ✓ Model_Out 存在\n');
catch
    fprintf('  ✗ Model_Out 不存在\n');
end

%% 添加To Workspace模块
fprintf('\n添加To Workspace模块...\n');

% 删除旧的（如果存在）
try
    get_param([model '/theta_out'], 'Position');
    delete_block([model '/theta_out']);
    fprintf('  删除旧的theta_out模块\n');
catch
    % 模块不存在，跳过
end

% 添加新的To Workspace模块
add_block('simulink/Sinks/To Workspace', [model '/theta_out'], ...
    'Position', [1200, 240, 1260, 270]);

% 设置参数
set_param([model '/theta_out'], ...
    'VariableName', 'theta_sim', ...
    'SaveFormat', 'Structure With Time');

fprintf('  ✓ To Workspace模块已添加\n');

%% 连接到Water_Dynamics的theta_out
fprintf('\n连接信号...\n');

try
    % 连接Water_Dynamics的theta_out到theta_out模块
    add_line(model, 'Water_Dynamics/2', 'theta_out/1', 'Autorouting', 'on');
    fprintf('  ✓ 连线成功: Water_Dynamics/2 -> theta_out\n');
catch ME
    fprintf('  ✗ 连线失败: %s\n', ME.message);
end

%% 保存模型
fprintf('\n保存模型...\n');
try
    save_system(model);
    fprintf('  ✓ 模型已保存\n\n');
catch ME
    fprintf('  ✗ 保存失败: %s\n\n', ME.message);
end

fprintf('=== 完成 ===\n');
fprintf('现在模型结构:\n');
fprintf('  Water_Dynamics/2 (theta_out) -> theta_out (To Workspace)\n');
fprintf('\n下一步:\n');
fprintf('运行: run_and_analyze_simple\n');
