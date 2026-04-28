%% 简化版：运行仿真并分析性能
% 使用方法：在模型创建后运行此脚本

clear; clc; close all;

fprintf('=== 运行仿真并分析性能 ===\n\n');

model = 'missile_closed_loop';

%% 设置参数
theta_ref = 1;
Kp_theta = 9;
Ki_theta = 0.1;
Kd_theta = 0.8;

Kp0_omega = 50;
Ki0_omega = 0.8;
Kd0_omega = 0.1;

a_omega = 0;
b_omega = 0;
c_theta = 0;

% 物理参数（必需）
J_air = 5;
J_water = 8;
D_air = 0.5;
D_water = 2.0;
K_theta = 1.2;
theta0 = 0;
omega0 = 0;
t_enter = 2.0;
t_blend = 1.0;
tau_eso = 1000;

assignin('base', 'theta_ref', theta_ref);
assignin('base', 'Kp_theta', Kp_theta);
assignin('base', 'Ki_theta', Ki_theta);
assignin('base', 'Kd_theta', Kd_theta);
assignin('base', 'Kp0_omega', Kp0_omega);
assignin('base', 'Ki0_omega', Ki0_omega);
assignin('base', 'Kd0_omega', Kd0_omega);
assignin('base', 'a_omega', a_omega);
assignin('base', 'b_omega', b_omega);
assignin('base', 'c_theta', c_theta);
assignin('base', 'J_air', J_air);
assignin('base', 'J_water', J_water);
assignin('base', 'D_air', D_air);
assignin('base', 'D_water', D_water);
assignin('base', 'K_theta', K_theta);
assignin('base', 'theta0', theta0);
assignin('base', 'omega0', omega0);
assignin('base', 't_enter', t_enter);
assignin('base', 't_blend', t_blend);
assignin('base', 'tau_eso', tau_eso);

fprintf('参数已设置\n\n');

%% 运行仿真
fprintf('运行仿真...\n');

try
    % 运行仿真
    simOut = sim(model, 'SaveOutput', 'off', 'SaveTime', 'off');
    fprintf('仿真完成\n\n');
catch ME
    fprintf('仿真失败: %s\n', ME.message);
    return;
end

%% 检查仿真输出
fprintf('提取theta数据...\n');

data_found = false;
t = [];
theta = [];

% 从SimOut对象提取数据
if isprop(simOut, 'theta_sim')
    fprintf('找到 theta_sim\n');
    try
        theta_sim_data = simOut.theta_sim;
        fprintf('  类型: %s\n', class(theta_sim_data));

        if isstruct(theta_sim_data)
            t = theta_sim_data.time;
            theta = theta_sim_data.signals.values;

            if size(theta, 1) == 1
                theta = theta';
            end

            fprintf('  ✓ 成功提取\n');
            fprintf('  数据点数: %d\n', length(t));
            data_found = true;
        end
    catch ME
        fprintf('  ✗ 提取失败: %s\n', ME.message);
    end
end

if ~data_found
    fprintf('\n✗ 无法提取数据\n');
    fprintf('SimOut 属性:\n');
    props = fieldnames(simOut);
    for i = 1:length(props)
        fprintf('  - %s\n', props{i});
    end
    return;
end

fprintf('\n');

%% 计算性能指标
fprintf('计算性能指标...\n');

% 参考值
theta_ref_val = theta_ref;

% 1. 超调量
[theta_max, idx_max] = max(theta);
Mp = (theta_max - theta_ref_val) / abs(theta_ref_val) * 100;

% 2. 稳态误差
ess = abs(theta(end) - theta_ref_val);

% 3. 调节时间（2%误差带）
band = 0.02 * abs(theta_ref_val);
Ts = NaN;

for i = 1:length(t)
    if all(abs(theta(i:end) - theta_ref_val) <= band)
        Ts = t(i);
        break;
    end
end

if isnan(Ts)
    Ts = t(end);
    converged = false;
else
    converged = true;
end

fprintf('  --- 控制性能指标 ---\n');
fprintf('  超调量 Mp     = %.2f %%\n', Mp);
fprintf('  调节时间 Ts   = %.3f s', Ts);
if ~converged
    fprintf(' (未收敛)\n');
else
    fprintf('\n');
end
fprintf('  稳态误差 ess = %.5f rad (%.2f度)\n', ess, ess*180/pi);
fprintf('  峰值时间     = %.3f s\n\n', t(idx_max));

%% 绘图
fprintf('绘制响应曲线...\n');

figure('Position', [100, 100, 1200, 800]);

subplot(2,1,1);
plot(t, theta, 'b-', 'LineWidth', 1.5);
hold on;
yline(theta_ref_val, '--r', 'LineWidth', 1.5);
yline(theta_ref_val + band, ':g', 'LineWidth', 1);
yline(theta_ref_val - band, ':g', 'LineWidth', 1);
plot(t(idx_max), theta_max, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(t(idx_max), theta_max + 0.1, sprintf('%.1f%%', Mp), ...
    'FontSize', 11, 'Color', 'r', 'FontWeight', 'bold');

if converged
    xline(Ts, '-k', sprintf('%.2fs', Ts), 'LineWidth', 2, ...
        'FontSize', 11, 'FontWeight', 'bold');
end

grid on;
xlabel('时间');
ylabel('\theta (rad)');
title(sprintf('姿态角响应 (Mp=%.1f%%, Ts=%.2fs)', Mp, Ts));
legend({'响应', '参考', '±2%误差带', '峰值'}, 'Location', 'best');

subplot(2,1,2);
plot(t, theta*180/pi, 'b-', 'LineWidth', 1.5);
hold on;
yline(theta_ref_val*180/pi, '--r', 'LineWidth', 1.5);
grid on;
xlabel('时间');
ylabel('\theta (度)');
title('姿态角响应（度）');

fprintf('✓ 完成\n');
