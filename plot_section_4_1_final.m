%% 4.1节仿真校验 - 学术级图表生成脚本（最终优化版）

clear; clc; close all;

fprintf('=== 4.1节仿真校验图表生成（最终版）===\n\n');

projectDir = 'C:\Users\25516\Desktop\graduationproject\simulink\test2';
cd(projectDir);

model = 'missile_closed_loop';

%% ===== 步骤1：创建模型并运行仿真 =====

fprintf('步骤1: 创建模型并运行仿真\n');

try
    create_correct_closed_loop_v3;
    fprintf('  ✓ 模型创建成功\n');

    % 启用扰动观测器（修改tau_eso参数）
    tau_eso_new = 0.01;  % 设置为合理的时间常数
    assignin('base', 'tau_eso', tau_eso_new);
    fprintf('  ✓ 扰动观测器已启用 (tau_eso = %.3f)\n', tau_eso_new);
catch ME
    fprintf('  ✗ 模型创建失败: %s\n', ME.message);
    return;
end

%% ===== 步骤2：添加数据导出模块 =====

fprintf('\n步骤2: 添加数据导出模块\n');

signals_to_export = {
    'theta',  'Water_Dynamics', 2, 'theta_sim';
    'omega',  'Water_Dynamics', 1, 'omega_sim';
    'J_t',    'Water_Dynamics', 3, 'J_t_sim';
    'D_t',    'Water_Dynamics', 4, 'D_t_sim';
    'ratio',  'Water_Dynamics', 5, 'ratio_sim';
    'M_pid',  'M_Comp', 1, 'M_pid_sim';
    'd_hat',  'Disturbance_Observer', 1, 'dhat_sim';
};

for i = 1:size(signals_to_export, 1)
    signal_name = signals_to_export{i, 1};
    source_block = signals_to_export{i, 2};
    source_port = signals_to_export{i, 3};
    var_name = signals_to_export{i, 4};

    to_workspace_block = [model '/' signal_name '_out'];

    try
        block_type = get_param(to_workspace_block, 'BlockType');
    catch
        add_block('simulink/Sinks/To Workspace', to_workspace_block, ...
            'Position', [1200, 200+i*50, 1260, 230+i*50]);
        set_param(to_workspace_block, ...
            'VariableName', var_name, ...
            'SaveFormat', 'StructureWithTime');

        try
            add_line(model, [source_block '/' num2str(source_port)], ...
                [signal_name '_out/1'], 'Autorouting', 'on');
        catch
        end
    end
end

fprintf('  ✓ 数据导出模块已添加\n');

%% ===== 步骤3：运行仿真 =====

fprintf('\n步骤3: 运行仿真\n');

try
    simOut = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');
    fprintf('  ✓ 仿真完成\n');
catch ME
    fprintf('  ✗ 仿真失败: %s\n', ME.message);
    return;
end

%% ===== 步骤4：提取数据 =====

fprintf('\n步骤4: 提取数据\n');

data = struct();
for i = 1:size(signals_to_export, 1)
    signal_name = signals_to_export{i, 1};
    var_name = signals_to_export{i, 4};

    try
        if isprop(simOut, var_name)
            signal_data = simOut.(var_name);
            t = signal_data.time;
            signal_values = signal_data.signals.values;

            if size(signal_values, 1) == 1
                signal_values = signal_values';
            end

            data.(signal_name) = signal_values;
            fprintf('  ✓ %s 提取成功\n', signal_name);
        end
    catch
        fprintf('  ✗ %s 提取失败\n', signal_name);
    end
end

theta_ref_val = evalin('base', 'theta_ref');
t_enter_val = evalin('base', 't_enter');

%% ===== 步骤5：计算性能指标 =====

fprintf('\n步骤5: 计算性能指标\n');

[theta_max, idx_max] = max(data.theta);
Mp = (theta_max - theta_ref_val) / abs(theta_ref_val) * 100;

band = 0.02 * abs(theta_ref_val);
Ts = NaN;
for i = 1:length(t)
    if all(abs(data.theta(i:end) - theta_ref_val) <= band)
        Ts = t(i);
        break;
    end
end

ess = abs(data.theta(end) - theta_ref_val);

fprintf('  超调量: %.2f%%, 调节时间: %.3fs, 稳态误差: %.5f rad\n', Mp, Ts, ess);

%% ========== 图表1：姿态角时域响应曲线（4.1.2）==========

fprintf('\n生成图表1: 姿态角时域响应曲线\n');

figure1 = figure('Position', [100, 100, 800, 600]);

color_response = [0.00, 0.45, 0.74];
color_ref = [0.85, 0.33, 0.10];
color_band = [0.47, 0.67, 0.19];
color_peak = [0.60, 0.20, 0.60];

plot(t, data.theta, '-', 'Color', color_response, 'LineWidth', 2.5);
hold on;

yline(theta_ref_val, '--', 'Color', color_ref, 'LineWidth', 1.5);
yline(theta_ref_val + band, ':', 'Color', color_band, 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(theta_ref_val - band, ':', 'Color', color_band, 'LineWidth', 1.2, 'HandleVisibility', 'off');

xline(t_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 2, ...
    'Label', 'Water Entry', 'FontSize', 11, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'bottom');

plot(t(idx_max), theta_max, 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', color_peak, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

text(t(idx_max)*1.02, theta_max + 0.03, ...
    sprintf('Mp = %.1f%%', Mp), ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', color_peak);

if ~isnan(Ts)
    xline(Ts, '-', 'Color', [0.30, 0.30, 0.30], 'LineWidth', 2, ...
        'Label', sprintf('Ts = %.2fs', Ts), 'FontSize', 10, 'FontWeight', 'bold', ...
        'LabelVerticalAlignment', 'bottom');
end

grid on;
set(gca, 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Attitude Angle \theta (rad)', 'FontSize', 14, 'FontWeight', 'bold');
title('Attitude Angle Response', 'FontSize', 16, 'FontWeight', 'bold');

legend({'Response', 'Reference', 'Peak Point'}, ...
    'Location', 'best', 'FontSize', 12, 'Box', 'on');

h_inset = axes('Position', [0.55, 0.2, 0.32, 0.25]);
idx_steady = find(t >= 8 & t <= 10);
plot(t(idx_steady), data.theta(idx_steady), '-', 'Color', color_response, 'LineWidth', 2.5);
hold on;
yline(theta_ref_val, '--', 'Color', color_ref, 'LineWidth', 1.5);
yline(theta_ref_val + band, ':', 'Color', color_band, 'LineWidth', 1.2);
yline(theta_ref_val - band, ':', 'Color', color_band, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('\theta (rad)', 'FontSize', 11);
title('Steady-State Error', 'FontSize', 12, 'FontWeight', 'bold');
set(h_inset, 'FontSize', 10);
box on;

saveas(figure1, 'figure4_1_theta_response.png');
saveas(figure1, 'figure4_1_theta_response.eps');
fprintf('  ✓ 图表1已保存\n');

%% ========== 图表2：PID参数时变曲线（4.1.3 - 三子图）==========

fprintf('\n生成图表2: PID参数时变曲线（三子图）\n');

figure2 = figure('Position', [100, 100, 900, 700]);

color_Kp = [0.85, 0.33, 0.10];
color_Kd = [0.47, 0.67, 0.19];
color_Ki = [0.60, 0.20, 0.60];

Kp0 = evalin('base', 'Kp0_omega');
a_omega = evalin('base', 'a_omega');
Kd0 = evalin('base', 'Kd0_omega');
b_omega = evalin('base', 'b_omega');
Ki0 = evalin('base', 'Ki0_omega');
c_theta = evalin('base', 'c_theta');

Kp_t = Kp0 + a_omega * abs(data.omega);
Kd_t = Kd0 + b_omega * abs(data.omega);
Ki_t = Ki0 * exp(-c_theta * abs(data.theta));

subplot(3, 1, 1);
plot(t, Kp_t, '-', 'Color', color_Kp, 'LineWidth', 1.2);
hold on;
xline(t_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 1.5, ...
    'Label', 'Water Entry', 'FontSize', 10, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'bottom');
grid on;
set(gca, 'FontSize', 11);
ylabel('K_p(t)', 'FontSize', 13, 'FontWeight', 'bold');
title('Proportional Gain Variation', 'FontSize', 13, 'FontWeight', 'bold');
legend({'K_p(t) = K_{p0} + a|\omega|'}, 'Location', 'best', 'FontSize', 10);

subplot(3, 1, 2);
plot(t, Kd_t, '-', 'Color', color_Kd, 'LineWidth', 1.2);
hold on;
xline(t_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 1.5, ...
    'Label', 'Water Entry', 'FontSize', 10, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'bottom');
grid on;
set(gca, 'FontSize', 11);
ylabel('K_d(t)', 'FontSize', 13, 'FontWeight', 'bold');
title('Derivative Gain Variation', 'FontSize', 13, 'FontWeight', 'bold');
legend({'K_d(t) = K_{d0} + b|\omega|'}, 'Location', 'best', 'FontSize', 10);

subplot(3, 1, 3);
plot(t, Ki_t, '-', 'Color', color_Ki, 'LineWidth', 1.2);
hold on;
xline(t_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 1.5, ...
    'Label', 'Water Entry', 'FontSize', 10, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'bottom');
grid on;
set(gca, 'FontSize', 11);
xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('K_i(t)', 'FontSize', 13, 'FontWeight', 'bold');
title('Integral Gain Variation', 'FontSize', 13, 'FontWeight', 'bold');
legend({'K_i(t) = K_{i0} e^{-c|\theta|}'}, 'Location', 'best', 'FontSize', 10);

sgtitle('Adaptive PID Parameters Variation', 'FontSize', 16, 'FontWeight', 'bold');

saveas(figure2, 'figure4_2_pid_params.png');
saveas(figure2, 'figure4_2_pid_params.eps');
fprintf('  ✓ 图表2已保存\n');

%% ========== 图表3：控制力矩组成分析（4.1.4）==========

fprintf('\n生成图表3: 控制力矩组成分析\n');

figure3 = figure('Position', [100, 100, 800, 600]);

M_total = data.M_pid + data.d_hat;

color_pid = [0.30, 0.60, 0.90];
color_comp = [0.85, 0.40, 0.40];
color_total = [0.20, 0.20, 0.20];

fill([t; flipud(t)], [data.M_pid; zeros(size(data.M_pid))], ...
    color_pid, 'FaceAlpha', 0.6, 'EdgeColor', 'none');
hold on;

fill([t; flipud(t)], [M_total; flipud(data.M_pid)], ...
    color_comp, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

plot(t, M_total, '-', 'Color', color_total, 'LineWidth', 2.5);
plot(t, data.M_pid, '--', 'Color', [0.15, 0.45, 0.75], 'LineWidth', 2);

xline(t_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 2, ...
    'Label', 'Water Entry', 'FontSize', 11, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'bottom');

[~, idx_impact] = max(abs(M_total));
text(t(idx_impact), M_total(idx_impact) + 40, ...
    sprintf('Peak: %.1f Nm', M_total(idx_impact)), ...
    'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'white');

grid on;
set(gca, 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Control Torque (Nm)', 'FontSize', 14, 'FontWeight', 'bold');
title('Control Torque Composition: PID + Disturbance Compensation', 'FontSize', 16, 'FontWeight', 'bold');

legend({'M_{PID} (PID Control)', 'Disturbance Compensation', ...
        'M_{total} (Total Torque)', 'M_{PID} Boundary', 'Water Entry'}, ...
    'Location', 'best', 'FontSize', 11, 'Box', 'on');

% 添加嵌入子图：放大1.5-2.5s
h_inset = axes('Position', [0.55, 0.2, 0.32, 0.28]);
idx_zoom = find(t >= 1.5 & t <= 2.5);

fill([t(idx_zoom); flipud(t(idx_zoom))], ...
    [data.M_pid(idx_zoom); zeros(length(idx_zoom), 1)], ...
    color_pid, 'FaceAlpha', 0.6, 'EdgeColor', 'none');
hold on;

fill([t(idx_zoom); flipud(t(idx_zoom))], ...
    [M_total(idx_zoom); flipud(data.M_pid(idx_zoom))], ...
    color_comp, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

plot(t(idx_zoom), M_total(idx_zoom), '-', 'Color', color_total, 'LineWidth', 2);
plot(t(idx_zoom), data.M_pid(idx_zoom), '--', 'Color', [0.15, 0.45, 0.75], 'LineWidth', 1.5);

xline(t_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 1.5);

grid on;
xlabel('Time (s)', 'FontSize', 10);
ylabel('Torque (Nm)', 'FontSize', 10);
title('Zoomed: 1.5-2.5s', 'FontSize', 11, 'FontWeight', 'bold');
set(h_inset, 'FontSize', 9);
xlim([1.5, 2.5]);
box on;

saveas(figure3, 'figure4_3_control_torque.png');
saveas(figure3, 'figure4_3_control_torque.eps');
fprintf('  ✓ 图表3已保存\n');

%% ===== 保存数据 =====

fprintf('\n保存数据\n');

save_data = struct();
save_data.t = t;
save_data.theta = data.theta;
save_data.omega = data.omega;
save_data.M_pid = data.M_pid;
save_data.d_hat = data.d_hat;
save_data.theta_ref = theta_ref_val;
save_data.Mp = Mp;
save_data.Ts = Ts;
save_data.ess = ess;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(sprintf('section_4_1_data_%s.mat', timestamp), '-struct', 'save_data');

fprintf('  ✓ 数据已保存\n');

fprintf('\n=== 4.1节所有图表生成完成 ===\n');
fprintf('\n生成的图表：\n');
fprintf('  图表1: figure4_1_theta_response.png/eps (4.1.2 姿态角响应)\n');
fprintf('  图表2: figure4_2_pid_params.png/eps (4.1.3 PID参数变化-三子图)\n');
fprintf('  图表3: figure4_3_control_torque.png/eps (4.1.4 控制力矩组成)\n');
