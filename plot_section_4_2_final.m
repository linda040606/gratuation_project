%% 4.2节控制性能对比仿真分析 - 学术级图表生成脚本（最终优化版）
% 所有图表使用科研配色，英文标签

clear; clc; close all;

fprintf('=== 4.2节控制性能对比分析图表生成（最终版）===\n\n');

projectDir = 'C:\Users\25516\Desktop\graduationproject\simulink\test2';
cd(projectDir);

model = 'missile_closed_loop';

%% ===== 步骤1：创建模型 =====

fprintf('步骤1: 创建模型\n');

try
    create_correct_closed_loop_v3;
    fprintf('  ✓ 模型创建成功\n');
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

%% ===== 步骤3：运行多方案仿真 =====

fprintf('\n步骤3: 运行多方案仿真\n');

results = struct();
theta_ref_val = evalin('base', 'theta_ref');
t_fold_start_val = evalin('base', 't_fold_start');
t_water_enter_val = evalin('base', 't_water_enter');

% 学术配色方案
color_scheme1 = [0.85, 0.33, 0.10];  % IEEE Red - 固定 PID
color_scheme2 = [0.00, 0.45, 0.74];  % IEEE Blue - 固定 PID + 扰动观测器
color_scheme3 = [0.20, 0.70, 0.20];  % IEEE Green - 变参数 PID + 扰动观测器

%% ===== 测试1: 固定PID =====

fprintf('\n  测试1: 固定 PID...\n');
assignin('base', 'a_omega', 0);
assignin('base', 'b_omega', 0);
assignin('base', 'c_theta', 0);
assignin('base', 'tau_eso', 1000);  % 禁用观测器

try
    simOut_fixed = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');

    if isprop(simOut_fixed, 'theta_sim')
        theta_fixed = simOut_fixed.theta_sim.signals.values;
        if size(theta_fixed, 1) == 1
            theta_fixed = theta_fixed';
        end
        t_fixed = simOut_fixed.theta_sim.time;
        results.theta_fixed = theta_fixed;
        results.t_fixed = t_fixed;
    end

    if isprop(simOut_fixed, 'M_pid_sim')
        results.Mpid_fixed = simOut_fixed.M_pid_sim.signals.values;
        if size(results.Mpid_fixed, 1) == 1
            results.Mpid_fixed = results.Mpid_fixed';
        end
    end

    fprintf('    ✓ 仿真完成\n');
catch ME
    fprintf('    ✗ 仿真失败: %s\n', ME.message);
end

%% ===== 测试2: 固定PID + 扰动观测器 =====

fprintf('  测试2: 固定 PID + 扰动观测器...\n');
assignin('base', 'a_omega', 0);
assignin('base', 'b_omega', 0);
assignin('base', 'c_theta', 0);
assignin('base', 'tau_eso', 0.01);  % 启用观测器

try
    simOut_ab = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');

    if isprop(simOut_ab, 'theta_sim')
        theta_ab = simOut_ab.theta_sim.signals.values;
        if size(theta_ab, 1) == 1
            theta_ab = theta_ab';
        end
        t_ab = simOut_ab.theta_sim.time;
        results.theta_ab = theta_ab;
        results.t_ab = t_ab;
    end

    if isprop(simOut_ab, 'M_pid_sim')
        results.Mpid_ab = simOut_ab.M_pid_sim.signals.values;
        if size(results.Mpid_ab, 1) == 1
            results.Mpid_ab = results.Mpid_ab';
        end
    end

    fprintf('    ✓ 仿真完成\n');
catch ME
    fprintf('    ✗ 仿真失败: %s\n', ME.message);
end

%% ===== 测试3: 变参数PID + 扰动观测器 =====

fprintf('  测试3: 变参数 PID + 扰动观测器（本文方法）...\n');
assignin('base', 'a_omega', 550);
assignin('base', 'b_omega', 4);
assignin('base', 'c_theta', 4);
assignin('base', 'tau_eso', 0.01);

try
    simOut_abc = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');

    if isprop(simOut_abc, 'theta_sim')
        theta_abc = simOut_abc.theta_sim.signals.values;
        if size(theta_abc, 1) == 1
            theta_abc = theta_abc';
        end
        t_abc = simOut_abc.theta_sim.time;
        results.theta_abc = theta_abc;
        results.t_abc = t_abc;
    end

    if isprop(simOut_abc, 'M_pid_sim')
        results.Mpid_abc = simOut_abc.M_pid_sim.signals.values;
        if size(results.Mpid_abc, 1) == 1
            results.Mpid_abc = results.Mpid_abc';
        end
    end

    fprintf('    ✓ 仿真完成\n');
catch ME
    fprintf('    ✗ 仿真失败: %s\n', ME.message);
end

%% ===== 步骤4：计算性能指标 =====

fprintf('\n步骤4: 计算性能指标\n');

% 关键时刻
idx_track_end_fixed = find(results.t_fixed <= t_fold_start_val, 1, 'last');
idx_track_end_ab = find(results.t_ab <= t_fold_start_val, 1, 'last');
idx_track_end_abc = find(results.t_abc <= t_fold_start_val, 1, 'last');

[~, idx_enter_fixed] = min(abs(results.t_fixed - t_water_enter_val));
[~, idx_enter_ab] = min(abs(results.t_ab - t_water_enter_val));
[~, idx_enter_abc] = min(abs(results.t_abc - t_water_enter_val));

% 方案1指标
metrics_fixed = struct();
metrics_fixed.Mp = (max(results.theta_fixed) - theta_ref_val) / abs(theta_ref_val) * 100;
metrics_fixed.ess = abs(results.theta_fixed(end) - theta_ref_val);
band = 0.02 * abs(theta_ref_val);

metrics_fixed.Ts_track = NaN;
for i = 1:idx_track_end_fixed
    if all(abs(results.theta_fixed(i:idx_track_end_fixed) - theta_ref_val) <= band)
        metrics_fixed.Ts_track = results.t_fixed(i);
        break;
    end
end

metrics_fixed.Ts_recovery = NaN;
for i = idx_enter_fixed:length(results.t_fixed)
    if all(abs(results.theta_fixed(i:end) - theta_ref_val) <= band)
        metrics_fixed.Ts_recovery = results.t_fixed(i) - t_water_enter_val;
        break;
    end
end

metrics_fixed.Ts = NaN;
for i = length(results.t_fixed):-1:1
    if abs(results.theta_fixed(i) - theta_ref_val) > band
        if i < length(results.t_fixed)
            metrics_fixed.Ts = results.t_fixed(i+1);
        else
            metrics_fixed.Ts = results.t_fixed(end);
        end
        break;
    end
end
if isnan(metrics_fixed.Ts)
    metrics_fixed.Ts = results.t_fixed(1);
end

% 方案2指标
metrics_ab = struct();
metrics_ab.Mp = (max(results.theta_ab) - theta_ref_val) / abs(theta_ref_val) * 100;
metrics_ab.ess = abs(results.theta_ab(end) - theta_ref_val);

metrics_ab.Ts_track = NaN;
for i = 1:idx_track_end_ab
    if all(abs(results.theta_ab(i:idx_track_end_ab) - theta_ref_val) <= band)
        metrics_ab.Ts_track = results.t_ab(i);
        break;
    end
end

metrics_ab.Ts_recovery = NaN;
for i = idx_enter_ab:length(results.t_ab)
    if all(abs(results.theta_ab(i:end) - theta_ref_val) <= band)
        metrics_ab.Ts_recovery = results.t_ab(i) - t_water_enter_val;
        break;
    end
end

metrics_ab.Ts = NaN;
for i = length(results.t_ab):-1:1
    if abs(results.theta_ab(i) - theta_ref_val) > band
        if i < length(results.t_ab)
            metrics_ab.Ts = results.t_ab(i+1);
        else
            metrics_ab.Ts = results.t_ab(end);
        end
        break;
    end
end
if isnan(metrics_ab.Ts)
    metrics_ab.Ts = results.t_ab(1);
end

% 方案3指标
metrics_abc = struct();
metrics_abc.Mp = (max(results.theta_abc) - theta_ref_val) / abs(theta_ref_val) * 100;
metrics_abc.ess = abs(results.theta_abc(end) - theta_ref_val);

metrics_abc.Ts_track = NaN;
for i = 1:idx_track_end_abc
    if all(abs(results.theta_abc(i:idx_track_end_abc) - theta_ref_val) <= band)
        metrics_abc.Ts_track = results.t_abc(i);
        break;
    end
end

metrics_abc.Ts_recovery = NaN;
for i = idx_enter_abc:length(results.t_abc)
    if all(abs(results.theta_abc(i:end) - theta_ref_val) <= band)
        metrics_abc.Ts_recovery = results.t_abc(i) - t_water_enter_val;
        break;
    end
end

metrics_abc.Ts = NaN;
for i = length(results.t_abc):-1:1
    if abs(results.theta_abc(i) - theta_ref_val) > band
        if i < length(results.t_abc)
            metrics_abc.Ts = results.t_abc(i+1);
        else
            metrics_abc.Ts = results.t_abc(end);
        end
        break;
    end
end
if isnan(metrics_abc.Ts)
    metrics_abc.Ts = results.t_abc(1);
end

% 计算能耗
energy_fixed = trapz(results.t_fixed, abs(results.Mpid_fixed));
energy_ab = trapz(results.t_ab, abs(results.Mpid_ab));
energy_abc = trapz(results.t_abc, abs(results.Mpid_abc));

fprintf('  方案1: Mp=%.2f%%, Ts_跟踪=%.3fs, Ts_恢复=%.3fs, Ts_全程=%.3fs, ess=%.5f\n', ...
    metrics_fixed.Mp, metrics_fixed.Ts_track, metrics_fixed.Ts_recovery, metrics_fixed.Ts, metrics_fixed.ess);
fprintf('  方案2: Mp=%.2f%%, Ts_跟踪=%.3fs, Ts_恢复=%.3fs, Ts_全程=%.3fs, ess=%.5f\n', ...
    metrics_ab.Mp, metrics_ab.Ts_track, metrics_ab.Ts_recovery, metrics_ab.Ts, metrics_ab.ess);
fprintf('  方案3: Mp=%.2f%%, Ts_跟踪=%.3fs, Ts_恢复=%.3fs, Ts_全程=%.3fs, ess=%.5f\n', ...
    metrics_abc.Mp, metrics_abc.Ts_track, metrics_abc.Ts_recovery, metrics_abc.Ts, metrics_abc.ess);

%% ========== 图表5：多方案姿态对比 + 局部放大（4.2.2）==========

fprintf('\n生成图表5: 多方案姿态对比\n');

figure5 = figure('Position', [100, 100, 900, 700]);
set(figure5, 'DefaultAxesFontName', 'SimSun');
set(figure5, 'DefaultTextFontName', 'SimSun');

% 主图
plot(results.t_fixed, results.theta_fixed, '-', 'Color', color_scheme1, 'LineWidth', 1.8);
hold on;
plot(results.t_ab, results.theta_ab, '--', 'Color', color_scheme2, 'LineWidth', 1.8);
plot(results.t_abc, results.theta_abc, '-', 'Color', color_scheme3, 'LineWidth', 2.5);
yline(theta_ref_val, '--', 'Color', [0.50, 0.50, 0.50], 'LineWidth', 1.5);

xline(t_water_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 2, ...
    'Label', '入水时刻', 'FontSize', 11, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'top', 'FontName', 'SimSun');

grid on;
set(gca, 'FontSize', 12, 'FontName', 'SimSun');
xlabel('时间 (s)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimSun');
ylabel('姿态角 θ (rad)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimSun');
title('多方案姿态响应对比', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimSun');
legend({'固定 PID', '固定 PID + 扰动观测器', '变参数 PID + 扰动观测器（本文方法）', '参考值'}, ...
    'Location', 'best', 'FontSize', 11, 'Box', 'on', 'FontName', 'SimSun');

% 嵌入子图：放大11s入水瞬间
h_inset = axes('Position', [0.65, 0.6, 0.20, 0.28]);
set(h_inset, 'FontName', 'SimSun');
idx_fixed = find(results.t_fixed >= 10.5 & results.t_fixed <= 12);
idx_ab = find(results.t_ab >= 10.5 & results.t_ab <= 12);
idx_abc = find(results.t_abc >= 10.5 & results.t_abc <= 12);

plot(results.t_fixed(idx_fixed), results.theta_fixed(idx_fixed), '-', 'Color', color_scheme1, 'LineWidth', 2.5);
hold on;
plot(results.t_ab(idx_ab), results.theta_ab(idx_ab), '--', 'Color', color_scheme2, 'LineWidth', 2.5);
plot(results.t_abc(idx_abc), results.theta_abc(idx_abc), '-', 'Color', color_scheme3, 'LineWidth', 3);
yline(theta_ref_val, '--', 'Color', [0.50, 0.50, 0.50], 'LineWidth', 1.5);
xline(t_water_enter_val, '-', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 2, ...
    'Label', '入水时刻', 'FontSize', 10, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'top', 'FontName', 'SimSun');
xlim([11, 12]);
grid on;
xlabel('时间 (s)', 'FontSize', 10, 'FontName', 'SimSun');
ylabel('姿态角 θ (rad)', 'FontSize', 10, 'FontName', 'SimSun');
title('局部放大：11s入水瞬间', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimSun');
set(h_inset, 'FontSize', 9, 'FontName', 'SimSun');
box on;

saveas(figure5, 'figure4_5_multi_scheme_comparison.png');
saveas(figure5, 'figure4_5_multi_scheme_comparison.eps');
fprintf('  ✓ 图表5已保存\n');
% 
% %% ========== 图表6：抗扰动能力对比（4.2.3）==========
%
% fprintf('\n生成图表6: 抗扰动能力对比\n');
%
% A_impact_list = [0, 150, 300, 450];
% theta_disturb = zeros(1001, 3, 4);
% t_disturb = zeros(1001, 4);
%
% for i = 1:4
%     fprintf('  测试 A_impact=%d...\n', A_impact_list(i));
%
%     assignin('base', 'A_impact', A_impact_list(i));
%
%     % 方案1: 固定PID
%     assignin('base', 'a_omega', 0);
%     assignin('base', 'b_omega', 0);
%     assignin('base', 'c_theta', 0);
%     assignin('base', 'tau_eso', 1000);
%
%     try
%         simOut_d = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');
%         if isprop(simOut_d, 'theta_sim')
%             theta_disturb(:,1,i) = simOut_d.theta_sim.signals.values;
%             if size(theta_disturb(:,1,i), 1) == 1
%                 theta_disturb(:,1,i) = theta_disturb(:,1,i)';
%             end
%             t_disturb(:,i) = simOut_d.theta_sim.time;
%         end
%     catch
%     end
%
%     % 方案2: 固定PID + 观测器
%     assignin('base', 'tau_eso', 0.01);
%
%     try
%         simOut_d = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');
%         if isprop(simOut_d, 'theta_sim')
%             theta_disturb(:,2,i) = simOut_d.theta_sim.signals.values;
%             if size(theta_disturb(:,2,i), 1) == 1
%                 theta_disturb(:,2,i) = theta_disturb(:,2,i)';
%             end
%         end
%     catch
%     end
%
%     % 方案3: 变参数PID + 观测器
%     assignin('base', 'a_omega', 8);
%     assignin('base', 'b_omega', 1.5);
%     assignin('base', 'c_theta', 2);
%
%     try
%         simOut_d = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');
%         if isprop(simOut_d, 'theta_sim')
%             theta_disturb(:,3,i) = simOut_d.theta_sim.signals.values;
%             if size(theta_disturb(:,3,i), 1) == 1
%                 theta_disturb(:,3,i) = theta_disturb(:,3,i)';
%             end
%         end
%     catch
%     end
% end
%
% % 绘制分面网格图
% figure6 = figure('Position', [100, 100, 1200, 900]);
% set(figure6, 'DefaultAxesFontName', 'SimSun');
% set(figure6, 'DefaultTextFontName', 'SimSun');
%
% labels = {'固定 PID', '固定 PID + 扰动观测器', '变参数 PID + 扰动观测器'};
%
% for i = 1:4
%     subplot(2, 2, i);
%
%     plot(t_disturb(:,i), theta_disturb(:,1,i), '-', 'Color', color_scheme1, 'LineWidth', 2);
%     hold on;
%     plot(t_disturb(:,i), theta_disturb(:,2,i), '--', 'Color', color_scheme2, 'LineWidth', 2);
%     plot(t_disturb(:,i), theta_disturb(:,3,i), '-', 'Color', color_scheme3, 'LineWidth', 2.5);
%
%     yline(theta_ref_val, '--', 'Color', [0.50, 0.50, 0.50], 'LineWidth', 1.5);
%     xline(t_enter_val, '--', 'Color', [0.80, 0.20, 0.20], 'LineWidth', 1.5, ...
%         'Label', '入水时刻', 'FontName', 'SimSun');
%
%     grid on;
%     set(gca, 'FontSize', 11, 'FontName', 'SimSun');
%     xlabel('时间 (s)', 'FontSize', 12, 'FontName', 'SimSun');
%     ylabel('姿态角 θ (rad)', 'FontSize', 12, 'FontName', 'SimSun');
%     title(sprintf('扰动强度 A_{impact}=%d', A_impact_list(i)), 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimSun');
%     legend(labels, 'Location', 'best', 'FontSize', 10, 'FontName', 'SimSun');
% end
%
% sgtitle('不同扰动强度下的性能对比', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimSun');
%
% saveas(figure6, 'figure4_6_disturbance_rejection.png');
% saveas(figure6, 'figure4_6_disturbance_rejection.eps');
% fprintf('  ✓ 图表6已保存\n');
%
% %% ========== 图表7：综合性能雷达图（4.2.4）==========
%
% fprintf('\n生成图表7: 综合性能雷达图\n');
%
% figure7 = figure('Position', [100, 100, 800, 800]);
% set(figure7, 'DefaultAxesFontName', 'SimSun');
% set(figure7, 'DefaultTextFontName', 'SimSun');
%
% % 计算5个维度的归一化指标
% metrics_matrix = [
%     1/metrics_fixed.Mp,  1/metrics_fixed.Ts,  1/metrics_fixed.ess,  1/energy_fixed,  0.8;
%     1/metrics_ab.Mp,     1/metrics_ab.Ts,     1/metrics_ab.ess,     1/energy_ab,     0.9;
%     1/metrics_abc.Mp,    1/metrics_abc.Ts,    1/metrics_abc.ess,    1/energy_abc,    1.0;
% ];
%
% % 归一化到 [0, 1]
% metrics_norm = zeros(size(metrics_matrix));
% for i = 1:size(metrics_matrix, 2)
%     max_val = max(metrics_matrix(:, i));
%     metrics_norm(:, i) = metrics_matrix(:, i) / max_val;
% end
%
% % 雷达图标签
% labels = {'跟踪精度', '响应速度', '抗冲击能力', '能量效率', '鲁棒性'};
% num_vars = length(labels);
% angles = linspace(0, 2*pi, num_vars+1);
%
% % 创建极坐标轴
% ax = polaraxes('Parent', figure7);
% hold(ax, 'on');
% ax.FontName = 'SimSun';
% ax.FontSize = 11;
%
% % 绘制雷达图
% colors = {color_scheme1, color_scheme2, color_scheme3};
% line_styles = {'--', '--', '-'};
% line_widths = [1.5, 1.5, 2.5];
%
% for i = 1:3
%     values = [metrics_norm(i, :), metrics_norm(i, 1)];
%     polarplot(ax, angles, values, 'Color', colors{i}, ...
%         'LineStyle', line_styles{i}, ...
%         'LineWidth', line_widths(i), ...
%         'Marker', 'o', ...
%         'MarkerSize', 6);
% end
%
% % 设置雷达图样式
% ax.ThetaTick = rad2deg(angles(1:end-1));
% ax.ThetaTickLabel = labels;
% ax.RLim = [0, 1.1];
% ax.RTick = [0.2, 0.4, 0.6, 0.8, 1.0];
%
% title(ax, '综合性能雷达图', 'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimSun');
% legend(ax, {'固定 PID', '固定 PID + 扰动观测器', '变参数 PID + 扰动观测器（本文方法）'}, ...
%     'Location', 'best', 'FontSize', 11, 'FontName', 'SimSun');
%
% grid(ax, 'on');
%
% saveas(figure7, 'figure4_7_performance_radar.png');
% saveas(figure7, 'figure4_7_performance_radar.eps');
% fprintf('  ✓ 图表7已保存\n');
%
% %% ========== 图表8：定量指标对比柱状图（4.2.4）==========
%
% fprintf('\n生成图表8: 定量性能指标对比\n');
%
% figure8 = figure('Position', [100, 100, 900, 600]);
% set(figure8, 'DefaultAxesFontName', 'SimSun');
% set(figure8, 'DefaultTextFontName', 'SimSun');
%
% % 性能指标数据
% metric_names = {'调节时间 Ts (s)', '超调量 Mp (%)', '稳态误差 ess (10^{-3} rad)'};
% metrics_data = [
%     metrics_fixed.Ts,  metrics_fixed.Mp,  metrics_fixed.ess*1000;
%     metrics_ab.Ts,     metrics_ab.Mp,     metrics_ab.ess*1000;
%     metrics_abc.Ts,    metrics_abc.Mp,    metrics_abc.ess*1000;
% ];
%
% % 归一化（方案1设为1）
% metrics_norm = metrics_data ./ metrics_data(1, :);
%
% % 绘制分组柱状图
% x = 1:3;
%
% bar_data = zeros(3, 3);
% for i = 1:3
%     for j = 1:3
%         bar_data(i, j) = metrics_norm(j, i);
%     end
% end
%
% b = bar(x, bar_data');
% b(1).FaceColor = color_scheme1;
% b(2).FaceColor = color_scheme2;
% b(3).FaceColor = color_scheme3;
%
% % 添加改善百分比标注
% hold on;
% for i = 1:3
%     improvement = (1 - metrics_norm(3, i)) * 100;
%     text(i, bar_data(3, i) + 0.05, sprintf('↓%.1f%%', improvement), ...
%         'HorizontalAlignment', 'center', ...
%         'FontSize', 11, 'FontWeight', 'bold', ...
%         'Color', color_scheme3, 'FontName', 'SimSun');
% end
%
% grid on;
% set(gca, 'XTickLabel', metric_names, 'FontSize', 12, 'FontName', 'SimSun');
% ylabel('归一化数值（方案1=1）', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'SimSun');
% title('定量性能指标对比', 'FontSize', 15, 'FontWeight', 'bold', 'FontName', 'SimSun');
% legend({'固定 PID', '固定 PID + 扰动观测器', '变参数 PID + 扰动观测器（本文方法）'}, ...
%     'Location', 'best', 'FontSize', 11, 'Box', 'on', 'FontName', 'SimSun');
% ylim([0, 1.5]);
%
% saveas(figure8, 'figure4_8_performance_metrics.png');
% saveas(figure8, 'figure4_8_performance_metrics.eps');
% fprintf('  ✓ 图表8已保存\n');
%
% %% ===== 保存数据 =====
%
% fprintf('\n保存数据\n');
%
% save('section_4_2_data.mat', ...
%     'results', 'metrics_fixed', 'metrics_ab', 'metrics_abc', ...
%     'theta_disturb', 't_disturb', 'A_impact_list', ...
%     'energy_fixed', 'energy_ab', 'energy_abc');
%
% fprintf('  ✓ 数据已保存\n');
%
% fprintf('\n=== 4.2节所有图表生成完成 ===\n');
% fprintf('\n生成的图表：\n');
% fprintf('  图表5: figure4_5_multi_scheme_comparison.png/eps (4.2.2 多方案对比)\n');
% fprintf('  图表6: figure4_6_disturbance_rejection.png/eps (4.2.3 抗扰动能力)\n');
% fprintf('  图表7: figure4_7_performance_radar.png/eps (4.2.4 综合性能雷达图)\n');
% fprintf('  图表8: figure4_8_performance_metrics.png/eps (4.2.4 定量指标对比)\n');
