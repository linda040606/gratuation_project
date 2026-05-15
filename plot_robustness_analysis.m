%% 鲁棒性分析：J/D ±20%摄动下的控制性能验证
% 验证控制策略在转动惯量与阻尼参数不确定情况下的鲁棒性

clear; clc; close all;

fprintf('=== 鲁棒性分析：J/D ±20%%摄动 ===\n\n');

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

fprintf('\n');

%% ===== 步骤2：添加数据导出模块 =====

fprintf('步骤2: 添加数据导出模块\n');

signals_to_export = {
    'theta',  'Water_Dynamics', 2, 'theta_sim';
    'omega',  'Water_Dynamics', 1, 'omega_sim';
    'J_t',    'Water_Dynamics', 3, 'J_t_sim';
    'D_t',    'Water_Dynamics', 4, 'D_t_sim';
    'ratio',  'Water_Dynamics', 5, 'ratio_sim';
};

for i = 1:size(signals_to_export, 1)
    signal_name = signals_to_export{i, 1};
    source_block = signals_to_export{i, 2};
    source_port = signals_to_export{i, 3};
    var_name = signals_to_export{i, 4};

    to_workspace_block = [model '/' signal_name '_out'];

    try
        get_param(to_workspace_block, 'BlockType');
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

fprintf('  ✓ 数据导出模块已添加\n\n');

%% ===== 步骤3：读取当前模型参数 =====

fprintf('步骤3: 读取当前模型参数\n');

theta_ref = evalin('base', 'theta_ref');
Kp_theta = evalin('base', 'Kp_theta');
Ki_theta = evalin('base', 'Ki_theta');
Kd_theta = evalin('base', 'Kd_theta');
Kp0_omega = evalin('base', 'Kp0_omega');
Ki0_omega = evalin('base', 'Ki0_omega');
Kd0_omega = evalin('base', 'Kd0_omega');
a_omega = evalin('base', 'a_omega');
b_omega = evalin('base', 'b_omega');
c_theta = evalin('base', 'c_theta');
t_fold_start = evalin('base', 't_fold_start');
t_water_enter = evalin('base', 't_water_enter');
tau_eso = evalin('base', 'tau_eso');

J_air_nominal = evalin('base', 'J_air');
J_water_nominal = evalin('base', 'J_water');
D_air_nominal = evalin('base', 'D_air');
D_water_nominal = evalin('base', 'D_water');
J_wing_reduction_nominal = evalin('base', 'J_wing_reduction');
wing_reduction_ratio = J_wing_reduction_nominal / J_air_nominal;

fprintf('  控制器参数:\n');
fprintf('    Kp_theta=%.1f, Ki_theta=%.2f, Kd_theta=%.1f\n', Kp_theta, Ki_theta, Kd_theta);
fprintf('    Kp0_omega=%.1f, Ki0_omega=%.2f, Kd0_omega=%.3f\n', Kp0_omega, Ki0_omega, Kd0_omega);
fprintf('    a_omega=%.1f, b_omega=%.1f, c_theta=%.1f\n', a_omega, b_omega, c_theta);
fprintf('  当前时序:\n');
fprintf('    t_fold_start=%.2fs, t_water_enter=%.2fs\n', t_fold_start, t_water_enter);
fprintf('  收翼惯量削减比例:\n');
fprintf('    J_wing_reduction/J_air = %.4f\n', wing_reduction_ratio);
fprintf('\n');

%% ===== 步骤4：定义参数摄动场景 =====

fprintf('步骤4: 定义参数摄动场景\n');

scenarios = struct();

scenarios(1).name = 'Nominal';
scenarios(1).J_air = J_air_nominal;
scenarios(1).J_water = J_water_nominal;
scenarios(1).D_air = D_air_nominal;
scenarios(1).D_water = D_water_nominal;
scenarios(1).J_wing_reduction = wing_reduction_ratio * scenarios(1).J_air;
scenarios(1).J_ratio = 0;
scenarios(1).D_ratio = 0;
scenarios(1).color = [0.00 0.45 0.74];
scenarios(1).style = '-';
scenarios(1).width = 2.2;

scenarios(2).name = 'J+20%, D+20%';
scenarios(2).J_air = J_air_nominal * 1.2;
scenarios(2).J_water = J_water_nominal * 1.2;
scenarios(2).D_air = D_air_nominal * 1.2;
scenarios(2).D_water = D_water_nominal * 1.2;
scenarios(2).J_wing_reduction = wing_reduction_ratio * scenarios(2).J_air;
scenarios(2).J_ratio = 20;
scenarios(2).D_ratio = 20;
scenarios(2).color = [0.85 0.33 0.10];
scenarios(2).style = '--';
scenarios(2).width = 1.8;

scenarios(3).name = 'J+20%, D-20%';
scenarios(3).J_air = J_air_nominal * 1.2;
scenarios(3).J_water = J_water_nominal * 1.2;
scenarios(3).D_air = D_air_nominal * 0.8;
scenarios(3).D_water = D_water_nominal * 0.8;
scenarios(3).J_wing_reduction = wing_reduction_ratio * scenarios(3).J_air;
scenarios(3).J_ratio = 20;
scenarios(3).D_ratio = -20;
scenarios(3).color = [0.20 0.70 0.20];
scenarios(3).style = '-.';
scenarios(3).width = 1.8;

scenarios(4).name = 'J-20%, D+20%';
scenarios(4).J_air = J_air_nominal * 0.8;
scenarios(4).J_water = J_water_nominal * 0.8;
scenarios(4).D_air = D_air_nominal * 1.2;
scenarios(4).D_water = D_water_nominal * 1.2;
scenarios(4).J_wing_reduction = wing_reduction_ratio * scenarios(4).J_air;
scenarios(4).J_ratio = -20;
scenarios(4).D_ratio = 20;
scenarios(4).color = [0.60 0.20 0.80];
scenarios(4).style = ':';
scenarios(4).width = 1.8;

scenarios(5).name = 'J-20%, D-20%';
scenarios(5).J_air = J_air_nominal * 0.8;
scenarios(5).J_water = J_water_nominal * 0.8;
scenarios(5).D_air = D_air_nominal * 0.8;
scenarios(5).D_water = D_water_nominal * 0.8;
scenarios(5).J_wing_reduction = wing_reduction_ratio * scenarios(5).J_air;
scenarios(5).J_ratio = -20;
scenarios(5).D_ratio = -20;
scenarios(5).color = [0.00 0.60 0.60];
scenarios(5).style = '-';
scenarios(5).width = 1.8;

fprintf('  5组参数场景:\n');
fprintf('    基准: J=[%.1f, %.1f], D=[%.2f, %.2f], J_wing_reduction=%.3f\n', ...
    J_air_nominal, J_water_nominal, D_air_nominal, D_water_nominal, J_wing_reduction_nominal);
for i = 2:length(scenarios)
    fprintf('    %d. %s: J=[%.1f, %.1f], D=[%.3f, %.2f], J_wing_reduction=%.3f\n', ...
        i, scenarios(i).name, scenarios(i).J_air, scenarios(i).J_water, ...
        scenarios(i).D_air, scenarios(i).D_water, scenarios(i).J_wing_reduction);
end
fprintf('\n');

%% ===== 步骤5：运行仿真 =====

fprintf('步骤5: 运行仿真\n');

for i = 1:length(scenarios)
    fprintf('  测试%d: %s\n', i, scenarios(i).name);

    assignin('base', 'J_air', scenarios(i).J_air);
    assignin('base', 'J_water', scenarios(i).J_water);
    assignin('base', 'D_air', scenarios(i).D_air);
    assignin('base', 'D_water', scenarios(i).D_water);
    assignin('base', 'J_wing_reduction', scenarios(i).J_wing_reduction);

    try
        simOut = sim(model, 'SaveOutput', 'on', 'SaveTime', 'on');

        if isprop(simOut, 'theta_sim') && isprop(simOut, 'J_t_sim') && isprop(simOut, 'D_t_sim')
            theta = simOut.theta_sim.signals.values;
            if size(theta, 1) == 1
                theta = theta';
            end
            t = simOut.theta_sim.time;

            J_t = simOut.J_t_sim.signals.values;
            if size(J_t, 1) == 1
                J_t = J_t';
            end

            D_t = simOut.D_t_sim.signals.values;
            if size(D_t, 1) == 1
                D_t = D_t';
            end

            scenarios(i).theta = theta;
            scenarios(i).t = t;
            scenarios(i).J_t = J_t;
            scenarios(i).D_t = D_t;
            scenarios(i).valid = true;

            fprintf('    ✓ 仿真完成\n');
        else
            scenarios(i).valid = false;
            fprintf('    ✗ 数据提取失败\n');
        end
    catch ME
        scenarios(i).valid = false;
        fprintf('    ✗ 仿真失败: %s\n', ME.message);
    end
end

fprintf('\n');

%% ===== 步骤6：计算性能指标 =====

fprintf('步骤6: 计算性能指标\n');

theta_ref_val = theta_ref;
band = 0.02 * abs(theta_ref_val);

fprintf('\n');
fprintf('========================================================================================\n');
fprintf('%-18s | %-8s | %-8s | %-12s | %-12s | %-12s\n', ...
    '场景', 'J变化', 'D变化', 'Ts_track(s)', 'Ts_rec(s)', 'ess(rad)');
fprintf('========================================================================================\n');

for i = 1:length(scenarios)
    if scenarios(i).valid
        theta = scenarios(i).theta;
        t = scenarios(i).t;
        J_t = scenarios(i).J_t;
        D_t = scenarios(i).D_t;

        idx_track_end = find(t <= t_fold_start, 1, 'last');
        if isempty(idx_track_end)
            idx_track_end = 1;
        end

        Ts_track = NaN;
        for j = 1:idx_track_end
            if all(abs(theta(j:idx_track_end) - theta_ref_val) <= band)
                Ts_track = t(j);
                break;
            end
        end
        if isnan(Ts_track)
            Ts_track = t(idx_track_end);
        end
        scenarios(i).Ts_track = Ts_track;

        idx_water_enter = find(t >= t_water_enter, 1);
        if isempty(idx_water_enter)
            idx_water_enter = length(t);
        end

        Ts_recovery = NaN;
        for j = idx_water_enter:length(t)
            if all(abs(theta(j:end) - theta_ref_val) <= band)
                Ts_recovery = t(j) - t_water_enter;
                break;
            end
        end
        scenarios(i).Ts_recovery = Ts_recovery;

        idx_ss = find(t >= 16 & t <= min(20, t(end)));
        if ~isempty(idx_ss)
            ess = mean(abs(theta(idx_ss) - theta_ref_val));
        else
            ess = abs(theta(end) - theta_ref_val);
        end
        scenarios(i).ess = ess;

        idx_fold_window = find(t >= max(t_fold_start, t_water_enter - 0.5) & t <= t_water_enter - 0.05);
        if isempty(idx_fold_window)
            idx_fold_window = max(1, idx_water_enter-1):max(1, idx_water_enter-1);
        end
        scenarios(i).J_fold_end = mean(J_t(idx_fold_window));
        scenarios(i).D_fold_end = mean(D_t(idx_fold_window));

        idx_post_water = find(t >= t_water_enter + 0.1 & t <= min(t_water_enter + 1.0, t(end)));
        if isempty(idx_post_water)
            idx_post_water = idx_water_enter:length(t);
        end
        scenarios(i).J_post_water = mean(J_t(idx_post_water));
        scenarios(i).D_post_water = mean(D_t(idx_post_water));

        fprintf('%-18s | %+7d%%  | %+7d%%  | %12.4f | %12.4f | %12.6f\n', ...
            scenarios(i).name, scenarios(i).J_ratio, scenarios(i).D_ratio, ...
            scenarios(i).Ts_track, scenarios(i).Ts_recovery, scenarios(i).ess);
    else
        scenarios(i).Ts_track = NaN;
        scenarios(i).Ts_recovery = NaN;
        scenarios(i).ess = NaN;
        scenarios(i).J_fold_end = NaN;
        scenarios(i).D_fold_end = NaN;
        scenarios(i).J_post_water = NaN;
        scenarios(i).D_post_water = NaN;
        fprintf('%-18s | %+7d%%  | %+7d%%  | %12s | %12s | %12s\n', ...
            scenarios(i).name, scenarios(i).J_ratio, scenarios(i).D_ratio, 'N/A', 'N/A', 'N/A');
    end
end

fprintf('========================================================================================\n');

valid_idx = find([scenarios.valid]);
if ~isempty(valid_idx)
    Ts_track_values = [scenarios(valid_idx).Ts_track];
    ess_values = [scenarios(valid_idx).ess];
    J_fold_values = [scenarios(valid_idx).J_fold_end];
    D_fold_values = [scenarios(valid_idx).D_fold_end];
    J_post_values = [scenarios(valid_idx).J_post_water];
    D_post_values = [scenarios(valid_idx).D_post_water];

    fprintf('\n  参数轨迹检查:\n');
    fprintf('    收翼末端 J_t 范围 [%.4f, %.4f]\n', min(J_fold_values), max(J_fold_values));
    fprintf('    入水前 D_t 范围 [%.4f, %.4f]\n', min(D_fold_values), max(D_fold_values));
    fprintf('    入水后 J_t 范围 [%.4f, %.4f]\n', min(J_post_values), max(J_post_values));
    fprintf('    入水后 D_t 范围 [%.4f, %.4f]\n', min(D_post_values), max(D_post_values));
    fprintf('\n  统计结果:\n');
    fprintf('    跟踪调节时间: 范围 [%.4f, %.4f] s\n', min(Ts_track_values), max(Ts_track_values));
    fprintf('    稳态误差: 范围 [%.6f, %.6f] rad\n', min(ess_values), max(ess_values));
end

fprintf('\n');

%% ===== 步骤7：绘制对比图 =====

fprintf('步骤7: 绘制对比图\n');

hFig = figure('Position', [80, 60, 1300, 900]);
set(hFig, 'DefaultAxesFontName', 'SimSun');
set(hFig, 'DefaultTextFontName', 'SimSun');

subplot(2, 2, 1);
hold on;
for i = 1:length(scenarios)
    if scenarios(i).valid
        plot(scenarios(i).t, scenarios(i).theta, ...
            'Color', scenarios(i).color, ...
            'LineStyle', scenarios(i).style, ...
            'LineWidth', scenarios(i).width);
    end
end
yline(theta_ref_val, '--k', 'LineWidth', 1.5, 'Label', '参考值', 'FontName', 'SimSun');
xline(t_fold_start, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.5, ...
    'Label', '收翼开始', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimSun');
xline(t_water_enter, '--', 'Color', [0.8 0.2 0.2], 'LineWidth', 1.8, ...
    'Label', '入水时刻', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimSun');
grid on;
xlabel('时间 (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
ylabel('姿态角 \theta (rad)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
title('姿态角响应对比', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimSun');
legend({scenarios.name}, 'Location', 'best', 'FontSize', 10, 'FontName', 'SimSun');
xlim([0, 20]);
set(gca, 'FontName', 'SimSun');

subplot(2, 2, 2);
hold on;
t_window = [11, 12];
for i = 1:length(scenarios)
    if scenarios(i).valid
        idx = scenarios(i).t >= t_window(1) & scenarios(i).t <= t_window(2);
        plot(scenarios(i).t(idx), scenarios(i).theta(idx), ...
            'Color', scenarios(i).color, ...
            'LineStyle', scenarios(i).style, ...
            'LineWidth', scenarios(i).width + 0.5);
    end
end
yline(theta_ref_val, '--k', 'LineWidth', 1.5);
xline(t_water_enter, '-', 'Color', [0.8 0.2 0.2], 'LineWidth', 2, ...
    'Label', 't=11.0s', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'SimSun');
grid on;
xlabel('时间 (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
ylabel('姿态角 \theta (rad)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
title('局部放大：11s~12s入水响应', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimSun');
xlim(t_window);
set(gca, 'FontName', 'SimSun');

subplot(2, 2, 3);
hold on;
for i = 1:length(scenarios)
    if scenarios(i).valid
        plot(scenarios(i).t, scenarios(i).J_t, ...
            'Color', scenarios(i).color, ...
            'LineStyle', scenarios(i).style, ...
            'LineWidth', scenarios(i).width);
    end
end
xline(t_fold_start, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.5, ...
    'Label', '收翼开始', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimSun');
xline(t_water_enter, '--', 'Color', [0.8 0.2 0.2], 'LineWidth', 1.8, ...
    'Label', '入水时刻', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimSun');
grid on;
xlabel('时间 (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
ylabel('J_t', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
title('时变转动惯量 J_t 对比', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimSun');
legend({scenarios.name}, 'Location', 'best', 'FontSize', 10, 'FontName', 'SimSun');
xlim([0, 20]);
set(gca, 'FontName', 'SimSun');

subplot(2, 2, 4);
hold on;
for i = 1:length(scenarios)
    if scenarios(i).valid
        plot(scenarios(i).t, scenarios(i).D_t, ...
            'Color', scenarios(i).color, ...
            'LineStyle', scenarios(i).style, ...
            'LineWidth', scenarios(i).width);
    end
end
xline(t_fold_start, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.5, ...
    'Label', '收翼开始', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimSun');
xline(t_water_enter, '--', 'Color', [0.8 0.2 0.2], 'LineWidth', 1.8, ...
    'Label', '入水时刻', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'SimSun');
grid on;
xlabel('时间 (s)', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
ylabel('D_t', 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'SimSun');
title('时变阻尼 D_t 对比', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'SimSun');
legend({scenarios.name}, 'Location', 'best', 'FontSize', 10, 'FontName', 'SimSun');
xlim([0, 20]);
set(gca, 'FontName', 'SimSun');

sgtitle('鲁棒性分析：J 和 D ±20%同步摄动', ...
    'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'SimSun');

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename_png = sprintf('robustness_JD_%s.png', timestamp);
filename_eps = sprintf('robustness_JD_%s.eps', timestamp);
saveas(hFig, filename_png);
saveas(hFig, filename_eps);
fprintf('  ✓ 图片已保存: %s\n', filename_png);
fprintf('  ✓ 图片已保存: %s\n', filename_eps);

%% ===== 保存数据 =====

fprintf('\n步骤8: 保存数据\n');

filename_mat = sprintf('robustness_JD_data_%s.mat', timestamp);
save(filename_mat, 'scenarios', 'theta_ref_val', 'band', ...
    'J_air_nominal', 'J_water_nominal', 'D_air_nominal', 'D_water_nominal', ...
    't_fold_start', 't_water_enter', 'wing_reduction_ratio');
fprintf('  ✓ 数据已保存: %s\n', filename_mat);

fprintf('\n=== 鲁棒性分析完成 ===\n');
