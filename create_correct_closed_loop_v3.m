% 创建重构版导弹闭环控制系统（Subsystem封装版）
% D项修复版：修正D项实现为真正的微分项
%
% 修复内容：
% - 添加Derivative模块计算d(e_omega)/dt
% - D项从 Kd*omega 改为 Kd*d(e_omega)/dt

clear; clc; close all;

fprintf('=== 创建重构版导弹闭环控制系统（D项修复版）===\n');

projectDir = 'C:\Users\25516\Desktop\graduationproject\simulink\test2';
model = 'missile_closed_loop';
modelPath = fullfile(projectDir, [model '.slx']);

% 清理旧模型
if bdIsLoaded(model)
    close_system(model, 0);
end
if exist(modelPath, 'file')
    delete(modelPath);
end

% 新建模型
new_system(model);
open_system(model);

% 仿真设置
set_param(model, 'Solver', 'ode45');
set_param(model, 'StopTime', '10');
set_param(model, 'MaxStep', '1e-3');
set_param(model, 'SaveTime', 'on', 'TimeSaveName', 'tout');
set_param(model, 'SaveOutput', 'on', 'OutputSaveName', 'yout');

%% ===== 顶层仅保留 6 个模块 =====
% 1) Signal_Interface
add_block('simulink/Ports & Subsystems/Subsystem', [model '/Signal_Interface'], ...
    'Position', [60, 70, 260, 350]);

% 2) Variable_PID
add_block('simulink/Ports & Subsystems/Subsystem', [model '/Variable_PID'], ...
    'Position', [330, 120, 550, 280]);

% 3) Disturbance_Observer
add_block('simulink/Ports & Subsystems/Subsystem', [model '/Disturbance_Observer'], ...
    'Position', [620, 320, 840, 470]);

% 4) M_Comp (M = M_pid + d_hat)
add_block('simulink/Math Operations/Add', [model '/M_Comp'], ...
    'Position', [620, 170, 650, 200]);

% 5) Water_Dynamics
add_block('simulink/Ports & Subsystems/Subsystem', [model '/Water_Dynamics'], ...
    'Position', [740, 90, 1020, 300]);

% 6) Model_Out（单一向量输出）
add_block('simulink/Sinks/Out1', [model '/Model_Out'], ...
    'Position', [1110, 180, 1140, 200]);

% 7) Scope 监测
add_block('simulink/Sinks/Scope', [model '/Scope_Angle_Err'], ...
    'Position', [60, 380, 180, 420]);
add_block('simulink/Sinks/Scope', [model '/Scope_Omega_Err'], ...
    'Position', [200, 380, 320, 420]);


%% ===== 配置 Signal_Interface 子系统 =====
si = [model '/Signal_Interface'];

% 删除默认 In1/Out1
delete_block([si '/In1']);
delete_block([si '/Out1']);

% 输入：theta, omega, M_pid, d_hat, M, J_t, D_t, water_ratio
add_block('simulink/Sources/In1', [si '/theta_in'], 'Position', [25, 35, 55, 55]);
add_block('simulink/Sources/In1', [si '/omega_in'], 'Position', [25, 75, 55, 95]);
add_block('simulink/Sources/In1', [si '/Mpid_in'], 'Position', [25, 115, 55, 135]);
add_block('simulink/Sources/In1', [si '/dhat_in'], 'Position', [25, 155, 55, 175]);
add_block('simulink/Sources/In1', [si '/M_in'], 'Position', [25, 195, 55, 215]);
add_block('simulink/Sources/In1', [si '/Jt_in'], 'Position', [25, 235, 55, 255]);
add_block('simulink/Sources/In1', [si '/Dt_in'], 'Position', [25, 275, 55, 295]);
add_block('simulink/Sources/In1', [si '/ratio_in'], 'Position', [25, 315, 55, 335]);

% 角度外环（保持原控制结构）: omega_ref = PI(theta_ref - theta)
add_block('simulink/Sources/Constant', [si '/Theta_Ref'], 'Position', [95, 20, 145, 40]);
set_param([si '/Theta_Ref'], 'Value', 'theta_ref');

add_block('simulink/Math Operations/Subtract', [si '/Angle_Err'], 'Position', [180, 35, 210, 65]);
add_block('simulink/Continuous/PID Controller', [si '/Angle_PI'], 'Position', [240, 30, 310, 70]);
set_param([si '/Angle_PI'], 'P', 'Kp_theta', 'I', 'Ki_theta', 'D', 'Kd_theta');

% 向量打包输出（便于减少顶层输出端口）
add_block('simulink/Signal Routing/Mux', [si '/Output_Mux'], 'Position', [340, 150, 370, 290]);
set_param([si '/Output_Mux'], 'Inputs', '8');

% 输出：omega_ref、y_vec、Angle_Err
add_block('simulink/Sinks/Out1', [si '/omega_ref_out'], 'Position', [400, 45, 430, 65]);
add_block('simulink/Sinks/Out1', [si '/Angle_Err_out'], 'Position', [400, 90, 430, 110]);
add_block('simulink/Sinks/Out1', [si '/y_vec_out'], 'Position', [400, 210, 430, 230]);

% 连线
add_line(si, 'Theta_Ref/1', 'Angle_Err/1');
add_line(si, 'theta_in/1', 'Angle_Err/2');
add_line(si, 'Angle_Err/1', 'Angle_PI/1');
add_line(si, 'Angle_Err/1', 'Angle_Err_out/1');  % 引出 Angle_Err
add_line(si, 'Angle_PI/1', 'omega_ref_out/1');

add_line(si, 'theta_in/1', 'Output_Mux/1');
add_line(si, 'omega_in/1', 'Output_Mux/2');
add_line(si, 'Mpid_in/1', 'Output_Mux/3');
add_line(si, 'dhat_in/1', 'Output_Mux/4');
add_line(si, 'M_in/1', 'Output_Mux/5');
add_line(si, 'Jt_in/1', 'Output_Mux/6');
add_line(si, 'Dt_in/1', 'Output_Mux/7');
add_line(si, 'ratio_in/1', 'Output_Mux/8');
add_line(si, 'Output_Mux/1', 'y_vec_out/1');

%% ===== 配置 Variable_PID 子系统 =====
vp = [model '/Variable_PID'];
delete_block([vp '/In1']);
delete_block([vp '/Out1']);

% 输入：omega_ref, omega, theta
add_block('simulink/Sources/In1', [vp '/omega_ref_in'], 'Position', [30, 40, 60, 60]);
add_block('simulink/Sources/In1', [vp '/omega_in'], 'Position', [30, 90, 60, 110]);
add_block('simulink/Sources/In1', [vp '/theta_in'], 'Position', [30, 140, 60, 160]);

% e_omega
add_block('simulink/Math Operations/Subtract', [vp '/Omega_Err'], 'Position', [95, 55, 125, 85]);

% abs 信号
add_block('simulink/Math Operations/Abs', [vp '/Abs_Omega'], 'Position', [95, 105, 125, 135]);
add_block('simulink/Math Operations/Abs', [vp '/Abs_Theta'], 'Position', [95, 155, 125, 185]);

% Kp = Kp0 + a*|omega|
add_block('simulink/Sources/Constant', [vp '/Kp0'], 'Position', [150, 15, 190, 35]);
set_param([vp '/Kp0'], 'Value', 'Kp0_omega');
add_block('simulink/Math Operations/Gain', [vp '/Gain_a'], 'Position', [150, 105, 190, 135]);
set_param([vp '/Gain_a'], 'Gain', 'a_omega');
add_block('simulink/Math Operations/Add', [vp '/Kp_Sum'], 'Position', [220, 80, 250, 110]);

% Kd = Kd0 + b*|omega|
add_block('simulink/Sources/Constant', [vp '/Kd0'], 'Position', [150, 45, 190, 65]);
set_param([vp '/Kd0'], 'Value', 'Kd0_omega');
add_block('simulink/Math Operations/Gain', [vp '/Gain_b'], 'Position', [150, 145, 190, 175]);
set_param([vp '/Gain_b'], 'Gain', 'b_omega');
add_block('simulink/Math Operations/Add', [vp '/Kd_Sum'], 'Position', [220, 125, 250, 155]);

% Ki = Ki0 * exp(-c*|theta|)
add_block('simulink/Math Operations/Gain', [vp '/Gain_neg_c'], 'Position', [150, 185, 190, 215]);
set_param([vp '/Gain_neg_c'], 'Gain', '-c_theta');
add_block('simulink/Math Operations/Math Function', [vp '/Exp_Block'], 'Position', [220, 185, 260, 215]);
set_param([vp '/Exp_Block'], 'Operator', 'exp');
add_block('simulink/Math Operations/Gain', [vp '/Gain_Ki0'], 'Position', [290, 185, 330, 215]);
set_param([vp '/Gain_Ki0'], 'Gain', 'Ki0_omega');

% P / I / D
add_block('simulink/Continuous/Integrator', [vp '/Int_eOmega'], 'Position', [290, 40, 320, 70]);
add_block('simulink/Math Operations/Product', [vp '/P_Term'], 'Position', [290, 85, 330, 115]);
add_block('simulink/Math Operations/Product', [vp '/I_Term'], 'Position', [360, 40, 400, 70]);
add_block('simulink/Math Operations/Product', [vp '/D_Term'], 'Position', [360, 130, 400, 160]);

% === 关键修复：添加Derivative模块 ===
add_block('simulink/Continuous/Derivative', [vp '/De_OmegaErr'], 'Position', [220, 145, 260, 175]);

add_block('simulink/Math Operations/Add', [vp '/Mpid_Sum'], 'Position', [470, 90, 500, 120]);
set_param([vp '/Mpid_Sum'], 'Inputs', '+++');

% 输出：M_pid、Omega_Err
add_block('simulink/Sinks/Out1', [vp '/Mpid_out'], 'Position', [535, 95, 565, 115]);
add_block('simulink/Sinks/Out1', [vp '/Omega_Err_out'], 'Position', [535, 140, 565, 160]);

% 连线
add_line(vp, 'omega_ref_in/1', 'Omega_Err/1');
add_line(vp, 'omega_in/1', 'Omega_Err/2');
add_line(vp, 'Omega_Err/1', 'Omega_Err_out/1');  % 引出 Omega_Err

add_line(vp, 'omega_in/1', 'Abs_Omega/1');
add_line(vp, 'theta_in/1', 'Abs_Theta/1');

add_line(vp, 'Abs_Omega/1', 'Gain_a/1');
add_line(vp, 'Kp0/1', 'Kp_Sum/1');
add_line(vp, 'Gain_a/1', 'Kp_Sum/2');

add_line(vp, 'Abs_Omega/1', 'Gain_b/1');
add_line(vp, 'Kd0/1', 'Kd_Sum/1');
add_line(vp, 'Gain_b/1', 'Kd_Sum/2');

add_line(vp, 'Abs_Theta/1', 'Gain_neg_c/1');
add_line(vp, 'Gain_neg_c/1', 'Exp_Block/1');
add_line(vp, 'Exp_Block/1', 'Gain_Ki0/1');

add_line(vp, 'Omega_Err/1', 'Int_eOmega/1');
add_line(vp, 'Kp_Sum/1', 'P_Term/1');
add_line(vp, 'Omega_Err/1', 'P_Term/2');

add_line(vp, 'Gain_Ki0/1', 'I_Term/1');
add_line(vp, 'Int_eOmega/1', 'I_Term/2');

% === 关键修复：D项连线（使用微分） ===
add_line(vp, 'Omega_Err/1', 'De_OmegaErr/1');      % e_omega -> 微分
add_line(vp, 'De_OmegaErr/1', 'D_Term/2');        % d(e_omega)/dt -> D_Term
add_line(vp, 'Kd_Sum/1', 'D_Term/1');             % Kd -> D_Term

add_line(vp, 'P_Term/1', 'Mpid_Sum/1');
add_line(vp, 'I_Term/1', 'Mpid_Sum/2');
add_line(vp, 'D_Term/1', 'Mpid_Sum/3');
add_line(vp, 'Mpid_Sum/1', 'Mpid_out/1');

%% ===== 配置 Disturbance_Observer 子系统 =====
do = [model '/Disturbance_Observer'];
delete_block([do '/In1']);
delete_block([do '/Out1']);

% 输入：omega, J_t, M, D_t, theta
add_block('simulink/Sources/In1', [do '/omega_in'], 'Position', [30, 50, 60, 70]);
add_block('simulink/Sources/In1', [do '/Jt_in'], 'Position', [30, 100, 60, 120]);
add_block('simulink/Sources/In1', [do '/M_in'], 'Position', [30, 150, 60, 170]);
add_block('simulink/Sources/In1', [do '/Dt_in'], 'Position', [30, 200, 60, 220]);
add_block('simulink/Sources/In1', [do '/theta_in'], 'Position', [30, 250, 60, 270]);

% 计算 omega 的导数
add_block('simulink/Continuous/Derivative', [do '/Omega_Derivative'], 'Position', [95, 45, 135, 75]);

% 计算 J_t * d(omega)/dt
add_block('simulink/Math Operations/Product', [do '/J_OmegaDot_Product'], 'Position', [170, 65, 210, 95]);

% 计算 D_t * omega（已知阻尼项）
add_block('simulink/Math Operations/Product', [do '/Domega_Product'], 'Position', [170, 195, 210, 225]);

% 计算 K_theta * theta（已知恢复力项）
add_block('simulink/Math Operations/Gain', [do '/Ktheta_Gain'], 'Position', [170, 245, 210, 275]);
set_param([do '/Ktheta_Gain'], 'Gain', 'K_theta');

% 计算残差：Residual = M - J*dω/dt - D*ω - K*θ
add_block('simulink/Math Operations/Add', [do '/Residual_Sum'], 'Position', [245, 95, 275, 125]);
set_param([do '/Residual_Sum'], 'Inputs', '+---');  % M - J*dω/dt - D*ω - K*θ

% 低通滤波器
add_block('simulink/Continuous/Transfer Fcn', [do '/ESO_LPF'], 'Position', [310, 95, 380, 125]);
set_param([do '/ESO_LPF'], 'Numerator', '[1]', 'Denominator', '[tau_eso 1]');

% 输出：d_hat
add_block('simulink/Sinks/Out1', [do '/dhat_out'], 'Position', [415, 100, 445, 120]);

% 连线（修正符号）
add_line(do, 'omega_in/1', 'Omega_Derivative/1');
add_line(do, 'Omega_Derivative/1', 'J_OmegaDot_Product/1');
add_line(do, 'Jt_in/1', 'J_OmegaDot_Product/2');

% 已知动力学项
add_line(do, 'Dt_in/1', 'Domega_Product/1');
add_line(do, 'omega_in/1', 'Domega_Product/2');
add_line(do, 'theta_in/1', 'Ktheta_Gain/1');

% 残差计算（修正符号：M - J*dω/dt - D*ω - K*θ）
add_line(do, 'M_in/1', 'Residual_Sum/1');              % input1: M（正）
add_line(do, 'J_OmegaDot_Product/1', 'Residual_Sum/2'); % input2: J*dω/dt（负）
add_line(do, 'Domega_Product/1', 'Residual_Sum/3');     % input3: D*ω（负）
add_line(do, 'Ktheta_Gain/1', 'Residual_Sum/4');        % input4: K*θ（负）

add_line(do, 'Residual_Sum/1', 'ESO_LPF/1');
add_line(do, 'ESO_LPF/1', 'dhat_out/1');

%% ===== 配置 Water_Dynamics 子系统 =====
wd = [model '/Water_Dynamics'];
delete_block([wd '/In1']);
delete_block([wd '/Out1']);

% 输入：M
add_block('simulink/Sources/In1', [wd '/M_in'], 'Position', [25, 105, 55, 125]);

% 时变参数 J(t), D(t), ratio
add_block('simulink/Sources/Clock', [wd '/Clock'], 'Position', [25, 20, 55, 40]);
add_block('simulink/Sources/Constant', [wd '/Neg_t_enter'], 'Position', [70, 20, 130, 40]);
set_param([wd '/Neg_t_enter'], 'Value', '-t_enter');
add_block('simulink/Math Operations/Add', [wd '/Time_Shift'], 'Position', [155, 20, 185, 50]);
add_block('simulink/Math Operations/Gain', [wd '/Inv_t_blend'], 'Position', [210, 20, 250, 50]);
set_param([wd '/Inv_t_blend'], 'Gain', '1/t_blend');
add_block('simulink/Discontinuities/Saturation', [wd '/Ratio_Sat'], 'Position', [275, 20, 315, 50]);
set_param([wd '/Ratio_Sat'], 'UpperLimit', '1', 'LowerLimit', '0');
add_block('simulink/User-Defined Functions/Fcn', [wd '/Smoothstep'], 'Position', [340, 20, 410, 50]);
set_param([wd '/Smoothstep'], 'Expr', 'u*u*(3-2*u)');

% J_t
add_block('simulink/Sources/Constant', [wd '/J_air'], 'Position', [440, 10, 490, 30]);
set_param([wd '/J_air'], 'Value', 'J_air');
add_block('simulink/Sources/Constant', [wd '/Delta_J'], 'Position', [440, 40, 500, 60]);
set_param([wd '/Delta_J'], 'Value', 'J_water-J_air');
add_block('simulink/Math Operations/Product', [wd '/J_Delta_Product'], 'Position', [530, 30, 570, 60]);
add_block('simulink/Math Operations/Add', [wd '/J_t_Sum'], 'Position', [600, 20, 630, 50]);

% D_t
add_block('simulink/Sources/Constant', [wd '/D_air'], 'Position', [440, 80, 490, 100]);
set_param([wd '/D_air'], 'Value', 'D_air');
add_block('simulink/Sources/Constant', [wd '/Delta_D'], 'Position', [440, 110, 500, 130]);
set_param([wd '/Delta_D'], 'Value', 'D_water-D_air');
add_block('simulink/Math Operations/Product', [wd '/D_Delta_Product'], 'Position', [530, 100, 570, 130]);
add_block('simulink/Math Operations/Add', [wd '/D_t_Sum'], 'Position', [600, 90, 630, 120]);

% ========== 入水冲击项建模 ==========
% 目标：M_total = M + A_impact * exp(-(t - t_enter)/tau_impact), t >= t_enter

% 时间偏移：计算 (t - t_enter)
add_block('simulink/Sources/Clock', [wd '/Clock_Impact']);
add_block('simulink/Sources/Constant', [wd '/Neg_t_enter_Impact']);
set_param([wd '/Neg_t_enter_Impact'], 'Value', '-t_enter');
add_block('simulink/Math Operations/Add', [wd '/Time_Shift_Impact']);

% 指数衰减：exp(-(t - t_enter)/tau_impact)
add_block('simulink/Math Operations/Gain', [wd '/Impact_Decay']);
set_param([wd '/Impact_Decay'], 'Gain', '-1/tau_impact');
add_block('simulink/Math Operations/Math Function', [wd '/Exp_Impact']);
set_param([wd '/Exp_Impact'], 'Operator', 'exp');

% 冲击幅值
add_block('simulink/Sources/Constant', [wd '/Impact_Amp']);
set_param([wd '/Impact_Amp'], 'Value', 'A_impact');
add_block('simulink/Math Operations/Product', [wd '/Impact_Product']);

% 触发条件：(t - t_enter) >= 0
add_block('simulink/Logic and Bit Operations/Compare To Constant', [wd '/Impact_Enable']);
set_param([wd '/Impact_Enable'], 'const', '0', 'relop', '>=');

% Switch：仅在 t >= t_enter 时输出冲击
add_block('simulink/Signal Routing/Switch', [wd '/Impact_Switch']);

% M_total = M + Impact
add_block('simulink/Math Operations/Add', [wd '/M_with_Impact']);

% 动力学主方程：J*omega_dot = M - D*omega + K*theta
add_block('simulink/Math Operations/Product', [wd '/Domega_Product'], 'Position', [220, 170, 260, 200]);
add_block('simulink/Math Operations/Gain', [wd '/Ktheta_Gain'], 'Position', [220, 220, 270, 250]);
set_param([wd '/Ktheta_Gain'], 'Gain', 'K_theta');

add_block('simulink/Math Operations/Add', [wd '/OmegaDot_Num'], 'Position', [300, 165, 330, 205]);
set_param([wd '/OmegaDot_Num'], 'Inputs', '+--');  % 修正：M - D·ω - K·θ

add_block('simulink/Math Operations/Divide', [wd '/Divide_J'], 'Position', [360, 170, 400, 200]);

add_block('simulink/Continuous/Integrator', [wd '/Omega_Int'], 'Position', [440, 170, 470, 200]);
set_param([wd '/Omega_Int'], 'InitialCondition', 'omega0');
add_block('simulink/Continuous/Integrator', [wd '/Theta_Int'], 'Position', [520, 170, 550, 200]);
set_param([wd '/Theta_Int'], 'InitialCondition', 'theta0');

% 输出：omega, theta, J_t, D_t, ratio
add_block('simulink/Sinks/Out1', [wd '/omega_out'], 'Position', [700, 160, 730, 180]);
add_block('simulink/Sinks/Out1', [wd '/theta_out'], 'Position', [700, 190, 730, 210]);
add_block('simulink/Sinks/Out1', [wd '/Jt_out'], 'Position', [700, 20, 730, 40]);
add_block('simulink/Sinks/Out1', [wd '/Dt_out'], 'Position', [700, 80, 730, 100]);
add_block('simulink/Sinks/Out1', [wd '/ratio_out'], 'Position', [700, 120, 730, 140]);

% 连线：ratio 与 J/D
add_line(wd, 'Clock/1', 'Time_Shift/1');
add_line(wd, 'Neg_t_enter/1', 'Time_Shift/2');
add_line(wd, 'Time_Shift/1', 'Inv_t_blend/1');
add_line(wd, 'Inv_t_blend/1', 'Ratio_Sat/1');
% 绕过 Smoothstep，直接使用 Ratio_Sat 实现阶跃式入水

add_line(wd, 'Ratio_Sat/1', 'J_Delta_Product/1');
add_line(wd, 'Delta_J/1', 'J_Delta_Product/2');
add_line(wd, 'J_air/1', 'J_t_Sum/1');
add_line(wd, 'J_Delta_Product/1', 'J_t_Sum/2');

add_line(wd, 'Ratio_Sat/1', 'D_Delta_Product/1');
add_line(wd, 'Delta_D/1', 'D_Delta_Product/2');
add_line(wd, 'D_air/1', 'D_t_Sum/1');
add_line(wd, 'D_Delta_Product/1', 'D_t_Sum/2');

% ========== 冲击项连线 ==========
% 时间偏移：Clock_Impact + (-t_enter) = (t - t_enter)
add_line(wd, 'Clock_Impact/1', 'Time_Shift_Impact/1');
add_line(wd, 'Neg_t_enter_Impact/1', 'Time_Shift_Impact/2');

% 指数衰减链：(t - t_enter) → Gain(-1/tau) → Exp
add_line(wd, 'Time_Shift_Impact/1', 'Impact_Decay/1');
add_line(wd, 'Impact_Decay/1', 'Exp_Impact/1');

% 幅值：Exp × A_impact → Impact
add_line(wd, 'Exp_Impact/1', 'Impact_Product/1');
add_line(wd, 'Impact_Amp/1', 'Impact_Product/2');

% 触发条件：(t - t_enter) >= 0
add_line(wd, 'Time_Shift_Impact/1', 'Impact_Enable/1');

% Switch：Impact_Product → in1, Impact_Enable → cond, 0 → in2 (默认)
add_line(wd, 'Impact_Product/1', 'Impact_Switch/1');
add_line(wd, 'Impact_Enable/1', 'Impact_Switch/2');

% 连线：动力学方程
add_line(wd, 'D_t_Sum/1', 'Domega_Product/1');
add_line(wd, 'Omega_Int/1', 'Domega_Product/2');
add_line(wd, 'Theta_Int/1', 'Ktheta_Gain/1');

add_line(wd, 'M_in/1', 'M_with_Impact/1');
add_line(wd, 'Impact_Switch/1', 'M_with_Impact/2');
add_line(wd, 'M_with_Impact/1', 'OmegaDot_Num/1');
add_line(wd, 'Domega_Product/1', 'OmegaDot_Num/2');
add_line(wd, 'Ktheta_Gain/1', 'OmegaDot_Num/3');

add_line(wd, 'OmegaDot_Num/1', 'Divide_J/1');
add_line(wd, 'J_t_Sum/1', 'Divide_J/2');
add_line(wd, 'Divide_J/1', 'Omega_Int/1');
add_line(wd, 'Omega_Int/1', 'Theta_Int/1');

% 连线：输出
add_line(wd, 'Omega_Int/1', 'omega_out/1');
add_line(wd, 'Theta_Int/1', 'theta_out/1');
add_line(wd, 'J_t_Sum/1', 'Jt_out/1');
add_line(wd, 'D_t_Sum/1', 'Dt_out/1');
add_line(wd, 'Ratio_Sat/1', 'ratio_out/1');

%% ===== 顶层连线 =====
% Water_Dynamics -> Signal_Interface
add_line(model, 'Water_Dynamics/2', 'Signal_Interface/1', 'Autorouting', 'on'); % theta
add_line(model, 'Water_Dynamics/1', 'Signal_Interface/2', 'Autorouting', 'on'); % omega

% Signal_Interface -> Variable_PID
add_line(model, 'Signal_Interface/1', 'Variable_PID/1', 'Autorouting', 'on');    % omega_ref

% Water_Dynamics -> Variable_PID
add_line(model, 'Water_Dynamics/1', 'Variable_PID/2', 'Autorouting', 'on');      % omega
add_line(model, 'Water_Dynamics/2', 'Variable_PID/3', 'Autorouting', 'on');      % theta

% Variable_PID -> M_Comp 与 Signal_Interface
add_line(model, 'Variable_PID/1', 'M_Comp/1', 'Autorouting', 'on');              % M_pid
add_line(model, 'Variable_PID/1', 'Signal_Interface/3', 'Autorouting', 'on');    % Mpid_in

% Disturbance_Observer 输入
add_line(model, 'Water_Dynamics/1', 'Disturbance_Observer/1', 'Autorouting', 'on'); % omega
add_line(model, 'Water_Dynamics/3', 'Disturbance_Observer/2', 'Autorouting', 'on'); % J_t
add_line(model, 'M_Comp/1', 'Disturbance_Observer/3', 'Autorouting', 'on');         % M
add_line(model, 'Water_Dynamics/4', 'Disturbance_Observer/4', 'Autorouting', 'on'); % D_t
add_line(model, 'Water_Dynamics/2', 'Disturbance_Observer/5', 'Autorouting', 'on'); % theta

% Disturbance_Observer -> M_Comp 与 Signal_Interface
add_line(model, 'Disturbance_Observer/1', 'M_Comp/2', 'Autorouting', 'on');      % d_hat
add_line(model, 'Disturbance_Observer/1', 'Signal_Interface/4', 'Autorouting', 'on');

% M_Comp -> Water_Dynamics 与 Signal_Interface
add_line(model, 'M_Comp/1', 'Water_Dynamics/1', 'Autorouting', 'on');
add_line(model, 'M_Comp/1', 'Signal_Interface/5', 'Autorouting', 'on');

% Water_Dynamics 其余输出 -> Signal_Interface
add_line(model, 'Water_Dynamics/3', 'Signal_Interface/6', 'Autorouting', 'on'); % J_t
add_line(model, 'Water_Dynamics/4', 'Signal_Interface/7', 'Autorouting', 'on'); % D_t
add_line(model, 'Water_Dynamics/5', 'Signal_Interface/8', 'Autorouting', 'on'); % ratio

% 单输出向量
add_line(model, 'Signal_Interface/3', 'Model_Out/1', 'Autorouting', 'on');

% Scope 连线：监测误差
add_line(model, 'Signal_Interface/2', 'Scope_Angle_Err/1', 'Autorouting', 'on');  % Angle_Err
add_line(model, 'Variable_PID/2', 'Scope_Omega_Err/1', 'Autorouting', 'on');% Omega_Err


%% ===== 参数写入工作区 =====
% 参考与外环（极保守参数，确保稳定性）
theta_ref = -10*pi/180;
Kp_theta = 9;  % 极小增益，确保稳定
Ki_theta = 0.1;
Kd_theta = 0.8;

% 变参数 PID（内环：极保守参数，禁用积分）
Kp0_omega = 50;  % 极小增益
Ki0_omega = 0.8;    % 完全不用积分
Kd0_omega = 0.1;  % 小微分增益
a_omega =550;
b_omega = 4;
c_theta = 4;

% 入水动力学
J_air = 5;
J_water = 8;
D_air = 0.5;
D_water = 2.0;
K_theta = 1.2;

% 初值与切换
theta0 = 0;
omega0 = 0;
t_enter = 2.0;
t_blend = 0.02;   % 20 ms 毫秒级入水

% 入水冲击参数
A_impact = 100;      % 冲击强度（建议100~300）
tau_impact = 0.05;   % 衰减时间常数（20ms）

% 观测器（修正后使用适中参数）
tau_eso = 0.01;  % 适中值，平衡响应速度和噪声抑制

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
assignin('base', 'A_impact', A_impact);
assignin('base', 'tau_impact', tau_impact);
assignin('base', 'tau_eso', tau_eso);

%% 保存
save_system(model, modelPath);
fprintf('模型已保存: %s\n', modelPath);
fprintf('完成：D项已修复为真正的微分项！\n');
fprintf('修复内容：\n');
fprintf('  - 添加Derivative模块计算d(e_omega)/dt\n');
fprintf('  - D项从 Kd*omega 改为 Kd*d(e_omega)/dt\n');
fprintf('  - 现在Kd将正常发挥微分作用！\n');
