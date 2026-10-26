%% 电流环设计验证（第一阶段：只用 Control System Toolbox，不用 Simulink）
% 验证 docs/电流环控制基础.md 中的手算结果：
%   Kp = L*w_bw, Ki = R*w_bw；相位裕度 = 90° - 360°*f_bw*Td；无稳态误差

clear; close all;

%% 电机参数（M4b / M4c 实测）
R  = 3.21;        % 相电阻 [Ω]
L  = 0.88e-3;     % 相电感 [H]
Td = 75e-6;       % 延迟 [s]，通用 1.5*Ts（保守）
Ts = 50e-6;       % PWM / 控制周期 [s]（20 kHz），离散模型用
Vbus = 12;        % 母线电压 [V]
Vmax = Vbus/sqrt(3);   % SVPWM 线性区最大相电压幅值 ≈ 6.93 V，PI 输出限幅用

%% 设计：选带宽，算 Kp、Ki
f_bw = 1000;                  % 带宽 [Hz]
w_bw = 2*pi*f_bw;             % 角频率带宽 [rad/s]
Kp = L*w_bw;                  % [V/A]
Ki = R*w_bw;                  % [V/(A*s)]
fprintf('Kp = %.3f V/A,  Ki = %.0f V/(A*s),  Ki*Ts = %.3f V/A\n', Kp, Ki, Ki*50e-6);

%% 三个方块的传递函数
s   = tf('s');
G_m  = 1/(L*s + R);           % 电机：电压 -> 电流
G_pi = Kp + Ki/s;             % PI：误差 -> 电压
G_d  = exp(-Td*s);            % 延迟

%% 环路与闭环
G_L  = G_pi*G_d*G_m;          % 环路：误差 -> 电流（串联相乘）
G_cl = feedback(G_L, 1);      % 闭环：G_L/(1+G_L)，目标电流 -> 实际电流

%% 相位裕度（看环路 G_L）
[~, PM, ~, w_c] = margin(G_L);
fprintf('穿越频率 = %.0f Hz,  相位裕度 = %.1f°   (手算 1000 Hz, %.1f°)\n', ...
        w_c/2/pi, PM, 90 - 360*f_bw*Td);

%% 稳态（看闭环 G_cl 在 s = 0）
fprintf('G_cl(0) = %.4f   (手算 1，无稳态误差)\n', dcgain(G_cl));

%% 阶跃响应
info = stepinfo(G_cl);
fprintf('超调 = %.1f%%,  上升时间(10%%→90%%) = %.3f ms\n', info.Overshoot, info.RiseTime*1e3);

%% 画图
figure('Name', '环路 Bode 图'); margin(G_L); grid on;
figure('Name', '闭环阶跃响应'); step(G_cl, 1e-3); grid on;

%% 对比不同带宽
fprintf('\n带宽     相位裕度   超调\n');
for f = [500 1000 1500 2000 2500]
    w  = 2*pi*f;
    GL = (L*w + R*w/s)*G_d*G_m;
    [~, pm] = margin(GL);
    si = stepinfo(feedback(GL, 1));
    fprintf('%5d Hz   %5.1f°   %5.1f%%\n', f, pm, si.Overshoot);
end
