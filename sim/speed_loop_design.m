%% 速度环设计验证（Control System Toolbox）【Claude 代写，待复核】
% 对应 docs/M8b_速度环.md：
%   被控对象 iq -> w = (Kt/J)/(s + B/J)（M8a 实测），电流环看成一阶滞后
%   Kp = wc/(Kt/J)，Ki = Kp*wc/4；速度测量 + PI 保持的等效延迟 Tw
% 预期：穿越 ~51 Hz，相位裕度 ~55°，增益裕度 ~12.5 dB

clear; close all;

%% 被控对象参数（M8a 实测，docs/M8a_q轴电流测试.md）
KtJ = 1.1e4;      % Kt/J [rad/(s^2*A)]
BJ  = 0.97;       % B/J  [1/s]
TcJ = 183;        % 库仑摩擦 Tc/J [rad/s^2]，阶跃响应仿真里当作恒定扰动
Tw  = 1e-3;       % 速度环周期 [s]
tau_i = 1/(2*pi*1000);   % 电流环（带宽 1 kHz）近似成一阶滞后的时间常数 [s]
iq_max = 0.4;     % iq_ref 限幅 [A]

%% 设计
fc = 50;  wc = 2*pi*fc;
Kp = wc/KtJ;              % [A/(rad/s)]
Ki = Kp*wc/4;             % [A/rad]，PI 零点在 wc/4
fprintf('Kp = %.4f A/(rad/s),  Ki = %.3f A/rad,  Ki*Tw = %.5f\n', Kp, Ki, Ki*Tw);

%% 传递函数
s     = tf('s');
G_m   = KtJ/(s + BJ);           % 机械：iq -> w
G_i   = 1/(tau_i*s + 1);        % 电流环：iq_ref -> iq
G_d   = exp(-Tw*s);             % 测速平均 Tw/2 + PI 输出保持 Tw/2
G_pi  = Kp + Ki/s;

G_L   = G_pi*G_d*G_i*G_m;
G_cl  = feedback(G_L, 1);

[GM, PM, w_pc, w_c] = margin(G_L);
fprintf('穿越频率 = %.1f Hz,  相位裕度 = %.1f°,  增益裕度 = %.1f dB（%.0f Hz）\n', ...
        w_c/2/pi, PM, 20*log10(GM), w_pc/2/pi);

%% 小阶跃（线性，不含摩擦和限幅）：100 rpm
rpm = 2*pi/60;
info = stepinfo(G_cl);
fprintf('超调 = %.0f%%,  上升时间(10%%->90%%) = %.1f ms,  调节时间(2%%) = %.0f ms\n', ...
        info.Overshoot, info.RiseTime*1e3, info.SettlingTime*1e3);

figure('Name', '速度环 Bode 图'); margin(G_L); grid on;
figure('Name', '速度环阶跃响应 600 -> 700 rpm');
[y, t] = step(G_cl*100, 0.08);
plot(t*1e3, 600 + y); grid on; xlabel('t [ms]'); ylabel('\omega [rpm]');

%% 不同穿越频率对比
fprintf('\n穿越     相位裕度   超调\n');
for f = [20 30 50 80 100]
    w  = 2*pi*f;  kp = w/KtJ;  ki = kp*w/4;
    GL = (kp + ki/s)*G_d*G_i*G_m;
    [~, pm] = margin(GL);
    si = stepinfo(feedback(GL, 1));
    fprintf('%4d Hz   %5.1f°   %5.0f%%\n', f, pm, si.Overshoot);
end
