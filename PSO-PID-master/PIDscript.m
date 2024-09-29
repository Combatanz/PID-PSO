% 开发人员：Jason
% 开发地点：Tsinghua University
% 开发时间：2024.9.25
% 开发内容：PID控制程序(基于kalman滤波)

clear;
clc;

% ---------------- 参数设置 ----------------

% 采样时间
Ts = 0.01;    % 采样时间，单位：秒

% PID控制器增益
Kp = 1.0;     % 比例增益
Ki = 0.5;     % 积分增益
Kd = 0.1;     % 微分增益

% 参考信号（设定值）
setpoint = 200.0;  % 设定值

% 系统参数（可调整模拟真实系统的传递函数）
sysNum = [99];              % 被控对象的传递函数的分子
sysDen = [1 10 20];         % 被控对象的传递函数的分母    类似simulink的传递函数模式
sys = tf(sysNum, sysDen);

% 离散化系统（用于仿真）
sys_d = c2d(sys, Ts, 'zoh');  % 零阶保持离散化

% 仿真时间
simTime = 100;    % 仿真总时间

% 初始化变量
error_prev = 0;  % 上一次的误差
integral = 0;    % 积分项初值
u_prev = 0;      % 上一次的控制输出

% 存储数据
timeArray = 0:Ts:simTime;     % 时间向量
n = length(timeArray);        % 时间向量长度
errorArray = zeros(1, n);     % 误差数组   ARRAY  
controlArray = zeros(1, n);   % 控制器输出  
outputArray = zeros(1, n);    % 系统输出

% ---------------- 主循环：PID 控制器 ----------------

for i = 1:n
    % 当前时间
    t = timeArray(i);
    
    % 系统输出
    if i == 1
        output = 0;  % 初始系统输出设为0
    else
        % 确保至少有两个采样点传递给 lsim
        if i > 2
            output = lsim(sys_d, controlArray(1:i-1), timeArray(1:i-1));  % 离散系统仿真输出
            output = output(end);  % 获取最新输出
        else
            output = outputArray(i-1);  % 前几次迭代时，直接使用上次的输出
        end
    end
    
    % 计算误差（设定值 - 系统输出）
    error = setpoint - output;
    
    % 比例项
    P = Kp * error;
    
    % 积分项
    integral = integral + error * Ts;
    I = Ki * integral;
    
    % 微分项
    D = Kd * (error - error_prev) / Ts;
    
    % PID控制器输出
    u = P + I + D;
    
    % 存储数据
    errorArray(i) = error;
    controlArray(i) = u;
    outputArray(i) = output;
    
    % 更新上一次误差
    error_prev = error;
end

% ---------------- 绘图 ----------------

figure;
subplot(3,1,1);
plot(timeArray, errorArray, 'r', 'LineWidth', 2);
title('误差 (Error)');
xlabel('时间 (s)');
ylabel('误差');

subplot(3,1,2);
plot(timeArray, controlArray, 'b', 'LineWidth', 2);
title('控制器输出 (Control Output)');
xlabel('时间 (s)');
ylabel('控制信号');

subplot(3,1,3);
plot(timeArray, outputArray, 'g', 'LineWidth', 2);
title('系统输出 (System Output)');
xlabel('时间 (s)');
ylabel('输出信号');

plot(timeArray, controlArray, 'b', 'LineWidth', 2);  % 控制器输出信号
hold on;
plot(timeArray, outputArray, 'g', 'LineWidth', 2);   % 系统输出信号
hold off;

title('控制器输出与系统输出');
xlabel('时间 (s)');
ylabel('信号');
legend('控制器输出 (Control Output)', '系统输出 (System Output)', 'Location', 'Best');

grid on;

