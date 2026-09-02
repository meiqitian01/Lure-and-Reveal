% MATLAB代码：UAV动力学方程PD控制器设计

% 初始化参数
A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];

% 初始条件
x = [0; 0; 0; 10; 0; 0];  % 初始状态 [px, py, pz, vx, vy, vz]
x_hat = x;              % 初始估计状态
x_bar = [0; 0; 0; 2; 0; 0];  % 初始desired状态

% 仿真参数
dt = 0.1;               % 时间步长 (秒)
T = 50;                 % 总仿真时间 (秒)
time = 0:dt:T;          % 时间向量

% PD控制器增益
Kp = diag([2, 2, 2, 0.5, 0.5, 0.5]);  % 比例增益
Kd = diag([1, 1, 1, 0.2, 0.2, 0.2]);  % 微分增益

% 记录状态
state_log = zeros(6, length(time));

% 仿真循环
for t = 1:length(time)
    % 更新desired状态
    if x_bar(1) < 100  % 目标位置 px 达到 100 m
        x_bar(1) = x_bar(1) + 2 * dt;  % 以 2 m/s 速度前进
    end
    
    % 计算控制输入 a = Kp * (x_hat - x_bar) + Kd * (x_dot_est - x_dot_des)
    position_error = x_hat(1:3) - x_bar(1:3);  % 位置误差
    velocity_error = x_hat(4:6) - x_bar(4:6);  % 速度误差
    a = -Kp * position_error - Kd * velocity_error;
    
    % 动力学方程 x_dot = A*x + B*a
    x_dot = A * x + B * a;
    
    % 更新状态 x (积分)
    x = x + x_dot * dt;
    
    % 更新估计状态（假设估计值等于实际值）
    x_hat = x;
    
    % 记录状态
    state_log(:, t) = x;
end

% 绘制状态随时间变化的图
figure;
subplot(3, 1, 1);
plot(time, state_log(1, :));
title('Position in North Direction (px)');
xlabel('Time (s)');
ylabel('Position (m)');

title('UAV State Evolution');
subplot(3, 1, 2);
plot(time, state_log(4, :));
title('Velocity in North Direction (vx)');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

subplot(3, 1, 3);
plot(time, state_log(1, :), 'r', 'DisplayName', 'px'); hold on;
plot(time, state_log(4, :), 'b', 'DisplayName', 'vx');
legend;
xlabel('Time (s)');
title('Combined Position and Velocity');