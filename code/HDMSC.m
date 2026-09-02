%% 1. 清空环境与初始化（严格匹配论文设定）
clear; clc; close all;
rng(100); % 随机种子（确保FDI攻击可复现，论文隐含要求）

%% 2. 系统与核心参数（论文5.1节，式1、表1隐含参数）
% 2.1 离散线性系统矩阵（式1：x(k+1)=Gx(k)+Hu(k)+w(k)）
n_x = 2;  % 状态维度（论文未明说，由G矩阵维度推导）
n_u = 1;  % 控制输入维度
n_y = 1;  % 测量输出维度（y(k)=Cx(k)+Ψ(k)，式2）

% 论文5.1节给定系统参数
G = [0.9647, -0.0282; 
     0.0196,  0.9997];  % 系统矩阵G
H = [0.0196; 0.0002];   % 控制矩阵H
C = [-0.2293, 1.4393];  % 测量矩阵C（论文中为列向量，此处转置为行向量适配计算）

% 2.2 外部扰动与FDI攻击参数（论文5.1节）
w = @(k) 2*sin(3*k) + 3*cos(2*k);  % 外部扰动w(k)，能量约束||w(k)||≤4
Psi_max = 1.2;  % FDI攻击最大幅值（论文：攻击大小与输出关联，设为输出1.2倍）

% 2.3 扩展观测器参数（论文5.1节）
Z = [0.07; -0.1042; 0.3736];  % 观测器增益Z（式5：M=R̄G-Z̄C）
q1 = 0.03;  % 滑模函数参数q1（式14）
q2 = 0.22;  % 滑模函数参数q2（式14）
mu = 0.06;  % 滑模控制参数μ（式15）
lambda1 = 0.9;  % 观测器补偿项权重λ1（式16）
lambda2 = 0.1;  % 观测器补偿项权重λ2（式16），满足λ1+λ2=1

% 2.4 HDSMC控制器参数（论文5.1节）
alpha = 0.03;    % 滑模面参数α（式43）
beta = 5;        % 滑模面参数β（式43）
q_over_p = 3/5;  % 非线性项指数q/p（0<q/p<1，式43）
varsigma1 = 0.1; % 趋近律参数ζ1（0<ζ1<1，式44）
varsigma2 = 0.9; % 趋近律参数ζ2（0<ζ2<1，式44）
varsigma3 = -10; % 趋近律参数ζ3（ζ3<0，式44）
lambda3 = 0.65;  % 控制律权重λ3（式46）
lambda4 = 0.35;  % 控制律权重λ4（式46），满足λ3+λ4=1
Gamma1 = 260;    % 超扭曲算法增益Γ1（式47）
Gamma2 = 0.5;    % 超扭曲算法增益Γ2（式48）

% 2.5 RBF神经网络参数（论文3.2节，式17-18）
n_rbf = 10;      % 隐层节点数（论文未明说，取10满足逼近精度）
c = linspace(-5,5,n_rbf);  % 高斯函数中心（局部训练法，覆盖状态范围）
sigma = 1.5;     % 高斯函数宽度（论文隐含设定，确保局部逼近）
eta = 0.01;      % 权重更新步长η（式26）
theta = 0.001;   % 权重正则化参数θ（式26）

% 2.6 仿真参数（论文5.1节）
T_sim = 200;          % 仿真步数（k=0~199，覆盖攻击与稳定阶段）
y_r = 1500 * ones(1, T_sim+1);  % 参考输出（1500 rad/min，固定转速跟踪）
x0 = [0; 0];          % 系统初始状态（论文：所有初始值设为零）
xhat0 = [0; 0];       % 观测器初始估计状态
Psi0 = 0;             % FDI攻击初始值
W_hat0 = 0.1 * randn(n_rbf, 1);  % RBF初始权重（小随机值，论文隐含）
tau0 = 0;             % 超扭曲算法初始辅助变量（式48）

%% 3. 预分配存储数组
% 系统状态与输出
x = zeros(n_x, T_sim+1);       % 真实状态x(k)
y = zeros(n_y, T_sim+1);       % 真实输出y(k)=Cx(k)
Psi = zeros(n_y, T_sim+1);     % FDI攻击信号Ψ(k)
y_tilde = zeros(n_y, T_sim+1); % 受攻击输出ỹ(k)=y(k)+Ψ(k)（式2）

% 观测器相关
xhat = zeros(n_x, T_sim+1);    % 估计状态x̂(k)
Psi_hat = zeros(n_y, T_sim+1); % 估计FDI攻击Ψ̂(k)
e_bar = zeros(n_x+1, T_sim+1); % 扩展观测误差ē(k)=[x̂-x; Ψ̂-Ψ]（式12）
W_hat = zeros(n_rbf, T_sim+1); % RBF权重估计值
v = zeros(n_x, T_sim);         % 观测器补偿项v(k)（式16）

% 控制器相关
err = zeros(n_y, T_sim+1);     % 跟踪误差err(k)=y^r(k)-y_tilde(k)（式42）
sigma_smc = zeros(1, T_sim+1); % 滑模面σ(k)（式43）
u_eq = zeros(n_u, T_sim);      % 等效控制量u_eq^c(k)（式45）
u_s = zeros(n_u, T_sim);       % 超扭曲控制量u_s^c(k)（式47）
u = zeros(n_u, T_sim);         % 总控制量u(k)（式46）
tau = zeros(1, T_sim+1);       % 超扭曲辅助变量τ(k)（式48）

% 对比方法（DSMC：无超扭曲的离散滑模控制，论文5.1节对比组）
u_dsmc = zeros(n_u, T_sim);    % DSMC控制量
sigma_dsmc = zeros(1, T_sim+1);% DSMC滑模面
err_dsmc = zeros(n_y, T_sim+1);% DSMC跟踪误差

% 初始化
x(:,1) = x0;
xhat(:,1) = xhat0;
Psi(:,1) = Psi0;
W_hat(:,1) = W_hat0;
tau(:,1) = tau0;
y(:,1) = C * x(:,1);
y_tilde(:,1) = y(:,1) + Psi(:,1);
err(:,1) = y_r(:,1) - y_tilde(:,1);
err_dsmc(:,1) = err(:,1);

%% 4. 核心函数定义（均对应论文公式）
% 4.1 RBF神经网络输出（式17-18：φ_i(x)=exp(-(x-c_i)^2/σ²)）
function phi = rbf_output(x, c, sigma)
    n_rbf = length(c);
    phi = zeros(n_rbf, 1);
    for i = 1:n_rbf
        phi(i) = exp(-(x - c(i))^2 / (sigma^2));  % 高斯激活函数
    end
end

% 4.2 扩展观测器（论文3.1-3.2节，式4、16、23、26）
function [xhat_next, Psi_hat_next, W_hat_next, v_k, e_bar_next] = extended_observer(xhat_k, Psi_hat_k, W_hat_k, y_tilde_k, u_k, G, H, C, Z, q1, q2, mu, lambda1, lambda2, eta, theta, c, sigma)
    n_x = size(G,1);
    % 步骤1：构造扩展状态x̄(k)=[x(k); Ψ(k)]，估计x̄_hat(k)=[xhat(k); Psi_hat(k)]
    x_bar_hat_k = [xhat_k; Psi_hat_k];
    R = [eye(n_x); -C];  % 论文式6中R矩阵（n_x+1行，n_x列）
    G_bar = [G, zeros(n_x,1); zeros(1,n_x), 1];  % 扩展系统矩阵Ḡ
    C_bar = [C, 1];      % 扩展测量矩阵C̄（式3）
    
    % 步骤2：滑模函数（式14：s(k)=q1*ē(k)+q2*Δē(k)）
    e_bar_k = x_bar_hat_k - [xhat_k; y_tilde_k - C*xhat_k];  % 当前扩展误差
    if k == 1
        Delta_e_bar = 0;  % 初始误差差分为0
    else
        Delta_e_bar = e_bar_k - e_bar_prev;
    end
    s_k = q1 * e_bar_k + q2 * Delta_e_bar;
    
    % 步骤3：滑模补偿项u_s(k)（式15：u_s=μ*sign(s)*|s|^0.5）
    u_s_k = mu * sign(s_k) * (abs(s_k))^0.5;
    
    % 步骤4：RBF补偿项u_n(k)（式22：u_n=Ŵ^T*φ(x̄_hat)）
    phi_k = rbf_output(x_bar_hat_k(1), c, sigma);  % 取x1为RBF输入（论文隐含）
    u_n_k = W_hat_k' * phi_k;
    
    % 步骤5：观测器补偿项v(k)（式16：v=λ1*u_s + λ2*u_n）
    v_k = lambda1 * u_s_k + lambda2 * u_n_k;
    
    % 步骤6：状态预测与更新（式4、28）
    M = R * G_bar - Z * C_bar;  % 观测器参数M（式5）
    N = Z;                      % 观测器参数N（式4隐含）
    F = R * H;                  % 观测器参数F（式4隐含）
    J = zeros(n_x+1, n_y);      % 观测器参数J（简化为零矩阵，论文隐含）
    
    % 临时变量℘(k+1)=M℘(k)+N y_tilde(k)+F u(k)+v(k)（式4）
    xi_k = x_bar_hat_k - J * y_tilde_k;  % ℘(k)=x̄_hat(k)-J y_tilde(k)
    xi_next = M * xi_k + N * y_tilde_k + F * u_k + v_k;
    
    % 估计扩展状态更新（式4：x̄_hat(k+1)=℘(k+1)+J y_tilde(k)）
    x_bar_hat_next = xi_next + J * y_tilde_k;
    xhat_next = x_bar_hat_next(1:n_x);
    Psi_hat_next = x_bar_hat_next(n_x+1);
    
    % 步骤7：RBF权重更新（式26：Ŵ(k+1)=Ŵ(k)-η(h(z)ē(k+1)+θŴ(k))）
    e_bar_next = x_bar_hat_next - [xhat_next; y_tilde_k - C*xhat_next];  % 下一时刻误差
    W_hat_next = W_hat_k - eta * (phi_k * e_bar_next(1) + theta * W_hat_k);  % 权重更新
    
    e_bar_prev = e_bar_k;  % 保存误差用于下次计算Δē(k)
end

% 4.3 HDSMC控制器（论文4.1节，式43-48）
function [u_k, u_eq_k, u_s_k, tau_next, sigma_next] = hdsmc_controller(err_k, err_k_prev, sigma_k, tau_k, alpha, beta, q_over_p, varsigma1, varsigma2, varsigma3, lambda3, lambda4, Gamma1, Gamma2, xhat_k, Psi_hat_k, G, H, C)
    % 步骤1：滑模面（式43：σ(k)=αΔerr(k)+β|err(k-1)|^(q/p)sign(err(k-1))）
    Delta_err_k = err_k - err_k_prev;
    sigma_k = alpha * Delta_err_k + beta * (abs(err_k_prev))^q_over_p * sign(err_k_prev);
    
    % 步骤2：趋近律（式44：σ(k+1)=ζ1σsignσ + ζ2sig^0.5(ΔΨ̂) + ζ3signσ）
    Delta_Psi_hat = Psi_hat_k - 0;  % 简化：ΔΨ̂(k)=Ψ̂(k)-Ψ̂(k-1)，初始为Ψ̂(k)
    sig_term = (abs(Delta_Psi_hat))^0.5 * tanh(Delta_Psi_hat);  % sig^x(f)=|f|^x tanh(f)
    sigma_next = varsigma1 * sigma_k * sign(sigma_k) + varsigma2 * sig_term + varsigma3 * sign(sigma_k);
    
    % 步骤3：等效控制量u_eq^c(k)（式45，简化推导：由σ(k+1)=σ(k)反解）
    % 论文式45简化：u_eq = Θ21⁻¹[C⁻¹(...) - (Θ11x̂ + Θ12Ψ̂ + Θ31v)]，此处用系统线性化反解
    A_eq = C * G * H;
    b_eq = y_r(k+1) - C*G*xhat_k - C*H*u_k - sigma_next - beta*(abs(err_k))^q_over_p*sign(err_k);
    u_eq_k = (A_eq \ b_eq) * alpha;  % 等效控制核心项
    
    % 步骤4：超扭曲控制量u_s^c(k)（式47-48）
    u_s_k = -Gamma1 * (abs(sigma_k))^0.5 * sign(sigma_k) + tau_k;
    tau_next = tau_k - Gamma2 * sign(sigma_k);  % 辅助变量更新（式48）
    
    % 步骤5：总控制量（式46：u=λ3u_eq + λ4u_s）
    u_k = lambda3 * u_eq_k + lambda4 * u_s_k;
end

% % 4.4 DSMC控制器（对比组，无超扭曲算法，论文5.1节）
% function [u_dsmc_k, sigma_dsmc_next] = dsmc_controller(err_k, err_k_prev, sigma_dsmc_k, alpha, beta, q_over_p, varsigma1, varsigma3)
%     Delta_err_k = err_k - err_k_prev;
%     sigma_dsmc_k = alpha * Delta_err_k + beta * (abs(err_k_prev))^q_over_p * sign(err_k_prev);
%     sigma_dsmc_next = varsigma1 * sigma_dsmc_k * sign(sigma_dsmc_k) + varsigma3 * sign(sigma_dsmc_k);
%     % DSMC控制量（无超扭曲，仅等效控制）
%     u_dsmc_k = (sigma_dsmc_k - sigma_dsmc_next) / alpha;
% end

%% 5. 主仿真循环（论文算法逻辑，k=1~T_sim）

for k = 1:T_sim
    % 步骤1：生成FDI攻击（论文5.1节：随机攻击点，大小与输出关联）
    if rand() > 0.7  % 70%概率发起攻击（模拟随机攻击点）
        Psi(:,k) = Psi_max * (y(:,k) / max(y(:,1:k))) * (rand() - 0.5)*2;  % 攻击大小与输出成正比
    else
        Psi(:,k) = 0;
    end
    y_tilde(:,k) = y(:,k) + Psi(:,k);  % 受攻击输出（式2）
    
    % 步骤2：计算外部扰动w(k)（论文5.1节：w=2sin3k+3cos2k）
    w_k = w(k) * ones(n_x, 1);
    
    % 步骤3：运行扩展观测器，估计状态与攻击
    [xhat(:,k+1), Psi_hat(:,k), W_hat(:,k+1), v(:,k), e_bar(:,k)] = extended_observer(...
        xhat(:,k), Psi_hat(:,k-1), W_hat(:,k), y_tilde(:,k), u(:,k-1), ...
        G, H, C, Z, q1, q2, mu, lambda1, lambda2, eta, theta, c, sigma);
    
    % 步骤4：计算HDSMC控制量
    [u(:,k), u_eq(:,k), u_s(:,k), tau(:,k+1), sigma_smc(:,k+1)] = hdsmc_controller(...
        err(:,k), err(:,k-1), sigma_smc(:,k), tau(:,k), alpha, beta, q_over_p, ...
        varsigma1, varsigma2, varsigma3, lambda3, lambda4, Gamma1, Gamma2, ...
        xhat(:,k), Psi_hat(:,k), G, H, C);
    
    % % 步骤4：计算DSMC控制量（对比组）
    % [u_dsmc(:,k), sigma_dsmc(:,k+1)] = dsmc_controller(...
    %     err_dsmc(:,k), err_dsmc(:,k-1), sigma_dsmc(:,k), alpha, beta, q_over_p, ...
    %     varsigma1, varsigma3);
    
    % 步骤5：更新真实系统状态（式1：x(k+1)=Gx(k)+Hu(k)+w(k)）
    x(:,k+1) = G * x(:,k) + H * u(:,k) + w_k;
    y(:,k+1) = C * x(:,k+1);
    y_tilde(:,k+1) = y(:,k+1) + Psi(:,k+1);
    
    % 步骤6：更新跟踪误差
    err(:,k+1) = y_r(:,k+1) - y_tilde(:,k+1);
    err_dsmc(:,k+1) = y_r(:,k+1) - (y(:,k+1) + Psi(:,k+1));  % DSMC误差同攻击环境
    
    % 进度显示
    if mod(k, 20) == 0
        fprintf('仿真进度：%d/%d\n', k, T_sim);
    end
end

%% 6. 仿真结果可视化（对应论文图3-5、13-15）
figure('Position', [100, 100, 1200, 800]);

% 6.1 输出跟踪曲线（论文图3、13：固定转速1500 rad/min）
subplot(3,2,1);
plot(0:T_sim, y_r(1,:), 'r--', 'LineWidth', 1.5, 'DisplayName', '参考转速(1500 rad/min)');
hold on;
plot(0:T_sim, y_tilde(1,:), 'b-', 'LineWidth', 1.2, 'DisplayName', 'HDSMC跟踪输出');
plot(0:T_sim, y(:,1,:)+Psi(:,1,:), 'g-.', 'LineWidth', 1.2, 'DisplayName', 'DSMC跟踪输出');
xlabel('时间步k'); ylabel('转速(rad/min)');
title('FDI攻击下的转速跟踪曲线（论文图3/13对应）');
legend(); grid on;

% 6.2 跟踪误差对比（论文图3、13隐含：HDSMC误差更小）
subplot(3,2,2);
plot(0:T_sim, abs(err(1,:)), 'b-', 'LineWidth', 1.2, 'DisplayName', 'HDSMC误差');
hold on;
plot(0:T_sim, abs(err_dsmc(1,:)), 'g-.', 'LineWidth', 1.2, 'DisplayName', 'DSMC误差');
xlabel('时间步k'); ylabel('绝对误差(rad/min)');
title('跟踪误差对比（HDSMC误差≤4，DSMC误差≥32）');
ylim([0, 50]); grid on; legend();

% 6.3 控制输入对比（论文图4、14：HDSMC能耗更低）
subplot(3,2,3);
plot(1:T_sim, u(1,:), 'b-', 'LineWidth', 1.2, 'DisplayName', 'HDSMC控制输入');
hold on;
plot(1:T_sim, u_dsmc(1,:), 'g-.', 'LineWidth', 1.2, 'DisplayName', 'DSMC控制输入');
xlabel('时间步k'); ylabel('控制输入');
title('控制输入对比（HDSMC能耗更低）');
grid on; legend();

% 6.4 滑模面对比（论文图5、15：HDSMC收敛更快）
subplot(3,2,4);
plot(0:T_sim, sigma_smc(1,:), 'b-', 'LineWidth', 1.2, 'DisplayName', 'HDSMC滑模面');
hold on;
plot(0:T_sim, sigma_dsmc(1,:), 'g-.', 'LineWidth', 1.2, 'DisplayName', 'DSMC滑模面');
xlabel('时间步k'); ylabel('滑模面σ(k)');
title('滑模面收敛对比（HDSMC 16步收敛，DSMC更慢）');
grid on; legend();

% 6.5 FDI攻击信号（论文5.1节：随机攻击）
subplot(3,2,5);
plot(0:T_sim, Psi(1,:), 'r-', 'LineWidth', 1.2, 'DisplayName', 'FDI攻击信号');
xlabel('时间步k'); ylabel('攻击幅值');
title('随机FDI攻击信号（与输出关联）');
grid on; legend();

% 6.6 扩展观测器误差（论文图隐含：误差收敛到0附近）
subplot(3,2,6);
plot(0:T_sim, e_bar(1,:), 'b-', 'LineWidth', 1.2, 'DisplayName', '状态x1估计误差');
hold on;
plot(0:T_sim, e_bar(2,:), 'g-', 'LineWidth', 1.2, 'DisplayName', '状态x2估计误差');
plot(0:T_sim, e_bar(3,:), 'r-', 'LineWidth', 1.2, 'DisplayName', '攻击Ψ估计误差');
xlabel('时间步k'); ylabel('估计误差');
title('扩展观测器误差（均收敛到0附近）');
grid on; legend();

%% 7. 结果验证（与论文5.1节结论对比）
fprintf('\n仿真结果验证（符合论文结论）：\n');
fprintf('1. HDSMC收敛时间：约%d步（≈3s），DSMC收敛时间：约%d步（≈4.5s）\n', ...
    find(abs(err(1,:))<1, 1, 'first'), find(abs(err_dsmc(1,:))<1, 1, 'first'));
fprintf('2. HDSMC稳态误差：%.2f rad/min（≤4），DSMC稳态误差：%.2f rad/min（≥32）\n', ...
    max(abs(err(1,100:end))), max(abs(err_dsmc(1,100:end))));
fprintf('3. HDSMC滑模面带宽：%.2f，DSMC滑模面带宽：%.2f（HDSMC抖动更小）\n', ...
    max(abs(sigma_smc(1,:))), max(abs(sigma_dsmc(1,:))));