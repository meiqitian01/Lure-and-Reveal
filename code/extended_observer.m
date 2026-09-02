% 4.2 扩展观测器（论文3.1-3.2节，式4、16、23、26）

% %------------------ 
% % HDMSC config
% %-------------------
% k =1;  %第一次HD flag
% % 观测器相关
% n_rbf =12;
% xhat_HD = zeros(6, 200);    % 估计状态x̂(k)
% Psi_hat = zeros(n_y, 200); % 估计FDI攻击Ψ̂(k)
% e_bar = zeros(n_x+1, 200); % 扩展观测误差ē(k)=[x̂-x; Ψ̂-Ψ]（式12）
% W_hat = zeros(n_rbf, 200); % RBF权重估计值
% v = zeros(6, 200);         % 观测器补偿项v(k)（式16）
% 
% % 控制器相关
% err = zeros(n_y, 200);     % 跟踪误差err(k)=y^r(k)-y_tilde(k)（式42）
% sigma_smc = zeros(1, 200); % 滑模面σ(k)（式43）
% u_eq = zeros(n_u, 200);      % 等效控制量u_eq^c(k)（式45）
% u_s = zeros(n_u, 200);       % 超扭曲控制量u_s^c(k)（式47）
% u_HD = zeros(n_u, 200);         % 总控制量u(k)（式46）
% tau = zeros(1, 200);       % 超扭曲辅助变量τ(k)（式48）

%需要写在main中的初始化
%想用论文的结果hdsmc_controller，extended_observer中的Z找不到。
        % if k ==1
        % 
        %     % 初始化
        %     x_HD(:,1) = x;
        %     xhat_HD(:,1) = x_hat_d';
        %     Psi(:,1) = 0;  
        %     W_hat(:,1) =  0.1 * randn(n_rbf, 1); 
        %     tau(:,1) = 0;
        %     err(:,1) = x_bar_list(:,epoch) - x_HD; %跟踪误差（式42)
        % 
        %     [xhat_HD(:,k+1), Psi_hat(:,k+1), W_hat(:,k+1), v(:,k), e_bar(:,k+1)] = extended_observer(k,...
        %         xhat_HD(:,k), Psi_hat(:,k), W_hat(:,k), x_hat, u_HD(:,k),C, D);
        % 
        %     % 步骤4：计算HDSMC控制量
        %     [u_HD(:,k), u_eq(:,k), u_s(:,k), tau(:,k+1), sigma_smc(:,k+1)] = hdsmc_controller(...
        %         err(:,k), err(:,k-1), sigma_smc(:,k), tau(:,k), G, H);
        % 
        %     k = k+1;
        % 
        % else
        %     %y_tilde = x_hat; %当前的测量值（就是系统的估计状态
        %         [xhat_HD(:,k+1), Psi_hat(:,k), W_hat(:,k+1), v(:,k), e_bar(:,k)] = extended_observer(...
        %         xhat_HD(:,k), Psi_hat(:,k-1), W_hat(:,k), x_hat, u_HD(:,k-1),C, D);
        % 
        %     % 步骤4：计算HDSMC控制量
        %     [u_HD(:,k), u_eq(:,k), u_s(:,k), tau(:,k+1), sigma_smc(:,k+1)] = hdsmc_controller(...
        %         err(:,k), err(:,k-1), sigma_smc(:,k), tau(:,k), G, H);
        % 
        % % 步骤5：更新真实系统状态（式1：x(k+1)=Gx(k)+Hu(k)+w(k)）
        % x = C*x + D*u_HD + w_k;
        % x(6) = -x(6);
        % 
        % % 步骤6：更新跟踪误差
        % err(:,k) = x_bar_list(:,epoch) - x_hat;
        % 
        % k=k+1;
        % 
        % end


%找不到合适的Z
function [xhat_next, Psi_hat_next, W_hat_next, v_k, e_bar_next] = extended_observer(k, xhat_k, ...
    Psi_hat_k, W_hat_k, y_tilde_k, u_k, G, H)

%input:
%   xhat_k: 估计状态
%   Psi_hat_k：估计FDI攻击Ψ̂(k)
%   W_hat_k：  RBF权重估计值
%   y_tilde_k： 受攻击输出（式2）
%   u_k：HD的控制输入
%   G、H C:  % 系统矩阵G 控制矩阵H 测量矩阵C（论文中为列向量，此处转置为行向量适配计算）

C = eye(6);
% 2.5 RBF神经网络参数（论文3.2节，式17-18）
n_rbf = 12;      % 隐层节点数（论文未明说，取10满足逼近精度）
c = linspace(-5,5,n_rbf);  % 高斯函数中心（局部训练法，覆盖状态范围）
sigma = 1.5;     % 高斯函数宽度（论文隐含设定，确保局部逼近）
eta = 0.01;      % 权重更新步长η（式26）
theta = 0.001;   % 权重正则化参数θ（式26）

% 2.3 扩展观测器参数（论文5.1节）
Z = [0.07; -0.1042; 0.3736];  % 观测器增益Z（式5：M=R̄G-Z̄C）
q1 = 0.03;  % 滑模函数参数q1（式14）
q2 = 0.22;  % 滑模函数参数q2（式14）
mu = 0.06;  % 滑模控制参数μ（式15）
lambda1 = 0.9;  % 观测器补偿项权重λ1（式16）
lambda2 = 0.1;  % 观测器补偿项权重λ2（式16），满足λ1+λ2=1


    n_x = size(G,1);
    % 步骤1：构造扩展状态x̄(k)=[x(k); Ψ(k)]，估计x̄_hat(k)=[xhat(k); Psi_hat(k)]
    x_bar_hat_k = [xhat_k; Psi_hat_k];
    R = [eye(n_x); -C];  % 论文式6中R矩阵（n_x+1行，n_x列）12*6
    G_bar = [G, zeros(6,6)];  % 扩展系统矩阵Ḡ [G, 0_{n*p}]。6*12
    C_bar = [C; eye(n_x)];      % 扩展测量矩阵C̄（式3）6*12
    
    % 步骤2：滑模函数（式14：s(k)=q1*ē(k)+q2*Δē(k)）
    e_bar_k = x_bar_hat_k - [xhat_k; y_tilde_k - C*xhat_k];  % 当前扩展误差。12行的列向量
    if k == 1
        Delta_e_bar = 0;  % 初始误差差分为0
    else
        Delta_e_bar = e_bar_k - e_bar_prev;  %修改
    end
    s_k = q1 * e_bar_k + q2 * Delta_e_bar;  %12行的列向量
    
    % 步骤3：滑模补偿项u_s(k)（式15：u_s=μ*sign(s)*|s|^0.5）
    u_s_k = mu * sign(s_k) * sqrt(norm(s_k));
    
    % 步骤4：RBF补偿项u_n(k)（式22：u_n=Ŵ^T*φ(x̄_hat)）
    phi_k = rbf_output(x_bar_hat_k, c, sigma);  % 取x1为RBF输入（论文隐含）
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

%% 4. 核心函数定义（均对应论文公式）
% 4.1 RBF神经网络输出（式17-18：φ_i(x)=exp(-(x-c_i)^2/σ²)）
function phi = rbf_output(x, c, sigma)
    n_rbf = length(c);
    phi = zeros(n_rbf, 1);
    for i = 1:n_rbf
        phi(i) = exp(-(x(i) - c(i))^2 / (sigma^2));  % 高斯激活函数
    end
end
