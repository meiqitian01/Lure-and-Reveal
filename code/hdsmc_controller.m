% 4.3 HDSMC控制器（论文4.1节，式43-48）
function [u_k, u_eq_k, u_s_k, tau_next, sigma_next] = hdsmc_controller(err_k, err_k_prev, sigma_k, tau_k, xhat_k, Psi_hat_k, G, H)
    

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
C = eye(6);

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