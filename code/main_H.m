clear all

%系统参数设置
n_x = 6;      % 状态维度
n_u = 3;      % 控制输入维度
n_y = 6;      % 测量输出维度
a = 5;        % 攻击周期（每a步发生一次攻击）
disturbance_bound = 0.01;

dt = 0.5;          % 时间步长 (秒)

C = [1 0 0 dt 0 0;
    0 1 0 0 dt 0;
    0 0 1 0 0 dt;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

D = [0.5*dt^2 0 0;
    0 0.5*dt^2 0;
    0 0 0.5*dt^2;
    dt 0 0;
    0 dt 0;
    0 0 dt];

Kc=[-0.1080         0         0    -0.6330         0         0;
         0    -0.0952         0         0    -0.5962         0;
         0         0    -0.0128         0         0    -0.2368];


%-------------------
% constants
%-------------------
R_earth = 6378137;      % 地球半径 (m)
deg_to_rad = 0.01745329252;  %角度转弧度
rad_to_deg = 1/deg_to_rad;
micro_g_to_meters_per_second_squared = 9.80665E-6;
initialization_errors.delta_eul_nb_n = [-0.01;0.008;0.01]*deg_to_rad; % rad

%-------------------
% IMU_config
%-------------------
% Accelerometer biases (micro-g, converted to m/s^2; body axes)
IMU_errors.b_a = [30;-45;26] * micro_g_to_meters_per_second_squared;
% Gyro biases (deg/hour, converted to rad/sec; body axes)
IMU_errors.b_g = [-0.0009;0.0013;-0.0008] * deg_to_rad / 3600;
% Accelerometer scale factor and cross coupling errors (ppm, converted to
% unitless; body axes)
IMU_errors.M_a = [100, -120,  80;...
                  -60, -120, 100;...
                 -100,   40,  90] * 1E-6;
% Gyro scale factor and cross coupling errors (ppm, converted to unitless;
% body axes)
IMU_errors.M_g = [8, -120, 100;...
                  0,   -6, -60;...
                  0,    0,  -7] * 1E-6;             
% Gyro g-dependent biases (deg/hour/g, converted to rad-sec/m; body axes)
IMU_errors.G_g = [0, 0, 0;...
                  0, 0, 0;...
                  0, 0, 0] * deg_to_rad / (3600 * 9.80665);             
% Accelerometer noise root PSD (micro-g per root Hz, converted to m s^-1.5)                
IMU_errors.accel_noise_root_PSD = 20 *...
    micro_g_to_meters_per_second_squared;
% Gyro noise root PSD (deg per root hour, converted to rad s^-0.5)                
IMU_errors.gyro_noise_root_PSD = 0.002 * deg_to_rad / 60;
% Accelerometer quantization level (m/s^2)
IMU_errors.accel_quant_level = 5E-5;
% Gyro quantization level (rad/s)
IMU_errors.gyro_quant_level = 1E-6;

%-------------------
% GNSS_config
%-------------------
% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 0.5;

% Initial estimated position (m; ECEF)
GNSS_config.init_est_r_ea_e = [0;0;0];

% Number of satellites in constellation
GNSS_config.no_sat = 30;
% Orbital radius of satellites (m)
GNSS_config.r_os = 2.656175E7;
% Inclination angle of satellites (deg)
GNSS_config.inclination = 55;
% Longitude offset of constellation (deg)
GNSS_config.const_delta_lambda = 0;
% Timing offset of constellation (s)
GNSS_config.const_delta_t = 0;

% Mask angle (deg)
GNSS_config.mask_angle = 10;
% Signal in space error SD (m) *Give residual where corrections are applied
GNSS_config.SIS_err_SD = 1;
% Zenith ionosphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_iono_err_SD = 2;
% Zenith troposphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_trop_err_SD = 0.2;
% Code tracking error SD (m) *Can extend to account for multipath
GNSS_config.code_track_err_SD = 1;
% Range rate tracking error SD (m/s) *Can extend to account for multipath
GNSS_config.rate_track_err_SD = 0.02;
% Receiver clock offset at time=0 (m);
GNSS_config.rx_clock_offset = 10000;
% Receiver clock drift at time=0 (m/s);
GNSS_config.rx_clock_drift = 100;

%-------------------
% TC_KF_config
%-------------------
% Initial attitude uncertainty per axis (deg, converted to rad)
TC_KF_config.init_att_unc = deg2rad(1);
%TC_KF_config.init_att_unc = degtorad(1);
% Initial velocity uncertainty per axis (m/s)
TC_KF_config.init_vel_unc = 0.1;
% Initial position uncertainty per axis (m)
TC_KF_config.init_pos_unc = 10;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted
% to m/s^2)
TC_KF_config.init_b_a_unc = 1000 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
TC_KF_config.init_b_g_unc = 10 * deg_to_rad / 3600;
% Initial clock offset uncertainty per axis (m)
TC_KF_config.init_clock_offset_unc = 10;
% Initial clock drift uncertainty per axis (m/s)
TC_KF_config.init_clock_drift_unc = 0.1;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)
TC_KF_config.gyro_noise_PSD = (0.02 * deg_to_rad / 60)^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)
TC_KF_config.accel_noise_PSD = (200 *...
    micro_g_to_meters_per_second_squared)^2;
% Accelerometer bias random walk PSD (m^2 s^-5) 1.0E-7;
TC_KF_config.accel_bias_PSD = 1.0E-5;
% Gyro bias random walk PSD (rad^2 s^-3)2.0E-12;
TC_KF_config.gyro_bias_PSD = 4.0E-11;
% Receiver clock frequency-drift PSD (m^2/s^3)
TC_KF_config.clock_freq_PSD = 1;
% Receiver clock phase-drift PSD (m^2/s)
TC_KF_config.clock_phase_PSD = 1;

% Pseudo-range measurement noise SD (m)
TC_KF_config.pseudo_range_SD = 2.5;
% Pseudo-range rate measurement noise SD (m/s)
TC_KF_config.range_rate_SD = 0.1;

%-------------------
% GNSS_KF_config
%-------------------
% Initial position uncertainty per axis (m)
GNSS_KF_config.init_pos_unc = 10;
% Initial velocity uncertainty per axis (m/s)
GNSS_KF_config.init_vel_unc = 0.1;
% Initial clock offset uncertainty per axis (m)
GNSS_KF_config.init_clock_offset_unc = 10;
% Initial clock drift uncertainty per axis (m/s)
GNSS_KF_config.init_clock_drift_unc = 0.1;

% Acceleration PSD per axis (m^2/s^3)
GNSS_KF_config.accel_PSD = 10;
% Receiver clock frequency-drift PSD (m^2/s^3)
GNSS_KF_config.clock_freq_PSD = 1;
% Receiver clock phase-drift PSD (m^2/s)
GNSS_KF_config.clock_phase_PSD = 1;

% Pseudo-range measurement noise SD (m)
GNSS_KF_config.pseudo_range_SD = 2.5;
% Pseudo-range rate measurement noise SD (m/s)
GNSS_KF_config.range_rate_SD = 0.5;


% Seeding of the random number generator for reproducability. Change
% this value for a different random number sequence (may not work in Octave).
RandStream.setGlobalStream(RandStream('mt19937ar','seed', 1));

%-------------------
% detector_config
%-------------------
Detector.alpha = 0.05;
Detector.time_window= 1;
no_GNSS_meas=8;
Detector.threshold = chi2inv(1 - Detector.alpha, Detector.time_window * no_GNSS_meas *2);
% 攻击检测阈值 a v r imu_a_bias, imu_g_bias, tu,fu
Ti = 0.05;
Detector.T_ECEF = [0;0;0;
    Ti;Ti;Ti;
    Ti;Ti;Ti;
    0.0441;0.0637;0.0392;
    1.0e-03 *0.2182;1.0e-03 * 0.3151;1.0e-03 * 0.1939;
    20000;
    101;
    ];

%------------------ 
% H_inf config
%-------------------
% 1. Weighting matrices
Wp = diag([1 1 1 1 1 1]);   % penalize position & velocity
Wu = 0.1*eye(n_u);             % penalize control effort

% 2. Generalized plant P setup
B1 = eye(n_x);                 % disturbance input
B2 = D;

% z1 = Wp*x
C1 = [Wp; zeros(n_u, n_x)];
D11 = [zeros(n_x); zeros(n_u, n_x)];   % remove direct w->z
D12 = [zeros(n_x,n_u); Wu];            % control enters z2 only

% Measurement output y = x
C2 = eye(n_x);
D21 = zeros(n_x, n_x);
D22 = zeros(n_x, n_u);

% 4. Hinfsyn design
[K, CL, gamma] = hinfsyn(ss(C, [B1 B2], [C1; C2], [D11 D12; D21 D22], dt), n_x, n_u);

[K_a, CL_a, gamma_a] = hinfsyn(ss(C, [B1 B2], [C1; C2], [D11 D12; D21 D22], dt), n_x, n_u);

disp(['hinfsyn returned gamma = ', num2str(gamma)]);


% --- Assumptions: you already ran

% rename to standard symbols
A = C;      % my 'C' in code is the plant A
B = D;      % my 'D' in code is the plant B

% check controller K is in workspace and is an ss object or LTI
if ~exist('K','var')
    error('Controller K not found. Run hinfsyn first and ensure K exists.');
end
Kss = ss(K);  % ensure it's an ss model

% extract controller matrices (controller has states n_k)
Ak = Kss.A;
Bk = Kss.B;
Ck = Kss.C;
Dk = Kss.D;

% Dimensions check (optional but recommended)
n_x = size(A,1);
n_ak = size(Ak,1);
if size(Bk,2) ~= n_x
    warning('Controller Bk expected to have #cols == n_x (measurement y = x). Check dimensions.');
end
if size(Ck,1) ~= size(B,2)
    warning('Controller Ck output dimension should equal plant input dimension n_u. Check sizes.');
end

% Construct augmented closed-loop A_cl for the interconnection (no disturbances)
A_cl = [ A + B*Dk,   B*Ck;
         Bk,         Ak    ];

% sanity check
fprintf('A_cl size: %d x %d\n', size(A_cl,1), size(A_cl,2));

% 1) spectral radius r
eigA = eig(A_cl);
r = max(abs(eigA));
fprintf('spectral radius r = %g\n', r);
if r >= 1
    warning('Closed-loop (augmented) is not asymptotically stable (r >= 1).');
end

% 2) choose rho in (r,1)
rho = (r + 1)/2;  % simple midpoint choice; you may pick rho closer to r if desired
fprintf('chosen rho = %g\n', rho);

% 3) solve discrete Lyapunov: A_cl' * P * A_cl - rho^2 * P = -I  (choose Q = I)
ncl = size(A_cl,1);
Q = eye(ncl);
% Solve: A_cl' * P * A_cl - rho^2 * P = -Q  -> use dlyap on A_cl' with factor rho^2
% Note: dlyap requires Control System Toolbox
try
    P = dlyap(A_cl', rho^2 * eye(ncl)); % returns P such that P - A_cl' * P * A_cl = rho^2*... different forms exist
catch ME
    error('dlyap failed: %s\nEnsure Control System Toolbox is installed or choose different rho.', ME.message);
end

% check P positive definite
eigP = eig(P);
lam_min = min(real(eigP));
lam_max = max(real(eigP));
if lam_min <= 0
    error('P is not positive definite. Try choosing rho closer to 1 (but > r).');
end

% 4) compute M
M = sqrt(lam_max/lam_min);
fprintf('Lyapunov P found. lambda_min=%g, lambda_max=%g, M=%g\n', lam_min, lam_max, M);

% 5) compute ||B|| (2-norm) for d_max = ||B||*barU + w_max
Bnorm = norm(B,2);
fprintf('||B||_2 = %g\n', Bnorm);

% 6) compute allowable bar_U for a desired epsilon (steady-state tolerance)
% set desired epsilon and w_max (if known)
if ~exist('eps','var')
    eps = 0.4;  % example tolerance, set as needed
end
if ~exist('w_max','var')
    w_max = disturbance_bound; % you provided disturbance_bound = 0.01
end

% steady-state bound: limsup ||delta x|| <= (M/(1-rho)) * d_max  where d_max = ||B||*barU + w_max
ISS_prefactor = M / (1 - rho);
dmax_allowed = (1 - rho) * eps / M; % equivalent to d_max <= dmax_allowed
barU_allowed = (dmax_allowed - w_max) / Bnorm; %MPC用的bar u
fprintf('=> Maximum bar_U (<=) = %g  (if positive; else infeasible)\n', max(0, barU_allowed));
u_min = 2*Ti*(1-rho)/(M*(1-rho^10)) + 2*w_max + Ti;
fprintf('=> Minimum bar_U (>=) = %g\n', u_min);

%------------------ 
% MPC config
%-------------------
MPC.k_exp = 10;    %暴露时间
MPC.N = MPC.k_exp+5;
MPC.Q = diag([10,10,10,5,5,5]); % 状态惩罚权重（位置惩罚>速度）
MPC.R = diag([1,1,1]);          % 输入惩罚权重
MPC.u_bar = u_min;
MPC.U_change = [0.5;0.5;0.5];  % 控制量每次变化的上界
u_bound = [5;5;5];

%----------------------------------
%  Initialize dynamics parameters
% and original states
%--------------------------------

p = 500;         %预期前进距离
v_target = 5;   %预期前进速度
latitude_rad_initial = 50 * deg_to_rad;           % 起始纬度 (rad)
longitude_rad_initial = 0 * deg_to_rad;          % 起始经度 (rad)
height= 100;     %预期飞行高度

T = p / v_target;   % 总仿真时间 (秒)
time = 0:dt:T;      % 时间向量

x = [0; 0; height; 0; v_target; 0];
% H控制器初始化
xK = x;  % 控制器状态
xK_a = xK;

%-------desired trajectory------------
v_go = v_target * dt;  % dt=0.5; 500米/速度5 = 100，100/dt0.5 = 200

x_bar_diff= [0;v_go;0;0;0;0];
x_bar_list= [0; 0; height; 0; v_target; 0];

for i = 2:length(time)+10
    x_bar_next = x_bar_list(:, i-1) + x_bar_diff;  % 前一个向量加a得到当前向量
    x_bar_list = [x_bar_list,x_bar_next];
end

x_hat = x;
x_hat_a = x;

%-------------------
% Initialize log storage
%--------------------
x_log = zeros(length(time), 6);
x_hat_log = zeros(length(time), 6);
x_bar_log = zeros(length(time), 6);
motion_profile = zeros(length(time), 10); % [time, latitude, longitude, height, vn, ve, vd, roll, pitch, yaw]
e_log =  zeros(length(time), 6);   % 跟踪误差历史
a_log = zeros(length(time), 4);

%-------------------
% detector related
%--------------------
lamda_GI_result = zeros(T/GNSS_config.epoch_interval, 4);
delta_KF_x_log = zeros(T/GNSS_config.epoch_interval, 18);
delta_a = zeros(6,1);
attack =0;
exposure_flag =0;
detection_value_attacker = 0;
bound_threshold_glb = 0;

%-------------------
% attack: use a fixed direction vector (from the h(\hat x^{n,a}) to z_a)
%--------------------
e_fixed = randn(2*8,1);
e_fixed = e_fixed/norm(e_fixed);

e_fixed(1:4) = -e_fixed(1:4);
e_fixed(9:13) = -e_fixed(9:13);


if isfile('motion_profile.csv')
    delete('motion_profile.csv');
end

if isfile('x_bar_log.csv')
    delete('x_bar_log.csv');
end

if isfile('x_hat_log.csv')
    delete('x_hat_log.csv');
end

if isfile('x_log.csv')
    delete('x_log.csv');
end

if isfile('a_log.csv')
    delete('a_log.csv');
end

if isfile('delta_KF_x_log.csv')
    delete('delta_KF_x_log.csv');
end


% 仿真循环  epoch|time =1|0,2|0.5, 3|1, 4|1.5. time = (epoch-1)*0.5
for epoch = 1:length(time)

    if (epoch == 1)

        motion = [
            time(epoch);              % 时间
            latitude_rad_initial * rad_to_deg;             % 纬度
            longitude_rad_initial* rad_to_deg;            % 经度
            x(3);               % 高度 (m）
            x(4);                 % 北向速度 (m/s)
            x(5);                 % 东向速度 (m/s)
            x(6);                 % 下向速度 (m/s)
            0;                    % 横滚角 (假设为 0)
            0;                    % 俯仰角 (假设为 0)
            90                     % 偏航角 (假设为 0)
            ];

        in_profile = motion;
        in_profile(2:3) = deg_to_rad * in_profile(2:3); % Convert degrees to radians 把角度转换为弧度
        in_profile(8:10) = deg_to_rad * in_profile(8:10);

        % Initialize true navigation solution
        old_time = in_profile(1);
        G_old_time = in_profile(1);
        true_L_b = in_profile(2);% latitude
        true_lambda_b = in_profile(3); % longitude
        true_h_b = in_profile(4);%高度
        true_v_eb_n = in_profile(5:7);%三个方向的速度
        true_eul_nb = in_profile(8:10);

        % Outputs:
        %   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
        %                 along ECEF-frame axes (m)
        %   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
        %                 ECEF-frame axes (m/s)
        %   C_b_e         body-to-ECEF-frame coordinate transformation matrix

        %把欧拉角转换为相应的坐标转换矩阵。欧拉角：从一个参考坐标系（通常是固定的或惯性坐标系）变换到一个物体坐标系（通常是刚体本身的坐标系）的旋转
        true_C_b_n = Euler_to_CTM(true_eul_nb)';

        %从North-East-Down Frame转换到ECEF坐标。
        [old_true_r_eb_e,old_true_v_eb_e,old_true_C_b_e] =...
            NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);

        % 
        % temp_L_b = true_L_b + 1;
        % 
        % [temp_true_r_eb_e,temp_true_v_eb_e,temp_true_C_b_e] =...
        %     NED_to_ECEF(temp_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
        % disp(temp_true_r_eb_e);
        % Determine satellite positions and velocities
        % 卫星的位置和速度 no_sat * 3
        [sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(old_time,...
            GNSS_config);

        %不超过1800s
        GNSS_biases = Initialize_GNSS_biases(sat_r_es_e,old_true_r_eb_e,true_L_b,...
            true_lambda_b,GNSS_config);

        % Generate GNSS measurements：
        %   GNSS_measurements     GNSS measurement data:
        %     Column 1              Pseudo-range measurements (m)
        %     Column 2              Pseudo-range rate measurements (m/s)
        %     Columns 3-5           Satellite ECEF position (m)
        %     Columns 6-8           Satellite ECEF velocity (m/s)
        %   no_GNSS_meas          Number of satellites for which measurements are
        %                         supplied
        [GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(old_time,...
            sat_r_es_e,sat_v_es_e,old_true_r_eb_e,true_L_b,true_lambda_b,...
            old_true_v_eb_e,GNSS_biases,GNSS_config);


        % Determine Least-squares GNSS position solution
        %   est_r_ea_e            estimated ECEF user position (m)
        %   est_v_ea_e            estimated ECEF user velocity (m/s)
        %   est_clock             estimated receiver clock offset (m) and drift (m/s)
        [old_est_r_eb_e,old_est_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
            GNSS_measurements,no_GNSS_meas,GNSS_config.init_est_r_ea_e,[0;0;0]);

        G_est_r_eb_e = old_est_r_eb_e;
        G_est_v_eb_e = old_est_v_eb_e;
        G_est_clock = est_clock;

        [G_x_est,G_P_matrix] = Initialize_GNSS_KF(G_est_r_eb_e,G_est_v_eb_e,G_est_clock,GNSS_KF_config);

        G_est_C_b_n = true_C_b_n; % This sets the attitude errors to zero

        [G_est_L_b,G_est_lambda_b,G_est_h_b,G_est_v_eb_n] = pv_ECEF_to_NED(G_x_est(1:3),G_x_est(4:6));


        [old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] =pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);

        est_L_b = old_est_L_b;

        % Initialize estimated attitude solution
        %   est_C_b_n     body-to-NED coordinate transformation matrix solution
        old_est_C_b_n = Initialize_NED_attitude(true_C_b_n,initialization_errors);

        [temp1,temp2,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,...
            old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);

 
        % Determine errors and generate output record
        [G_delta_r_eb_n,G_delta_v_eb_n,G_delta_eul_nb_n] = Calculate_errors_NED(...
            G_est_L_b,G_est_lambda_b,G_est_h_b,G_est_v_eb_n,G_est_C_b_n,true_L_b,...
            true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);


        % Determine errors and generate output record
        [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
            old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n,...
            true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
       
        % Initialize Kalman filter P matrix and IMU bias states
        P_matrix = Initialize_TC_P_matrix(TC_KF_config);
        est_IMU_bias = zeros(6,1);

        % Initialize IMU quantization residuals
        quant_residuals = [0;0;0;0;0;0];

        % Generate IMU bias and clock output records
        out_IMU_bias_est(1,1) = old_time;
        out_IMU_bias_est(1,2:7) = est_IMU_bias';
        out_clock(1,1) = old_time;
        out_clock(1,2:3) = est_clock;

        G_out_clock(1,1) = G_old_time;
        G_out_clock(1,2:3) = G_x_est(7:8);

        % Generate KF uncertainty record
        G_out_KF_SD(1,1) = G_old_time;
        for i =1:8
            G_out_KF_SD(1,i+1) = sqrt(G_P_matrix(i,i));
        end

        % Generate KF uncertainty record
        out_KF_SD(1,1) = old_time;
        for i =1:17
            out_KF_SD(1,i+1) = sqrt(P_matrix(i,i));
        end % for i

        % Initialize GNSS model timing
        time_last_GNSS = old_time;
        G_time_last_GNSS = G_old_time;
        GNSS_epoch = 1;
        G_GNSS_epoch =1;
        % 更新估计状态（这里假设简单的估计等于实际状态）
        x_hat = x;

    else
        if epoch>=62  %30s-100s attack duration
            ka = 62;
            attack = 1;
        else
            attack = 0;
        end

        disp("epoch:");
        disp(epoch);

        %无攻击
        if attack==0
            [K_matrix_denominator, K_matrix,lamda_GI1,delta_KF_x, out_IMU_bias_est, out_clock, out_profile_1,old_time,...
                quant_residuals,est_IMU_bias, ...
                old_est_r_eb_e, old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
                time_last_GNSS,GNSS_epoch,est_clock, ...
                P_matrix,est_L_b,...
                G_out_clock,G_x_est,G_P_matrix, G_old_time] = State_estimation_normal(epoch, out_IMU_bias_est, out_clock, in_profile, old_time,IMU_errors,GNSS_config,TC_KF_config,...
                quant_residuals, ...
                est_IMU_bias, old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
                time_last_GNSS,GNSS_epoch,est_clock,P_matrix,est_L_b,GNSS_biases, ...
                GNSS_KF_config, G_x_est,G_P_matrix, G_old_time, G_out_clock);


            x_hat(1) = (out_profile_1(2)- latitude_rad_initial) * R_earth;
            x_hat(2) = (out_profile_1(3) - longitude_rad_initial) * R_earth* cos(out_profile_1(2));
            x_hat(3) = out_profile_1(4);
            x_hat(4:6) = out_profile_1(5:7);

        % else %只有攻击，没有shake
        %     [K_matrix_denominator, K_matrix,attack_flag,lamda_GI1,delta_KF_x,out_IMU_bias_est, out_clock, out_profile_1,old_time,...
        %         quant_residuals,est_IMU_bias, ...
        %         old_est_r_eb_e, old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
        %         time_last_GNSS,GNSS_epoch,est_clock, ...
        %         P_matrix,est_L_b,...
        %         G_out_clock,G_x_est,G_P_matrix, G_old_time, detection_value_attacker] = State_estimation_under_spoofing( ...
        %         Detector, delta_a, out_IMU_bias_est, out_clock, in_profile, old_time,IMU_errors,GNSS_config,TC_KF_config,...
        %         quant_residuals, ...
        %         est_IMU_bias, old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
        %         time_last_GNSS,GNSS_epoch,est_clock,P_matrix, est_L_b, GNSS_biases, ...
        %         GNSS_KF_config, G_x_est,G_P_matrix, G_old_time, G_out_clock,e_fixed,K_matrix_denominator, K_matrix);
        % 
        % 
        %      x_hat(1) = (out_profile_1(2)- latitude_rad_initial) * R_earth;
        %      x_hat(2) = (out_profile_1(3) - longitude_rad_initial) * R_earth* cos(out_profile_1(2));
        %      x_hat(3) = out_profile_1(4);
        %      x_hat(4:6) = out_profile_1(5:7);

        %end
        %shake注释开始
         else %有攻击，带shake
            if epoch==62  %第一次攻击，分开维护
                [K_matrix_denominator, K_matrix,lamda_a,out_IMU_bias_est, out_clock, out_profile_a,out_profile_d,old_time,...
                    quant_residuals,est_IMU_bias, ...
                    old_est_r_eb_e, old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
                    time_last_GNSS,GNSS_epoch,est_clock, ...
                    P_matrix,est_L_b,...
                    G_out_clock,G_x_est,G_P_matrix, G_old_time, detection_value_attacker] = State_estimation_att1st( ...
                    Detector, delta_a, out_IMU_bias_est, out_clock, in_profile, old_time,IMU_errors,GNSS_config,TC_KF_config,...
                    quant_residuals, ...
                    est_IMU_bias, old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
                    time_last_GNSS,GNSS_epoch,est_clock,P_matrix, est_L_b, GNSS_biases, ...
                    GNSS_KF_config, G_x_est,G_P_matrix, G_old_time, G_out_clock,e_fixed,K_matrix_denominator,K_matrix);

                %defender估计的状态（IMU值 NED米制
                x_hat_d(1) = (out_profile_d(2)- latitude_rad_initial) * R_earth;
                x_hat_d(2) = (out_profile_d(3) - longitude_rad_initial) * R_earth* cos(out_profile_d(2));
                x_hat_d(3) = out_profile_d(4);
                x_hat_d(4:6) = out_profile_d(5:7);

                %attacker：估算的系统状态 NED米制
                x_hat_a(1) = (out_profile_a(2)- latitude_rad_initial) * R_earth;
                x_hat_a(2) = (out_profile_a(3) - longitude_rad_initial) * R_earth* cos(out_profile_a(2));
                x_hat_a(3) = out_profile_a(4);
                x_hat_a(4:6) = out_profile_a(5:7);
                x_hat_a  =  x_hat_a';

                %attacker和defender维护相同的初始值
                old_est_r_eb_e_d = old_est_r_eb_e;
                old_est_r_eb_e_a = old_est_r_eb_e;

                old_est_v_eb_e_d = old_est_v_eb_e;
                old_est_v_eb_e_a = old_est_v_eb_e;

                old_est_C_b_e_a = old_est_C_b_e;
                old_est_C_b_e_d =old_est_C_b_e;

                %ture
                old_true_r_eb_e_d =  old_true_r_eb_e;
                old_true_v_eb_e_d = old_true_v_eb_e;
                old_true_C_b_e_d = old_true_C_b_e;

                %只有attacker维护
                est_clock_a = est_clock;
                P_matrix_a = P_matrix;

                %只有defender维护
                est_IMU_bias_d = est_IMU_bias;

            %有攻击，epoch>62，还未暴露，分开维护
            elseif exposure_flag==0

                %========exposure begin=======
                %有攻击
                %分开计算defender和attacker的状态，attacker依靠IMU，attacker依靠GNSS以及estimated system state
                %参数多，跟定义比对，顺序应该一致
                [quant_residuals,out_profile_a, out_profile_d, ...
                    old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d,old_est_r_eb_e_a, old_est_v_eb_e_a,...
                    GNSS_epoch,exposure_flag,...
                    old_true_r_eb_e_d, old_true_v_eb_e_d, old_true_C_b_e_d,...
                    old_time,est_IMU_bias_d, est_clock_a,est_L_b] = State_estimation_in_exposure(K_matrix_denominator, K_matrix,Detector, ...
                    GNSS_epoch,GNSS_config,GNSS_biases,...
                    in_profile_a, in_profile, quant_residuals,IMU_errors,...
                    old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d, old_est_r_eb_e_a,old_est_v_eb_e_a,old_est_C_b_e_a, ...
                    old_true_r_eb_e_d, old_true_v_eb_e_d, old_true_C_b_e_d,est_clock_a,old_time,est_IMU_bias_d,e_fixed);

        %shake注释结束
                % %=========check begins==========
                % %用真实GNSS测量值，期望是，短时间内（IMU偏移）不会超过threshold,
                % % 测试结果：epoch=92时，才超过阈值
                % [quant_residuals,out_profile_a, out_profile_d, ...
                %     old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d,old_est_r_eb_e_a, old_est_v_eb_e_a,...
                %     GNSS_epoch,exposure_flag,...
                %     old_true_r_eb_e_d, old_true_v_eb_e_d, old_true_C_b_e_d,...
                %     old_time,est_IMU_bias_d, est_clock_a] = State_estimation_in_exposure_check(K_matrix_denominator, K_matrix,Detector, ...
                %     GNSS_epoch,GNSS_config,GNSS_biases,...
                %     in_profile_a, in_profile, quant_residuals,IMU_errors,...
                %     old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d, old_est_r_eb_e_a,old_est_v_eb_e_a,old_est_C_b_e_a, ...
                %     old_true_r_eb_e_d, old_true_v_eb_e_d, old_true_C_b_e_d,est_clock_a,old_time,est_IMU_bias_d,e_fixed);
                % %=========check end==========
                % shake注释开始
                %defender估计的状态（IMU值）
                x_hat_d(1) = (out_profile_d(2)- latitude_rad_initial) * R_earth;
                x_hat_d(2) = (out_profile_d(3) - longitude_rad_initial) * R_earth* cos(out_profile_d(2));
                x_hat_d(3) = out_profile_d(4);
                x_hat_d(4:6) = out_profile_d(5:7);

                %attacker：估算的系统状态 NED米制
                x_hat_a(1) = (out_profile_a(2)- latitude_rad_initial) * R_earth;
                x_hat_a(2) = (out_profile_a(3) - longitude_rad_initial) * R_earth* cos(out_profile_a(2));
                x_hat_a(3) = out_profile_a(4);
                x_hat_a(4:6) = out_profile_a(5:7);

                 %==========exposure end==========
                %有攻击，epoch>=62，已经暴露 启动中
            elseif exposure_flag==1

                disp(['attack has been exposed at epoch =', num2str(epoch-1)]);

                %只用IMU
                [out_profile_1,old_time,quant_residuals,est_IMU_bias, ...
                    old_est_r_eb_e_d, old_est_v_eb_e_d,old_est_C_b_e_d,old_true_C_b_e_d,old_true_v_eb_e_d,...
                    old_true_r_eb_e_d] = State_estimation_only_INS(in_profile, quant_residuals, ...
                    est_IMU_bias, old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d,old_true_C_b_e_d, ...
                    old_true_v_eb_e_d,old_true_r_eb_e_d,IMU_errors);

               

                x_hat(1) = (out_profile_1(2)- latitude_rad_initial) * R_earth;
                x_hat(2) = (out_profile_1(3) - longitude_rad_initial) * R_earth* cos(out_profile_1(2));
                x_hat(3) = out_profile_1(4);
                x_hat(4:6) = out_profile_1(5:7);

            end

        end %shake注释结束
       
        %------chi-square detector --------
        if lamda_GI1 ~=0
            % lamda_GI_result(GNSS_epoch,1)  = epoch;
            % lamda_GI_result(GNSS_epoch,2) =  lamda_GI1;
            lamda_GI_result(epoch,1) = time(epoch);
            lamda_GI_result(epoch,2) =  lamda_GI1;
        end
        if delta_KF_x ~=0
            delta_KF_x_log(GNSS_epoch,1)  = time_last_GNSS;
            delta_KF_x_log(GNSS_epoch,2:18) =  delta_KF_x';
        end
        %------chi-square detector end--------
    end

    %=======测量结束============

    %---------controller ----------%
    w_k = randn(n_x, 1) * sqrt(disturbance_bound/n_x);
    w_k_a = randn(n_x, 1) * sqrt(disturbance_bound/n_x);
    %无攻击以及只攻击不shake，都用一般控制器 begins
    

    if attack ==0

        y = x_hat - x_bar_list(:,epoch);
        % 控制器更新
        a = K.C * xK + K.D * y;
        xK = K.A * xK + K.B * y;
        % 系统更新
        x = C*x + D*a + w_k;
        x(6) = -x(6);
        %attack控制器更新
        a_a = K_a.C * xK_a + K_a.D * y;
        xK_a = K_a.A * xK_a + K_a.B * y;
    

    elseif epoch>=62 && exposure_flag==0 %有攻击且攻击未被exposure时，计算shake

        disp(['Exposure stage, epoch=',num2str(epoch)]);
       
        if(size(x_hat_a,1) ==1)
            x_hat_a = x_hat_a';
        end

        %attacker计算的下一时刻位置 (暂时用PD)
        %attack控制器更新
        y_a = x_hat_a - x_bar_list(:,epoch);

        u_a = K_a.C * xK_a + K_a.D * y_a;
        xK_a = K_a.A * xK_a + K_a.B * y_a;

        for i=1:3
            if u_a(i)>0.5
                u_a(i)=0.5;
            elseif u_a(i)<-0.5
                u_a(i)=-0.5;
            end
        end

        %attacker状态更新
        x_hat_a = C*x_hat_a + D* u_a;
        x_hat_a(6) = -x_hat_a(6);

        %defender用mpc
        a_last = a;
        y = x_hat_d' - x_bar_list(:,epoch);
        % 控制器更新
        a = K.C * xK + K.D * y;
        xK = K.A * xK + K.B * y;

        T_NED = [old_est_C_b_e_d * Detector.T_ECEF(7:9);old_est_C_b_e_d * Detector.T_ECEF(4:6)];
        %计算nominal
        for i=1:3
            if a(i)>0.5
                a(i)=0.5;
            elseif a(i)<-0.5
                a(i)=-0.5;
            end
        end

        [a_opt] = exposure_mpc(ka, x_hat_d', x_hat_a, T_NED, Kc, x_bar_list, C, D, MPC , n_x, n_u, a_last, a, K_a, xK_a);
        a_shake= a_opt-a;
        disp("nominal control is");
        disp(a);
        disp("shake is");
        disp(a_shake);

        x = C*x + D* (a_opt) + w_k;
        x(6) = -x(6);

    elseif epoch>=62 && exposure_flag==1  %攻击已被检测，程序结束
        break;

    end

    % 计算经度变化和新的经纬度 (前进的距离 / 地球的纬线半径)
    dela_latitude = x(1) /R_earth;
    latitude = latitude_rad_initial + dela_latitude; % 新的纬度 (弧度)
    delta_longitude = x(2) / (R_earth * cos(latitude));
    longitude = longitude_rad_initial + delta_longitude; % 新的经度 (弧度)

    motion = [
        time(epoch);              % 时间
        latitude * rad_to_deg;             % 纬度
        longitude* rad_to_deg;            % 经度
        x(3);               % 高度 (m）
        x(4);                 % 北向速度 (m/s)
        x(5);                 % 东向速度 (m/s)
        x(6);                 % 下向速度 (m/s)
        0;                    % 横滚角 (假设为 0)
        0;                    % 俯仰角 (假设为 0)
        90                     % 偏航角 (假设为 0)
        ];


    in_profile = motion;
    % Convert degrees to radians 把角度转换为弧度
    in_profile(2:3) = deg_to_rad * in_profile(2:3);
    in_profile(8:10) = deg_to_rad * in_profile(8:10);

    motion_profile(epoch, :) = motion;
    x_log(epoch,1) = time(epoch);
    x_log(epoch, 2:7 ) = x';
    x_hat_log(epoch,1) = time(epoch);
    x_hat_log(epoch, 2:7) = x_hat';
    x_bar_log(epoch,1) = time(epoch);
    x_bar_log(epoch, 2:7) = x_bar_list(:,epoch)';

    a_log(epoch,1) = time(epoch);
    a_log(epoch,2:4) = a';

    if epoch>=62 && exposure_flag==0
        %=======仿照defender的，给attacker做一个inprofile========
        dela_latitude = x_hat_a(1) /R_earth;
        latitude = latitude_rad_initial + dela_latitude; % 新的纬度 (弧度)
        delta_longitude = x_hat_a(2) / (R_earth * cos(latitude));
        longitude = longitude_rad_initial + delta_longitude; % 新的经度 (弧度)

        motion_a = [
            time(epoch);              % 时间
            latitude * rad_to_deg;             % 纬度
            longitude* rad_to_deg;            % 经度
            x_hat_a(3);               % 高度 (m）
            x_hat_a(4);                 % 北向速度 (m/s)
            x_hat_a(5);                 % 东向速度 (m/s)
            x_hat_a(6);                 % 下向速度 (m/s)
            0;                    % 横滚角 (假设为 0)
            0;                    % 俯仰角 (假设为 0)
            90                     % 偏航角 (假设为 0)
            ];


        in_profile_a = motion_a;
        % Convert degrees to radians 把角度转换为弧度
        in_profile_a(2:3) = deg_to_rad * in_profile_a(2:3);
        in_profile_a(8:10) = deg_to_rad * in_profile_a(8:10);
    end

end

writematrix(motion_profile, 'output/motion_profile.csv');
writematrix(x_log,'output/x_log.csv');
writematrix(a_log,'output/a_log.csv');
writematrix(x_hat_log,'output/x_hat_log.csv');
writematrix(x_bar_log,'output/x_bar_log.csv');
writematrix(delta_KF_x_log,'output/delta_KF_x_log.csv');
writematrix(lamda_GI_result,'output/lamda_GI_result.csv');

% 显示detection结果

figure;

subplot(2, 1, 1);
plot(x_hat_log(:, 3), x_hat_log(:, 2));
title('x_hat');
ylabel('north');
xlabel('east');

subplot(2, 1, 2);
plot(x_log(:, 3), x_log(:, 2));
title('x_log');
xlabel('y true');
ylabel('x true');

exportgraphics(gcf, 'result.png', 'Resolution',800, ...
    'ContentType', 'vector', ...
    'BackgroundColor', 'none');