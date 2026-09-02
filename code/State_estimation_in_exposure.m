function [quant_residuals,out_profile_a, out_profile_d, ...
    old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d,old_est_r_eb_e_a, old_est_v_eb_e_a,...
    GNSS_epoch,exposure_flag,...
    old_true_r_eb_e_d, old_true_v_eb_e_d, old_true_C_b_e_d,...
    old_time,est_IMU_bias_d, est_clock_a, est_L_b] = State_estimation_in_exposure(K_matrix_denominator, K_matrix,Detector, ...
   GNSS_epoch,GNSS_config,GNSS_biases,...
    in_profile_a, in_profile_d, quant_residuals,IMU_errors,...
old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d, old_est_r_eb_e_a,old_est_v_eb_e_a,old_est_C_b_e_a, ...
old_true_r_eb_e_d, old_true_v_eb_e_d, old_true_C_b_e_d,est_clock_a,old_time,est_IMU_bias_d,e_fixed)


%inputs:
    % old_est_C_b_e_a 不变
    % old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d  defender——IMU的测量值
    % old_est_r_eb_e_a,old_est_v_eb_e_a,old_est_C_b_e_a  attacker——GNSS的测量值

%defender只使用IMU数值，attacker基于自己维护的state计算spoofed GNSS measurement
%===========================
% 算defender的IMU检测值
%============================

time = in_profile_d(1);

true_L_b = in_profile_d(2);
true_lambda_b = in_profile_d(3);
true_h_b = in_profile_d(4);
true_v_eb_n = in_profile_d(5:7);
true_eul_nb = in_profile_d(8:10)';
true_C_b_n = Euler_to_CTM(true_eul_nb)';

[true_r_eb_e_d,true_v_eb_e_d,true_C_b_e_d] =...
    NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);


% Time interval
tor_i = time - old_time;

% Calculate specific force and angular rate
[true_f_ib_b,true_omega_ib_b] = Kinematics_ECEF(tor_i,true_C_b_e_d,...
    old_true_C_b_e_d,true_v_eb_e_d,old_true_v_eb_e_d,old_true_r_eb_e_d);


% Simulate IMU errors  %角速度和特定力
[meas_f_ib_b,meas_omega_ib_b,quant_residuals] = IMU_model(tor_i,...
    true_f_ib_b,true_omega_ib_b,IMU_errors,quant_residuals);

% Correct IMU errors 修改
% meas_f_ib_b = meas_f_ib_b - est_IMU_bias_d(1:3);
% meas_omega_ib_b = meas_omega_ib_b - est_IMU_bias_d(4:6);

% Update estimated navigation solution  位置、速度、转移矩阵
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix
[est_r_eb_e_d,est_v_eb_e_d,est_C_b_e_d] = Nav_equations_ECEF(tor_i,...
    old_est_r_eb_e_d,old_est_v_eb_e_d,old_est_C_b_e_d,meas_f_ib_b,...
    meas_omega_ib_b);

% Convert navigation solution to NED 输出经、纬、高、速度、转换矩阵
[est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
    ECEF_to_NED(est_r_eb_e_d,est_v_eb_e_d,est_C_b_e_d);

% Generate defender output profile record
out_profile_d(1,1) = time;
out_profile_d(1,2) = est_L_b;
out_profile_d(1,3) = est_lambda_b;
out_profile_d(1,4) = est_h_b;
out_profile_d(1,5:7) = est_v_eb_n;

%  defender维护的旧值
% Reset old values
old_true_r_eb_e_d = true_r_eb_e_d;
old_true_v_eb_e_d = true_v_eb_e_d;
old_true_C_b_e_d = true_C_b_e_d;
old_est_C_b_e_d = est_C_b_e_d;
old_est_v_eb_e_d = est_v_eb_e_d;
old_est_r_eb_e_d = est_r_eb_e_d;

%===========================
% 算attacker这一把的攻击值
%============================

true_L_b_a = in_profile_a(2);
true_lambda_b_a = in_profile_a(3);
true_h_b_a = in_profile_a(4);
true_v_eb_n_a = in_profile_a(5:7);
true_eul_nb_a = in_profile_a(8:10)';
true_C_b_n_a = Euler_to_CTM(true_eul_nb)';

[true_r_eb_e_a,true_v_eb_e_a,true_C_b_e_a] =...
    NED_to_ECEF(true_L_b_a,true_lambda_b_a,true_h_b_a,true_v_eb_n_a,true_C_b_n_a);

GNSS_epoch = GNSS_epoch + 1;

%攻击者没有实际的运行值，只有估计值。delta_a是攻击者的计算误差。暂时是0.
est_r_eb_e_a = true_r_eb_e_a;
est_v_eb_e_a = true_v_eb_e_a;
est_C_b_e_a = true_C_b_e_a;

% Determine satellite positions and velocities
[sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(time,...
    GNSS_config);

%==========伪造GNSS measurement begin===========
%Generate GNSS measurements
[GNSS_measurements,no_meas] = Generate_GNSS_measurements(...
    time,sat_r_es_e,sat_v_es_e,true_r_eb_e_a,true_L_b_a,true_lambda_b_a,...
    true_v_eb_e_a,GNSS_biases,GNSS_config);

tor_s=0.5;
%delta_x是造成的K * delta_z
[GNSS_measurements, delta_x] = Spoofed_GNSS_measurement(est_r_eb_e_a, ...
    est_v_eb_e_a,Detector, ...
    no_meas,GNSS_measurements,K_matrix, K_matrix_denominator,e_fixed,est_clock_a, tor_s);


% 3. Propagate state estimates using (3.14) noting that only the clock
% states are non-zero due to closed-loop correction.
% x_est_propagated(1:15,1) = 0;
% x_est_propagated(16,1) = est_clock_a(1) + est_clock_a(2) * tor_s;
% x_est_propagated(17,1) = est_clock_a(2);

% Update IMU bias and GNSS receiver clock estimates
% 9. Update state estimates using (3.24)
%x_est_new = x_est_propagated + delta_x;
%est_IMU_bias_d = est_IMU_bias_d + x_est_new(10:15);
%est_clock_a = x_est_new(16:17)';

%更新攻击者认为的system estimated state
old_est_v_eb_e_a = old_est_v_eb_e_a + delta_x(4:6);
old_est_r_eb_e_a = old_est_r_eb_e_a+  delta_x(7:9);
%old_est_C_b_e_a不变
%==========伪造GNSS measurement end===========


%不需要再经历kalman filter了，attacker认为是固定的。
%单纯检测，检测1+检测2
[exposure_flag] = Detection(GNSS_epoch, GNSS_measurements, no_meas,K_matrix, ...
    K_matrix_denominator, est_r_eb_e_d,est_v_eb_e_d, est_clock_a, Detector);

% Convert navigation solution to NED 输出经、纬、高、速度、转换矩阵
[est_L_b_a,est_lambda_b_a,est_h_b_a,est_v_eb_n_a,~] =...
    ECEF_to_NED(old_est_r_eb_e_a,old_est_v_eb_e_a,old_est_C_b_e_a);

% Generate defender output profile record
out_profile_a(1,1) = time;
out_profile_a(1,2) = est_L_b_a;
out_profile_a(1,3) = est_lambda_b_a;
out_profile_a(1,4) = est_h_b_a;
out_profile_a(1,5:7) = est_v_eb_n_a;

old_time = time;

end % if time
