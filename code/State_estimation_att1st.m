%加上攻击，有检测器
function [K_matrix_denominator, K_matrix,lamda_a,out_IMU_bias_est, out_clock, out_profile_a,out_profile_d, old_time,...
    quant_residuals,est_IMU_bias, ...
    old_est_r_eb_e, old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
    time_last_GNSS,GNSS_epoch,est_clock, ...
    P_matrix,est_L_b,...
    G_out_clock,G_x_est,G_P_matrix, G_old_time, detection_value_attacker] = State_estimation_att1st(Detector, ...
    delta_a, out_IMU_bias_est, out_clock, in_profile, old_time,IMU_errors,GNSS_config,TC_KF_config,...
    quant_residuals, ...
    est_IMU_bias, old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,old_true_C_b_e,old_true_v_eb_e,old_true_r_eb_e, ...
    time_last_GNSS,GNSS_epoch,est_clock,P_matrix, est_L_b,GNSS_biases, ...
    GNSS_KF_config, G_x_est,G_P_matrix, G_old_time, G_out_clock, e_fixed, K_matrix_denominator,K_matrix)

%old_true_v_eb_e 用于IMU计算f和omega
%old_est_r_eb_e  用于KF更新

time = in_profile(1);
true_L_b = in_profile(2);
true_lambda_b = in_profile(3);
true_h_b = in_profile(4);
true_v_eb_n = in_profile(5:7);
true_eul_nb = in_profile(8:10)';
true_C_b_n = Euler_to_CTM(true_eul_nb)';

[true_r_eb_e,true_v_eb_e,true_C_b_e] =...
    NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);

lamda_a = 0;
detection_value_attacker = 0;

% Time interval
tor_i = time - old_time;

% Calculate specific force and angular rate
[true_f_ib_b,true_omega_ib_b] = Kinematics_ECEF(tor_i,true_C_b_e,...
    old_true_C_b_e,true_v_eb_e,old_true_v_eb_e,old_true_r_eb_e);

% Simulate IMU errors  %角速度和特定力
[meas_f_ib_b,meas_omega_ib_b,quant_residuals] = IMU_model(tor_i,...
    true_f_ib_b,true_omega_ib_b,IMU_errors,quant_residuals);

% Correct IMU errors
meas_f_ib_b = meas_f_ib_b - est_IMU_bias(1:3);
meas_omega_ib_b = meas_omega_ib_b - est_IMU_bias(4:6);

% Update estimated navigation solution  位置、速度、转移矩阵
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix
[est_r_eb_e,est_v_eb_e,est_C_b_e] = Nav_equations_ECEF(tor_i,...
    old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,meas_f_ib_b,...
    meas_omega_ib_b);

%本次开始，defender和attacker分开维护,defender的out_profile_d只用IMU
% Convert navigation solution to NED 输出经、纬、高、速度、转换矩阵
[est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
    ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);


% Generate defender output profile record
out_profile_d(1,1) = time;
out_profile_d(1,2) = est_L_b;
out_profile_d(1,3) = est_lambda_b;
out_profile_d(1,4) = est_h_b;
out_profile_d(1,5:7) = est_v_eb_n;


% Determine whether to update GNSS simulation and run Kalman filter
% 根据GNSS的更新率，决定是否执行GNSS模拟和卡尔曼滤波器的更新。
if (time - time_last_GNSS) >= GNSS_config.epoch_interval
    GNSS_epoch = GNSS_epoch + 1;
    tor_s = time - time_last_GNSS;  % KF time interval
    time_last_GNSS = time;

    % Determine satellite positions and velocities
    [sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(time,...
        GNSS_config);

    % Generate GNSS measurements
    [GNSS_measurements,no_meas] = Generate_GNSS_measurements(...
        time,sat_r_es_e,sat_v_es_e,true_r_eb_e,true_L_b,true_lambda_b,...
        true_v_eb_e,GNSS_biases,GNSS_config);

    %生成 spoofed GNSS measurements
    est_r_eb_e_spoofer = est_r_eb_e;
    est_v_eb_e_spoofer = est_v_eb_e;


    [spoofed_GNSS_measurement, delta_x] = Spoofed_GNSS_measurement(est_r_eb_e_spoofer, ...
    est_v_eb_e_spoofer,Detector, ...
    no_meas,GNSS_measurements,K_matrix, K_matrix_denominator,e_fixed,est_clock, tor_s);

    [K_matrix_denominator, K_matrix, lamda_a, delta_KF_x, est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,...
            est_clock,P_matrix] = TC_KF_Epoch_normal(spoofed_GNSS_measurement,...
            no_meas,tor_s,est_C_b_e,est_v_eb_e,est_r_eb_e,...
            est_IMU_bias,est_clock,P_matrix,meas_f_ib_b,...
            est_L_b,TC_KF_config);


     
    % Generate IMU bias and clock output records
    out_IMU_bias_est(GNSS_epoch,1) = time;
    out_IMU_bias_est(GNSS_epoch,2:7) = est_IMU_bias';
    out_clock(GNSS_epoch,1) = time;
    out_clock(GNSS_epoch,2:3) = est_clock;

    % Generate KF uncertainty output record
    out_KF_SD(GNSS_epoch,1) = time;
    for i =1:17
        out_KF_SD(GNSS_epoch,i+1) = sqrt(P_matrix(i,i));
    end % for i

end % if time

% Convert navigation solution to NED 输出经、纬、高、速度、转换矩阵
[est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
    ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);

% Generate output profile record for attacker
out_profile_a(1,1) = time;
out_profile_a(1,2) = est_L_b;
out_profile_a(1,3) = est_lambda_b;
out_profile_a(1,4) = est_h_b;
out_profile_a(1,5:7) = est_v_eb_n;


% Reset old values for attacker
old_time = time;
old_true_r_eb_e = true_r_eb_e;
old_true_v_eb_e = true_v_eb_e;
old_true_C_b_e = true_C_b_e;
old_est_r_eb_e = est_r_eb_e;
old_est_v_eb_e = est_v_eb_e;
old_est_C_b_e = est_C_b_e;












