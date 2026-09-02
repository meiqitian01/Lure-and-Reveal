function [exposure_flag] = Detection(GNSS_epoch, GNSS_measurements, no_meas,K_matrix, ...
    K_matrix_denominator, est_r_eb_e_d,est_v_eb_e_d, est_clock_old, Detector)


% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);
%先计算predicted measurement
%T2的计算不累加
pred_meas = zeros(no_meas,2);
u_as_e_T = zeros(no_meas,3);
exposure_flag=0;

tor_s=0.5;
% 3. Propagate state estimates using (3.14) noting that only the clock
% states are non-zero due to closed-loop correction.
x_est_propagated(1:15,1) = 0;
x_est_propagated(16,1) = est_clock_old(1) + est_clock_old(2) * tor_s;
x_est_propagated(17,1) = est_clock_old(2);


% Loop measurements =》Number of satellites
for j = 1:no_meas

    % Predict approx range 
    delta_r = GNSS_measurements(j,3:5)' - est_r_eb_e_d; %第j行，第三到五列：Satellite ECEF position (m)- prior estimated ECEF user position (m)
    approx_range = sqrt(delta_r' * delta_r);

    % Calculate frame rotation during signal transit time using (8.36)
    C_e_I = [1, omega_ie * approx_range / c, 0;...4444
             -omega_ie * approx_range / c, 1, 0;...
             0, 0, 1];

    % Predict pseudo-range using (9.165)
    delta_r = C_e_I *  GNSS_measurements(j,3:5)' - est_r_eb_e_d;
    range = sqrt(delta_r' * delta_r);
    pred_meas(j,1) = range + x_est_propagated(16);
        
    % Predict line of sight
    u_as_e_T(j,1:3) = delta_r' / range;
        
    % Predict pseudo-range rate using (9.165)
    range_rate = u_as_e_T(j,1:3) * (C_e_I * (GNSS_measurements(j,6:8)' +...
        Omega_ie * GNSS_measurements(j,3:5)') - (est_v_eb_e_d +...
        Omega_ie * est_r_eb_e_d));        
    pred_meas(j,2) = range_rate + x_est_propagated(17);
end % for j

% 8. Formulate measurement innovations using (14.119)
delta_z(1:no_meas,1) = GNSS_measurements(1:no_meas,1) -...
    pred_meas(1:no_meas,1); 
delta_z((no_meas + 1):(2 * no_meas),1) = GNSS_measurements(1:no_meas,2) -...
    pred_meas(1:no_meas,2);


%====================
%detector 1+2
%====================
% lamda = (delta_z)' * inv(K_matrix_denominator) * (delta_z);
% 
% if lamda> Detector.threshold
%     exposure_flag =1;
% else
%     %====================
%     %通过detection 1，检查detector 2
%     %====================
%     delta_x_ECEF = K_matrix * delta_z;
% 
%     if max(abs(delta_x_ECEF(4:9)) - Detector.T_ECEF(4:9))>1e-05
%         exposure_flag =1;
%     end
% 
% end
% disp(['GNSS_epoch=', num2str(GNSS_epoch), '  ,exposure_flag = ', num2str(exposure_flag), ...
%     '  ,detection statistic = ', num2str(lamda)]);
% 
% %function end

%only detection two
delta_x_ECEF = K_matrix * delta_z;
%lamda2 = max(abs(delta_x_ECEF(4:9)) - Detector.T_ECEF(4:9));
lamda2 = max(abs(delta_x_ECEF(4:9)));
if lamda2>Detector.T_ECEF(4)
    exposure_flag =1;
end

disp(['GNSS_epoch=', num2str(GNSS_epoch), '  ,exposure_flag = ', num2str(exposure_flag),'  ,detection statistic = ', num2str(lamda2)]);


end
