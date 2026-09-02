function [spoofed_GNSS_measurement, delta_x] = Spoofed_GNSS_measurement(est_r_eb_e_spoofer, ...
    est_v_eb_e_spoofer,Detector, ...
    no_meas,GNSS_measurements,K_matrix, K_matrix_denominator,e_fixed,est_clock_old, tor_s)

 
% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
Omega_ie = Skew_symmetric([0,0,omega_ie]);

%T2的计算不累加
pred_meas = zeros(no_meas,2);
meas_add_divi_M = zeros(no_meas,2); 
u_as_e_T = zeros(no_meas,3);

spoofed_GNSS_measurement = zeros(no_meas, 1);


% 3. Propagate state estimates using (3.14) noting that only the clock
% states are non-zero due to closed-loop correction.
x_est_propagated(1:15,1) = 0;
x_est_propagated(16,1) = est_clock_old(1) + est_clock_old(2) * tor_s;
x_est_propagated(17,1) = est_clock_old(2);


% Loop measurements =》Number of satellites
for j = 1:no_meas

    % Predict approx range 
    delta_r = GNSS_measurements(j,3:5)' - est_r_eb_e_spoofer; %第j行，第三到五列：Satellite ECEF position (m)- prior estimated ECEF user position (m)
    approx_range = sqrt(delta_r' * delta_r);

    % Calculate frame rotation during signal transit time using (8.36)
    C_e_I = [1, omega_ie * approx_range / c, 0;...4444
             -omega_ie * approx_range / c, 1, 0;...
             0, 0, 1];

    % Predict pseudo-range using (9.165)
    delta_r = C_e_I *  GNSS_measurements(j,3:5)' - est_r_eb_e_spoofer;
    range = sqrt(delta_r' * delta_r);
    pred_meas(j,1) = range + x_est_propagated(16);
        
    % Predict line of sight
    u_as_e_T(j,1:3) = delta_r' / range;
        
    % Predict pseudo-range rate using (9.165)
    range_rate = u_as_e_T(j,1:3) * (C_e_I * (GNSS_measurements(j,6:8)' +...
        Omega_ie * GNSS_measurements(j,3:5)') - (est_v_eb_e_spoofer +...
        Omega_ie * est_r_eb_e_spoofer));        
    pred_meas(j,2) = range_rate + x_est_propagated(17);

end % for j

%===========添加attack vector begin=================
[L, ~] = chol(K_matrix_denominator, 'lower');

attack_margin = Detector.threshold;

violate_detection = 1;

while violate_detection==1 %构造并检查是否满足第二个detection条件
    %-------------------
    % use an ramdom attack direction vector
    %-------------------
    % e = randn(2*no_meas,1);
    % e = e/norm(e);
    % 
    % e1 = sqrt(attack_margin) * e;
    % 
    % %-----------------------------
    % % %调节attacker每一步的攻击程度
    % %-----------------------------
    % a =0.9*L*e1;

    %-------------------
    % use a fixed attack direction vector
    %-------------------
    e1 = sqrt(attack_margin) * e_fixed;
    a =0.2* L*e1;

    %-------------------
    %Spoofing ALgorithm 1：
    % e is used same though the whole spoofing process
    %-------------------
    spoofed_GNSS_measurement(1:no_meas,1)= pred_meas(1:no_meas,1) + a(1:no_meas,1);
    spoofed_GNSS_measurement(1:no_meas,2)= pred_meas(1:no_meas,2) + a(no_meas+1:no_meas*2,1);
    spoofed_GNSS_measurement(1:no_meas,3:8) = GNSS_measurements(1:no_meas, 3:8);
    %-------------------
    %Spoofing ALgorithm 2：
    % e is added to the original measurements in the first half process and
    % subtracted in secnd half process
    %-------------------
    % if GNSS_epoch/2==0  %the first 20s of attack
    %     meas_add_divi_M(1:no_meas,1)= pred_meas(1:no_meas,1) + a(1:no_meas,1);
    %     meas_add_divi_M(1:no_meas,2)= pred_meas(1:no_meas,2) + a(no_meas+1:no_meas*2,1);
    % else
    %     meas_add_divi_M(1:no_meas,1)= pred_meas(1:no_meas,1) - a(1:no_meas,1);
    %     meas_add_divi_M(1:no_meas,2)= pred_meas(1:no_meas,2) - a(no_meas+1:no_meas*2,1);
    % end

    %spoofed_GNSS_measurement(1:no_meas,3:8) = GNSS_measurements(1:no_meas, 3:8);
    % diviation_M = meas_add_divi_M -GNSS_measurements;
    % diviation_M_M = [diviation_M(1:8,1);diviation_M(1:8,2)];


    delta_z(1:no_meas,1) = spoofed_GNSS_measurement(1:no_meas,1) -pred_meas(1:no_meas,1);
    delta_z((no_meas + 1):(2 * no_meas),1) = spoofed_GNSS_measurement(1:no_meas,2) -pred_meas(1:no_meas,2);

    detection_value_attacker = (delta_z)' * inv(K_matrix_denominator) * (delta_z);
    %violate_detection=0;

%----检查是否可通过detector2，如果可通过用，不可通过缩小-----
    delta_x =K_matrix * delta_z;
    violate_detection=0;
    if max(abs(delta_x) - Detector.T_ECEF)>1e-03
            

            violate_detection=1;
            disp("The attack deviation did not pass the detector 2, reconstructing...");

            %y有违反的，减小a
            a = 0.5* a;
            spoofed_GNSS_measurement(1:no_meas,1)= pred_meas(1:no_meas,1) + a(1:no_meas,1);
            meas_add_dispoofed_GNSS_measurementvi_M(1:no_meas,2)= pred_meas(1:no_meas,2) + a(no_meas+1:no_meas*2,1);
            spoofed_GNSS_measurement(1:no_meas,3:8) = GNSS_measurements(1:no_meas, 3:8);

            delta_z(1:no_meas,1) = meas_add_divi_M(1:no_meas,1) -pred_meas(1:no_meas,1);
            delta_z((no_meas + 1):(2 * no_meas),1) = spoofed_GNSS_measurement(1:no_meas,2) -pred_meas(1:no_meas,2);
            detection_value_attacker = (delta_z)' * inv(K_matrix_denominator) * (delta_z);
    end
end
delta_x = K_matrix * delta_z;


%===========添加attack vector END.=================
end

