function delta_c = PID_heading(e_psi, e_r, e_int)
    % PID_heading implements a PID controller for heading control.
    %
    % Inputs:
    %   e_psi - Heading error (psi - psi_d) in radians
    %   e_r   - Yaw rate error (r - r_d) in radians per second
    %   e_int - Integral of heading error over time
    %
    % Output:
    %   delta_c - Commanded rudder angle in radians
    wb = 0.06; 
    zeta = 1;
    T = 168.2782; 
    K = 0.00745;
    
    wn = 1/(sqrt(1-2*zeta^2+sqrt(4*zeta^4-4*zeta^2+2))) * wb;
    Kp = T/K*wn^2;
    Kd = 2*1*wn*T/K - 1/K;
    Ki = wn/10 * Kp;


    % Control law
    %delta_c = -Kp * e_psi - Kd * e_r - Ki * e_int; % PID
    delta_c = -Kp * e_psi - Kd * e_r; % PD
end
