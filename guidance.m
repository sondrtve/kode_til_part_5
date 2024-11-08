function chi_d = guidance(e_y, pi_p, h)
% chi_d = LOS_guidance(e_y,pi_p) computes the desired course angle chi_d
% for a craft following a straight line between waypoints.
%
% Input:    e_y and pi_p, cross-track error expressed in NED
%           x(1) and x(2), absolute velocities in surge and sway
%
% Outputs:  chi_d, desired course angle
%
    L = 161;                            % Length of ship (m)    
    delta = 10 * L;                     % Lookahead distance
    Kp = 1/delta;                       % Proportional gain Fossen (12.79)
    
    % %Part 4, 2c) Crab angle compensation
    % if (x(1)^2 + x(2)^2) <= 0 % x(1) = u og x(2) = v, må være non negative for sqrt()
    %     % LOS guidance without crab angle compensation
    %     chi_d = pi_p - atan(Kp * e_y);  % Fossen (12.78)
    % else
    %     % LOS guidance with crab angle compensation
    %     chi_d = pi_p - atan(Kp * e_y) - asin(x(2) / sqrt(x(1)^2 + x(2)^2));      % Fossen (12.106)
    % end

    % Part 4, 2d) Integral LOS
    kappa = 4;
    Ki = kappa * Kp;

    persistent e_yint
    if isempty(e_yint)
        e_yint = 0;
    end

    e_yint_dot = (delta * e_y) / (delta^2 + (e_y + kappa * e_yint)^2);
    e_yint = e_yint + h * e_yint_dot;

    e_tol = pi;                 % Integrator anti-windup
    if e_yint >= e_tol
        e_yint = e_tol;
    elseif e_yint <= -e_tol
        e_yint = -e_tol;
    end
    chi_d = pi_p - atan(Kp*e_y + Ki * e_yint);  % Fossen (12.108)
end
