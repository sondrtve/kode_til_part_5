function [xk1,yk1,xk,yk,last] = WP_selector(x)
% [xk1,yk1,xk,yk,last] = WP_selector(x) extracts the current and future 
% waypoints, based on the craft's position in NED.
%
% Input:    x(4), x(5), craft x- and y-position expressed in NED
%
% Outputs:  (xk,yk) and (xk1,yk1), waypoints  expressed in NED
%           last, number of previous waypoint expressed in NED
%

    persistent WP
    persistent k

    if isempty(WP)
        load('WP.mat');     % Loading the waypoints
    end

    if isempty(k)
        k = 1;
    end

    L = 161;                % Ship length
    R = 4 * L;              % Radius for circle of acceptance

    x_pos = x(4);           % Extract the positions in NED from the state vector
    y_pos = x(5);

    if k >= length(WP)      % Ensure matlab stops reading from file if empty
        xk = 0;
        yk = 0;
        yk1 = 0;
        xk1 = 0;
        last = 1;           % Set flag if last waypoint is reached
    else
        xk = WP(1,k);       % Current waypoint
        yk = WP(2,k);
        xk1 = WP(1,k+1);    % Aiming waypoint
        yk1 = WP(2,k+1);
        last = 0;
    end

    if ((xk1-x_pos)^2 + (yk1-y_pos)^2) <= R^2   % Criteria for selecting next waypoint Fossen (12.52)
        k = k + 1;
    end

end
