function [e_y,pi_p] = crossTrackError(xk1, yk1, xk, yk, x)
% [e_y,pi_p] = crossTrackError(xk1,yk1,xk,yk,x(4),x(5)) computes the cross-
% track error e_y for a craft located at position (x(4),x(5)) when the path
% is a straight line from waypoint (xk,yk) to waypoint (xk1,yk1).
%
% Input:    (xk,yk) and (xk1,yk1), waypoints  expressed in NED
%           (x(4),x(5)), craft North-East positions 
%
% Outputs:  e_y and pi_p, cross-track error expressed in NED
%

% path-tangential angle with respect to the North axis
pi_p = atan2(yk1-yk, xk1-xk);   % Fossen (12.54)

% cross-track error expressed in NED
e_y = -(x(4)-xk) * sin(pi_p) + (x(5)-yk) * cos(pi_p);   % Fossen (12.55)

end