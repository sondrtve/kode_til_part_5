% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           My name
% Study program:    My study program
close all;
clear all;
% Add folder for 3-D visualization files
addpath(genpath('flypath3d_v2'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
T_final = 6000;	        % Final simulation time (s)
h = 0.1;                % Sampling time (s)

psi_ref = -110 * pi/180;% desired yaw angle (rad)
U_ref   = 9;            % desired surge speed (m/s)

% Current disturbance
Vc = 1;                 % Current speed in m/s
beta_Vc = deg2rad(45);

% Initial states
V_w = 10;
beta_Vw_deg = 135;
beta_Vw = deg2rad(beta_Vw_deg);
rho_a = 1.247;
c_y = 0.95;
c_n = 0.15;
L_oa = 161; 
A_Lw = 10 * L_oa;

% initial states
eta_0 = [0 0 psi_ref]';       % Initial position and heading [x, y, psi]
nu_0  = [0 0 0]';       % Initial velocity components [u, v, r]
delta_0 = 0;            % Initial rudder angle
n_0 = 0;                % Initial propeller speed
Qm_0 = 0;               % Initial motor torque
x = [nu_0' eta_0' delta_0 n_0 Qm_0]'; % The state vector can be extended with addional states here
xd = [0 0 0]';          % Reference model desired states: psi_d, psi_ddot, psi_dddot
e_int = 0;              % integral error
e_yint = 0;             % integral cross-track error

% Load the waypoints and extract the matrix
WP = load('WP.mat');
WP = WP.WP;  % Access the actual waypoint matrix inside the struct

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0:h:T_final;                % Time vector
nTimeSteps = length(t);         % Number of time steps

simdata = zeros(nTimeSteps, 13); % Pre-allocate matrix for efficiency
wait_bar = waitbar(0, 'Starting');

% Initialize arrays to collect path data
x_path = zeros(1, nTimeSteps);
y_path = zeros(1, nTimeSteps);

% Plotting 2a
% Initialize arrays to store results for plotting
chi_vals = zeros(1, nTimeSteps);
chi_d_vals = zeros(1, nTimeSteps);
psi_vals = zeros(1, nTimeSteps);
beta_c_vals = zeros(1, nTimeSteps);
beta_vals = zeros(1, nTimeSteps);

% Main simulation loop with pathplotter call for real-time path plotting
% Main simulation loop
for i = 1:nTimeSteps
    % Current position and heading
    psi = x(6);    % Current heading of the ship

    % Part 2, 1A
    uc = Vc * cos(beta_Vc - psi);
    vc = Vc * sin(beta_Vc - psi);
    nu_c = [uc; vc; 0];
    
    % Part 2, 1c) Add wind
    u_w = V_w * cos(deg2rad(beta_Vw) - x(6));
    v_w = V_w * sin(deg2rad(beta_Vw) - x(6));
    u_rw = x(1) - u_w;
    v_rw = x(2) - v_w;
    V_rw = sqrt(u_rw^2 + v_rw^2);
    gamma_rw = -atan2(v_rw,u_rw);
    if t(i) >= 200      
        % Wind forces
        Ywind = 0.5 * rho_a * V_rw^2 * c_y * sin(gamma_rw) * A_Lw;
        Nwind = 0.5 * rho_a * V_rw^2 * c_n * sin(2 * gamma_rw) * A_Lw * L_oa;
        tau_wind = [0; Ywind; Nwind];
    else
        tau_wind = [0 0 0]';
    end
    
    % Legg til Part 4, 1b her
     % Part 4, 1b) Add the LOS guidance
    [xk1,yk1,xk,yk,last] = WP_selector(x);
    [e_y,pi_p] = crossTrackError(xk1, yk1, xk, yk, x);
    
    chi_d = guidance(e_y,pi_p,h);
    chi_d_vals(i) = chi_d;
    psi_ref = chi_d; 
    
    % Part 2, 2d) Add a reference model
    if i == 1
        xd = [psi_ref; 0; 0];
    end
    
    xd_dot = ref_model(xd, psi_ref);
    xd = xd + xd_dot * h;
    psi_d = xd(1);
    r_d = xd(2);
    u_d = U_ref;

    % Part 2, 2d) Add the heading controller
    if i == 1
        e_int = 0;
    end
    e_psi = ssa(psi - psi_d);
    r = x(3);
    e_r = r - r_d;
    e_int = e_int + e_psi * h;
    
    % Implementing integrator anti-windup
    e_tol = pi;                
    if e_int >= e_tol
        e_int = e_tol;
    elseif e_int <= -e_tol
        e_int = -e_tol;
    end
    
    delta_c = PID_heading(e_psi, e_r, e_int);

    % Part 3, 1e) Add open loop speed control
    %%n_c = 10;  % Open loop propeller speed [radians per second (rps)]

    % Part 3, 1f) Closed loop speed control
    n_c = PI_speed(U_ref);

    % Ship dynamics
    u = [delta_c; n_c];  % Control inputs: rudder angle and propeller speed
    [xdot, u] = ship(x, u, nu_c, tau_wind);  % Ship state derivatives
    
    % Store simulation data
    simdata(i, :) = [x(1:3)', x(4:6)', x(7), x(8), u(1), u(2), u_d, psi_d, r_d];     

    % Integrate to update state
    x = rk4(@ship, h, x, u, nu_c, tau_wind);
    
    % Collect the current position for path plotting
    x_path(i) = x(4);  % North position
    y_path(i) = x(5);  % East position
    
    % Display progress
    waitbar(i / nTimeSteps, wait_bar, sprintf('Progress: %d %%', floor(i / nTimeSteps * 100)));
end

close(wait_bar);

% Call pathplotter once after the simulation loop with the entire path
% pathplotter(x_path, y_path);
simdata = simdata(1:i,:);
t = t(1:i);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u           = simdata(:,1);                 % m/s
v           = simdata(:,2);                 % m/s
r           = simdata(:,3);                 % rad/s
r_deg       = (180/pi) * r;                 % deg/s
x           = simdata(:,4);                 % m
y           = simdata(:,5);                 % m
psi         = simdata(:,6);                 % rad
psi_deg     = (180/pi) * psi;               % deg
delta_deg   = (180/pi) * simdata(:,7);      % deg
n           = (30/pi) * simdata(:,8);       % rpm
delta_c_deg = (180/pi) * simdata(:,9);      % deg
n_c         = (30/pi) * simdata(:,10);      % rpm
u_d         = simdata(:,11);                % m/s
psi_d       = simdata(:,12);                % rad
psi_d_deg   = (180/pi) * psi_d;             % deg
r_d         =  simdata(:,13);               % rad/s
r_d_deg     = (180/pi) * r_d;               % deg/s

%%
figure(3)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi_deg,t,psi_d_deg,'linewidth',2);
title('Actual and desired yaw angle'); xlabel('Time (s)');  ylabel('Angle (deg)'); 
legend('actual yaw','desired yaw')
subplot(313)
plot(t,r_deg,t,r_d_deg,'linewidth',2);
title('Actual and desired yaw rates'); xlabel('Time (s)');  ylabel('Angle rate (deg/s)'); 
legend('actual yaw rate','desired yaw rate')

figure(2)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('actual surge','desired surge')
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed'); xlabel('Time (s)'); ylabel('Motor speed (RPM)');
legend('actual RPM','commanded RPM')
subplot(313)
plot(t,delta_deg,t,delta_c_deg,'linewidth',2);
title('Actual and commanded rudder angle'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('actual rudder angle','commanded rudder angle')
%% Create objects for 3-D visualization 
% Since we only simulate 3-DOF we need to construct zero arrays for the 
% excluded dimensions, including height, roll and pitch
z = zeros(length(x),1);
phi = zeros(length(psi),1);
theta = zeros(length(psi),1);

% create object 1: ship (ship1.mat)
new_object('flypath3d_v2/ship1.mat',[x,y,z,phi,theta,psi],...
'model','royalNavy2.mat','scale',(max(max(abs(x)),max(abs(y)))/1000),...
'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
'path','on','pathcolor',[.89 .0 .27],'pathwidth',2);

% Plot trajectories 
flypath('flypath3d_v2/ship1.mat',...
'animate','on','step',500,...
'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
'font','Georgia','fontsize',12,...
'view',[-25 35],'window',[900 900],...
'xlim', [min(y)-0.1*max(abs(y)),max(y)+0.1*max(abs(y))],... 
'ylim', [min(x)-0.1*max(abs(x)),max(x)+0.1*max(abs(x))], ...
'zlim', [-max(max(abs(x)),max(abs(y)))/100,max(max(abs(x)),max(abs(y)))/20]); 

%% Plots for task 2
% Extract data for plotting
u = simdata(:, 1);  % Surge velocity
v = simdata(:, 2);  % Sway velocity
psi_deg = simdata(:, 6) * 180 / pi;  % Heading angle in degrees
t = 0:h:T_final;

% Calculate angles
vr = simdata(:, 2);                      % Relative sway velocity
Ur = sqrt(u.^2 + v.^2);                  % Relative total speed
Chi_d = chi_d_vals * 180 / pi;           % Desired course in degrees
beta_c = atan(v ./ u) * 180 / pi;        % Crab angle in degrees
beta = asin(vr ./ Ur) * 180 / pi;        % Sideslip angle in degrees
chi = psi_deg + beta_c;                  % Course angle in degrees

% Plotting
figure;
subplot(3,1,1)
hold on
plot(t, beta_c, 'b', 'LineWidth', 1.5);  % Crab angle
plot(t, beta, 'r--', 'LineWidth', 1.5);  % Sideslip angle
hold off
title('Crab Angle \beta_c and Sideslip Angle \beta');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\beta_c [deg]', '\beta [deg]');
grid on

subplot(3,1,2)
hold on
plot(t, chi, 'g', 'LineWidth', 1.5);     % Course angle
plot(t, Chi_d, 'm--', 'LineWidth', 1.5); % Desired course angle
hold off
title('Course Angle \chi and Desired Course Angle \chi_d');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\chi [deg]', '\chi_d [deg]');
grid on

subplot(3,1,3)
plot(t, psi_deg, 'k', 'LineWidth', 1.5); % Heading angle
title('Heading Angle \psi');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\psi [deg]');
grid on
