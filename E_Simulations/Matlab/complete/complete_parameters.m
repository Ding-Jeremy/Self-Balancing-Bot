clear; clc;

% Parameters
% STRUCTURE 
m_b = 1.018;    % Mass of the chassis (without the wheels)
m_w = 0.195;    % Mass of one of the wheels
l = 0.0344;    % Distance to center of mass
g = 9.81;   % Gravity constant
r = 0.055;  % Wheels radius
J_w = 2.522e-4;      % Moment of inertia of each wheels
J_b = 2.822e-3; % Moment of inertia of the body (main pendulum)

% Physcis equations
a = m_b*l;
I_O = J_b+m_b*l^2;
m_tot=m_b+2*m_w;
m_O=m_tot+J_w/r^2;

%IMU position (from wheel axis).
rho = 0.05130; % distance [m]
phi = -0.373;   % angle [rad]

% Sampling period
T_s = 0.002; % 500 Hz

% Regulator (Inner Loop)
Kp_t = 1.53;
Td_t = 0.38;
Ti_t = 0;

% Regulator (Outer Loop)
Kp_v = 26;
Td_v = 0;
Ti_v = 0;


% Whole parameters
P = [m_b;m_w;l;g;r;J_w;J_b;rho;phi;T_s];
