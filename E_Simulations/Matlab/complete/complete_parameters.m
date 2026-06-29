clear; clc;

% Parameters
% STRUCTURE 
m_b = 1;    % Mass of the pendulum (without the wheels)
m_w = 0.1;    % Mass of one of the wheels
l = 0.08;    % Distance to center of mass
g = 9.81;   % Gravity constant
r = 0.055;  % Wheels radius
d = 0.141;   % Distance between wheels
Jw = 84351.64*1e-9;      % Moment of inertia of each wheels
Jbod = m_b*l^2; % Moment of inertia of the body (main pendulum)

% MOTORS
w_max=20; % Max angular speed [rad/s]
T_hold = 0.45; % Holding torque [nm]
P = [m_b;m_w;l;g;r;d;Jw;Jbod];
