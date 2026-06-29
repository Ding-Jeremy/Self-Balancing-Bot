clear; clc;

% Parameters
M = 0.5;
m = 0.2;
l = 0.1;
g = 9.81;
r = 0.055;
tau = 0.02;
P = [M;m;l;g;r;tau];

% Motor parameters
t_m = 0.1; % Motor response (1st order filter)
t_f = t_m/10; % Filtered derivative