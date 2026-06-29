function dX = pendulum_dynamics(~, X, M, m, l, g)
% Defines derivatives
x = X(1);
x_dot = X(2);
theta = X(3);
theta_dot = X(4);

F = 0;  % no control yet

den = M + m - m*cos(theta)^2;

% Linear acceleration
bx = 0.01; % linear damping coefficient

x_ddot = (F ...
          - bx*x_dot ...
          + m*l*sin(theta)*theta_dot^2 ...
          - m*g*sin(theta)*cos(theta)) / den;
% Angular acceleration
btheta = 0.01; % angular damping

theta_ddot = (g*sin(theta) ...
              - cos(theta)*x_ddot ...
              - btheta*theta_dot/(m*l)) / l;

dX = zeros(4,1);
dX(1) = x_dot;
dX(2) = x_ddot;
dX(3) = theta_dot;
dX(4) = theta_ddot;

end