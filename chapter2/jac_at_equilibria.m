clear all; clc; close all;

% Parameters
ms = 577;        % kg       (sprung mass)
mu = 50;         % kg       (unsprung mass)
ks = 20e3;       % N/m      (stiffness of the spring)
ku = 2.5e5;      % N/m      (stiffness of the tire)
Ls = 0.4;       % m        (length of the unstretched spring)
Lu = 0.34;       % m        (length of the unstretched tire)
cs = 2e3;        % N/(m/s)  (damping)
g  = 9.81;       % m/s^2    (gravity)
epsilon = 30;    % m^-2     (hardening effect)

syms x1 x2 x3 x4 real

% Define f symbolically
f = [ x3;
      x4;
      (ks*(x2-x1)*(1 + epsilon*(x2-x1)^2) + cs*(x4-x3) + ks*Ls - ms*g)/ms;
      (-ks*(x2-x1)*(1 + epsilon*(x2-x1)^2) - cs*(x4-x3) - ks*Ls - mu*g - ku*x2 + ku*Lu)/mu ];

X = [x1 x2 x3 x4];

% Compute Jacobian symbolically
J = jacobian(f, X);

% Equilibrium point (numerical)
xA = [0.41 0.32 0 0];

% Evaluate Jacobian numerically
J_num = double(subs(J, X, xA));

disp('J(xA):')
format short g
disp(J_num)

lambda = eig(J_num);

disp('Eigenvalues of J(xA):')
disp(lambda)

figure;
plot(real(lambda), imag(lambda), 'rx','MarkerSize',10,'LineWidth',2);
grid on
xlabel('Re(\lambda)')
ylabel('Im(\lambda)')
title('Eigenvalues of J(x_A)')

A = J_num;
save('linearized_model.mat','A')