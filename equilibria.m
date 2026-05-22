clear all; clc; close all;

% Parameters
ms = 577;        % kg       (sprung mass)
mu = 50;         % kg       (unsprung mass)
ks = 20e3;       % N/m      (stiffness of the spring)
ku = 2.5e5;      % N/m      (stiffness of the tire)
Ls = 0.40;       % m        (lenght of the unstretched spring)
Lu = 0.34;       % m        (lenght of the unstretched tire)
cs = 2e3;        % N/(m/s)  (damping)
g  = 9.81;       % m/s^2    (gravity)
epsilon = 20;    % m^-2     (hardening effect)

syms x1 x2 x3 x4 real
f1 = x3;
f2 = x4;
f3 = (ks*(x2-x1)*(1+epsilon*(x2-x1)^2)+cs*(x4-x3)+ks*Ls-ms*g)/ms;
f4 = (-ks*(x2-x1)*(1+epsilon*(x2-x1)^2)-cs*(x4-x3)-ks*Ls-mu*g-ku*x2+ku*Lu)...
    /mu;

sol = vpasolve([f1 == 0, f2 == 0, f3 == 0, f4 == 0], [x1, x2, x3, x4]);

disp('Equilibrium points:');

T = struct2table(sol);
T = round(T,4);   % 2 decimal digits
disp(T);