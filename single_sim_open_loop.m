clear; clc; close all;

%% Parameters of the Model


ms = 577;
mu = 50;
ks = 20e3;
ku = 2.5e5;
Ls = 0.40;
Lu = 0.34;
cs = 2000;
g  = 9.81;
epsilon = 30;

%% Equilibria
syms x1 x2 x3 x4 real
f_sym = [ x3;
          x4;
          (ks*(x2-x1)*(1 + epsilon*(x2-x1)^2) + cs*(x4-x3) + ks*Ls - ms*g)/ms;
          (-ks*(x2-x1)*(1 + epsilon*(x2-x1)^2) - cs*(x4-x3) - ks*Ls - mu*g - ku*x2 + ku*Lu)/mu ];

sol = vpasolve(f_sym == 0, [x1 x2 x3 x4]);
xA = double([sol.x1 sol.x2 sol.x3 sol.x4]).';
disp('Equilibrium point xA ='); disp(xA.');

%% ODE Function

odefun = @(t,x) [ ...
    x(3);
    x(4);
    (ks*(x(2)-x(1))*(1+epsilon*(x(2)-x(1))^2) + cs*(x(4)-x(3)) + ks*Ls - ms*g)/ms;
    (-ks*(x(2)-x(1))*(1+epsilon*(x(2)-x(1))^2) - cs*(x(4)-x(3)) - ks*Ls - mu*g - ku*x(2) + ku*Lu)/mu ];

%% Initial Condition
alpha = 0.5;                     % Amplitude of the perturbation
v = randn(4,1); v = v/norm(v); % Direction
x0 = xA + alpha * v;

tspan = [0 5];

%% Simulation

[t,x] = ode45(odefun, tspan, x0);

%% Plot

figure('Name','States in Time');
tiledlayout(4,1);

colors = {[52 175 64]/255, [255 102 51]/255, [52 175 64]/255, [255 102 51]/255};

for i = 1:4
    nexttile; hold on; grid on;
    plot(t, x(:,i), 'LineWidth',1.5, 'Color', colors{i});
    
    if i <= 2
        ylabel(['x_',num2str(i),' [m]']);      % positions
    else
        ylabel(['x_',num2str(i),' [m/s]']);    % velocities
    end
end

xlabel('t [s]');


%% Phase Portrait x1 vs x3

figure('Name','Phase portrait: Sprung');
hold on; grid on; box on;
plot(x(:,1), x(:,3), 'LineWidth',1.5,'Color',[52 175 64]/255);
xlabel('x_1 [m]'); ylabel('x_3 [m/s]');
title('Sprung: pos vs vel');


%% Phase Portrait x2 vs x4

figure('Name','Phase portrait: Unsprung');
hold on; grid on; box on;
plot(x(:,2), x(:,4), 'LineWidth',1.5,'Color',[255 102 51]/255);
xlabel('x_2 [m]'); ylabel('x_4 [m/s]');
title('Unsprung: pos vs vel');