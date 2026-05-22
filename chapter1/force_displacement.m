clc; clear; close all;

% Parameters
ks = 20e3;           % N/m (spring stiffness)
eps_vec = [0 30];  % nonlinear coefficients to test
x = linspace(0, 0.5, 1000);  % displacement range [m]

figure; hold on; grid on;

for eps = eps_vec
    F = ks * x .* (1 + eps * x.^2);
    plot(x, F, 'LineWidth', 2, 'DisplayName', ['\epsilon = ' num2str(eps)]);
end

xlabel('Displacement x [m]');
ylabel('Spring force F [N]');
title('Nonlinear spring vs Linear');
legend('Location','northwest');