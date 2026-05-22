function [A] = quarterCarStateJacobianFcn(x,u)
% Jacobian of nonlinear quarter-car model w.r.t. states and inputs
x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4);
F  = u(1); zr = u(2);

ms = 577; mu = 50; Ks = 2e4; Ku = 2.5e5; cs = 2e3;
eps_s = 30; % o valore scelto
Ls = 0.4; Lu = 0.34;

delta = x1 - x2;
d_spring = Ks + 3*eps_s*Ks*delta^2;

% Jacobian w.r.t. states
A = zeros(4,4);
A(1,3) = 1;
A(2,4) = 1;
A(3,1) = -d_spring/ms;
A(3,2) =  d_spring/ms;
A(3,3) = -cs/ms;
A(3,4) =  cs/ms;
A(4,1) =  d_spring/mu;
A(4,2) = -(d_spring + Ku)/mu;
A(4,3) =  cs/mu;
A(4,4) = -cs/mu;

% % Jacobian w.r.t. inputs u = [F; z_r]
% B = zeros(4,2);
% B(3,1) = 1/ms;      % F
% B(4,1) = -1/mu;     % F
% B(4,2) = Ku/mu;     % z_r
end
