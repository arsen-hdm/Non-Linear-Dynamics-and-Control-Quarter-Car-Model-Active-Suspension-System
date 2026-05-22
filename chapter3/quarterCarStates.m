function dxdt = quarterCarStates(x,u)
% Continuous-time state equations for nonlinear quarter-car model
%
% States:
%   x(1) = z_s     Sprung mass position [m]
%   x(2) = z_u     Unsprung mass position [m]
%   x(3) = dz_s    Sprung mass velocity [m/s]
%   x(4) = dz_u    Unsprung mass velocity [m/s]
%
% Inputs:
%   u(1) = F       Active suspension force [N]
%   u(2) = z_r     Road profile (measured disturbance) [m]

% States
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);

% Inputs
F  = u(1);
zr = u(2);

% Parameters
ms = 577;
mu = 50;
Ks = 2e4;
Ku = 2.5e5;
cs = 2e3;
eps_s = 30;
g = 9.81;
Ls = 0.40;
Lu = 0.34;

global mass_person;
global mass_wheel;

% Initialize state derivative
dxdt = zeros(4,1);

% Nonlinear spring term
delta = x2 - x1;
spring_nl = Ks*delta*(1 + eps_s*delta^2);

% State equations
dxdt(1) = x3;

dxdt(2) = x4;

dxdt(3) = ( spring_nl ...
           + cs*(x4 - x3) ...
           + Ks*Ls ...
           - (ms+mass_person)*g ...
           + F ) / (ms+mass_person);

dxdt(4) = ( -spring_nl ...
           - cs*(x4 - x3) ...
           - Ks*Ls ...
           - (mu+mass_wheel)*g ...
           - F ...
           + Ku*(zr - x2) ...
           + Ku*Lu ) / (mu+mass_wheel);
end
