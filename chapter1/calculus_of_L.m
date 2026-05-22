clear all; clc; close all;

%% Quarter-car: computation of Lu and Ls from z_bar (static conditions)

% Parameters
ms = 577;        % kg  (sprung mass)
mu = 50;         % kg  (unsprung mass)
ks = 20e3;       % N/m (stiffness of the spring)
ku = 2.5e5;      % N/m (stiffness of the tire)
g  = 9.81;       % m/s^2
epsilon = 30;

% imposing static conditions
zu_bar = 0.32;   % m (static height of the tire)
zs_bar = 0.41;   % m (static height of the car)

% % unstretched lenght of the wheel
% Lu = zu_bar + (ms + mu)*g/ku;
% 
% % unstretched lenght of the car's spring
% Ls = zs_bar - Lu + (ms/ks + (ms+mu)/ku)*g;

% unstretched lenght of the car's spring considering the non linear terms
Ls = (ms*g/ks)-(zu_bar-zs_bar)-epsilon*(zu_bar-zs_bar)^3;

% unstretched lenght of the wheel considering the non linear terms
Lu = (ks*(zu_bar-zs_bar)+(ks*epsilon*(zu_bar-zs_bar)^3)+ks*Ls+ku*zu_bar+mu*g)/ku;
