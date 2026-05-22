clear all; clc; close all;

syms x1 x2 x3 x4 ms mu ks ku cs eps Ls Lu g real

X = [x1; x2; x3; x4];

% Defining f symbolically

f = [ x3;
      x4;
      (ks*(x2-x1) + ks*eps*(x2-x1)^3 + cs*(x4-x3) + ks*Ls - ms*g)/ms;
      (-ks*(x2-x1) - ks*eps*(x2-x1)^3 - cs*(x4-x3) - ks*Ls - mu*g - ku*x2 + ku*Lu)/mu ];

% Symbolic Jacobian

J = jacobian(f, X);

% Visualizing it...

disp('Symbolic Jacobian:')
disp(J)
