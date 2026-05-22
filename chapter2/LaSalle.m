% LaSalle Analysis for Quarter-Car Model (Translated Coordinates)

% 1. Symbolic declaration
syms x1 x2 x3 x4 ms mu ks ku cs epsilon real

% 2. Translated system (deviation coordinates)
f3 = (ks*(x2 - x1)*(1 + epsilon*(x2 - x1)^2) + cs*(x4 - x3))/ms;
f4 = (-ks*(x2 - x1)*(1 + epsilon*(x2 - x1)^2) - cs*(x4 - x3) - ku*x2)/mu;

% 3. LaSalle condition: x4 = x3
x4_eq = x3;

% 4. Derivative along E: d(x4-x3)/dt = 0
f_diff = subs(f4 - f3, x4, x4_eq);
eqns = f_diff == 0;

% 5. Solve symbolically for x1, x2 (x3 remains symbolic)
vars = [x1, x2];
sol = solve(eqns, vars, 'Real', true);

% 6. Display symbolic solutions
disp('Symbolic solutions inside E (x3 free):');
x3_sym = x3; % still symbolic
for k = 1:length(sol.x1)
    fprintf('x1 = %s, x2 = %s, x3 = %s, x4 = %s\n', ...
        char(sol.x1(k)), char(sol.x2(k)), char(x3_sym), char(x3_sym));
end