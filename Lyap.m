%% 1. Symbolic declaration
syms x1_tilde x2_tilde x3_tilde x4_tilde real
syms ms mu ks ku cs epsilon real

% 2. Lyapunov function with traslated coordinates
delta = x2_tilde - x1_tilde;
V = 1/2*ms*x3_tilde^2 + 1/2*mu*x4_tilde^2 ...
    + 1/2*ks*delta^2 + 1/4*ks*epsilon*delta^4 ...
    + 1/2*ku*x2_tilde^2;

% 3. System in traslated coordinates
f1 = x3_tilde;
f2 = x4_tilde;
f3 = (ks*(x2_tilde - x1_tilde)*(1 + epsilon*(x2_tilde - x1_tilde)^2) ...
      + cs*(x4_tilde - x3_tilde))/ms;
f4 = (-ks*(x2_tilde - x1_tilde)*(1 + epsilon*(x2_tilde - x1_tilde)^2) ...
      - cs*(x4_tilde - x3_tilde) - ku*x2_tilde)/mu;

f = [f1; f2; f3; f4];
X_tilde = [x1_tilde; x2_tilde; x3_tilde; x4_tilde];

% 4. V dot simbolically computed
Vdot = simplify(jacobian(V,X_tilde)*f)