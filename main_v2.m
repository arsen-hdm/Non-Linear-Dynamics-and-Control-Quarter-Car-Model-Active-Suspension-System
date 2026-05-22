%% 
%% ================= USER SELECTION MENU =================

clear; clc; close all;

disp('==============================================')
disp('      ACTIVE SUSPENSION SIMULATION MENU       ')
disp('==============================================')

%% 1) Type of Simulation
while true
    disp(' ')
    disp('Select simulation type:')
    disp('1 -> Open Loop (Nonlinear)')
    disp('2 -> Linearized System (LQR)')
    disp('3 -> Nonlinear MPC')

    simType = input('Enter choice (1-3): ');

    if ismember(simType,[1 2 3])
        break
    else
        disp('Invalid choice. Please enter 1, 2 or 3.')
    end
end

%% 2) Type of Road
while true
    disp(' ')
    disp('Select road profile:')
    disp('1 -> Road Bump')
    disp('2 -> Off Road')

    roadType = input('Enter choice (1-2): ');

    if ismember(roadType,[1 2])
        break
    else
        disp('Invalid choice. Please enter 1 or 2.')
    end
end

%% 3) Parametric Uncertainty
while true
    disp(' ')
    disp('Enable parametric uncertainty?')
    disp('0 -> No')
    disp('1 -> Yes')

    uncertaintyFlag = input('Enter choice (0-1): ');

    if ismember(uncertaintyFlag,[0 1])
        break
    else
        disp('Invalid choice. Please enter 0 or 1.')
    end
end

%% 4) Disturbance Uncertainty
while true
    disp(' ')
    disp('Enable disturbance uncertainty?')
    disp('0 -> No')
    disp('1 -> Yes')

    disturbanceFlag = input('Enter choice (0-1): ');

    if ismember(disturbanceFlag,[0 1])
        break
    else
        disp('Invalid choice. Please enter 0 or 1.')
    end
end

disp(' ')
disp('Simulation starting...')
disp('==============================================')
pause(1)

%% Data Initialization...

%% Apply parametric uncertainty if selected

global mass_person;
global mass_wheel;

if uncertaintyFlag == 1
    mass_person = 70;  
    mass_wheel  = -7;
else
    mass_person = 0;
    mass_wheel  = 0;
end

%% Apply Disturbance if selected

if disturbanceFlag == 1
    uncertainty = 0.0003;
else
    uncertainty = 0;
end

%% Chosing the right Final Time

if roadType == 1
        tf = 5;
        disp('Road profile: BUMP')
else
        tf = 7;
        disp('Road profile: OFF-ROAD')
end

ts = 0.01;

ms = 577;
mu = 50;
ks = 20e3;
ku = 2.5e5;
Ls = 0.40;
Lu = 0.34;
cs = 2e3;
g  = 9.81;
epsilon = 30;

x_eq = [0.408, 0.315, 0, 0];                                               % equilibrium point
x0 = [0.001, -0.001, 0.001, -0.001];                                       % difference from the eq. point

%% OPEN LOOP SIMULATION (NONLINEAR PASSIVE)

if simType == 1
    
    disp('Running Open Loop Nonlinear Simulation...')
    
    sosp = 0;            % passive
    sat  = 15000;        % actuator saturation (not active here)
    numR = 1e5*[1 2 1];
    denR = [100 20 1];
    
    % Run Simulink model
    sim('Open_Loop');
    
    % Extract signals
    toutd0  = tout;
    z_r     = z_road;
    z_s     = z_sprung;
    z_u     = z_unsprung;
    z_s_dot = z_sprung_dot;
    z_u_dot = z_unsprung_dot;
    Force   = force;
    z_s_err = zs_error;
    z_u_err = zu_error;
    x1      = s;
    x1_ddot = ddot_s;
    x2      = u_r;

    v = 2.8;     % vehicle speed [m/s]
    x = v * toutd0;
    
    %% Discretize road for MPC compatibility
    % 
    % %t_grid = 0:ts:toutd0(end);
    % z_r_disc = interp1(toutd0, z_r, t_grid, 'linear', 'extrap');
    % 
    % if roadType == 1
    %     save('road_profile_bump.mat', 'z_r_disc', 'ts', 't_grid');
    % else
    %     save('off_road_profile.mat', 'z_r_disc', 'ts', 't_grid');
    % end

    %% ======== PLOTS ========
    
    figure
    sgtitle('Inputs of the system')
    subplot(211)
    plot(toutd0, z_r, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('z_r [m]')
    axis tight
    
    subplot(212)
    plot(toutd0, Force, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('F(t) [N]')
    axis tight
    
    figure
    sgtitle('Positions of the masses')

    plot(toutd0, z_s, 'LineWidth',2,'Color',[52 175 64]/255)
    hold on
    plot(toutd0, z_u, 'LineWidth',2,'Color',[255 102 51]/255)

    grid on
    xlabel('t [s]')
    ylabel('Position [m]')
    legend('z_s','z_u','Location','best')

    axis tight

    figure
    sgtitle('Velocities of the masses')
    subplot(211)
    plot(toutd0, z_s_dot, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('z_s_dot [m/s]')
    axis tight
    
    subplot(212)
    plot(toutd0, z_u_dot, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('z_u_dot [m/s]')
    axis tight
    
    
    figure
    sgtitle('Errors between desired positions and real ones')
    subplot(211)
    plot(toutd0, z_s_err, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('e_s [m]')
    axis tight
    
    subplot(212)
    plot(toutd0, z_u_err, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('e_u [m]')
    axis tight

    %% ================= PERFORMANCE INDEX DISPLAY =================

    disp(' ')
    disp('==============================================')
    disp('         PERFORMANCE INDICES (RMS)           ')
    disp('==============================================')

    % RMS globali
    RMS_x1      = rms(x1);
    RMS_x1_ddot = rms(x1_ddot);
    RMS_x2      = rms(x2);

    disp(' ')
    disp('==============================================')
    disp('              RMS PERFORMANCE                ')
    disp('==============================================')

    fprintf('RMS Sprung Mass Position (x1)        = %.6f m\n', RMS_x1);
    fprintf('RMS Sprung Mass Acceleration (x1_ddot)= %.6f m/s^2\n', RMS_x1_ddot);
    fprintf('RMS Tire Deflection (x2)              = %.6f m\n', RMS_x2);

    disp('==============================================')

end

%% ================= LQR SIMULATION (LINEARIZED) =================

if simType == 2
    
    disp('Running Linearized LQR Simulation...')
    
    load('linearized_model.mat')
    
    B = [0 0; 0 0; 1/ms 0; -1/ms ku/mu];
    C = [1 0 0 0];
    
    % Control input = active force
    Bc = B(:,1);
    
    %% ===== Augmented system (integral action) =====
    
    Aaug = [A, zeros(4,1); -C, 0];
    Baug = [Bc; 0];
    
    %% ===== Bryson tuning =====
    
    x1_max = 0.001;
    x2_max = 1e3;
    x3_max = 0.08;
    x4_max = 1e3;
    z_max  = 0.005;
    
    Q = diag(1./[x1_max^2, x2_max^2, x3_max^2, x4_max^2, z_max^2]);
    
    Q(1,1) = 20 * Q(1,1);
    Q(3,3) = 20 * Q(3,3);
    
    umax = 15000;
    R = 1500 * (1/(umax^2));
    
    %% ===== LQR gain =====
    
    [Klq,~,~] = lqr(Aaug, Baug, Q, R);
    
    Kx = Klq(1:4);
    Ki = Klq(5);
    
    %% ===== Stability check =====
    
    A_cl = Aaug - Baug*Klq;
    lambda_cl = eig(A_cl);
    
    disp('Closed-loop eigenvalues:')
    disp(lambda_cl)
    
    if all(real(lambda_cl) < 0)
        disp('Closed-loop system is stable')
    else
        disp('Closed-loop system is NOT stable')
    end
    
    %% ===== Simulation setup =====
    
    sosp = 1;        % active
    sat  = 6000;
    numR = 1e5*[1 2 1];
    denR = [100 20 1];
    
    sim('Linearized');
    
    %% ===== Extract signals =====
    
    toutd0  = tout;
    z_r     = z_road;
    z_s     = z_sprung;
    z_u     = z_unsprung;
    z_s_dot = z_sprung_dot;
    z_u_dot = z_unsprung_dot;
    Force   = force;
    z_s_err = zs_error;
    z_u_err = zu_error;
    noise = noise_sl;
    
    % Performance signals
    x1      = s;
    x1_ddot = ddot_s;
    x2      = u_r;
    
    v = 2.8;
    x = v * toutd0;

    % t_grid = 0:ts:toutd0(end);
    % noise_disc = interp1(toutd0, noise, t_grid, 'linear', 'extrap');
    % 
    % if tf == 5
    %     save('noise_signal_bump.mat', 'noise_disc', 'ts', 't_grid');
    % else
    %     save('noise_signal_off_road.mat', 'noise_disc', 'ts', 't_grid');
    % end


    %% ===== PLOTS =====
    
    figure
    sgtitle('Inputs of the system - LQR')
    subplot(211)
    plot(toutd0, z_r, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('z_r [m]')
    axis tight
    
    subplot(212)
    plot(toutd0, Force, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('F(t) [N]')
    axis tight

    figure
    sgtitle('Positions of the masses')

    plot(toutd0, z_s, 'LineWidth',2,'Color',[52 175 64]/255)
    hold on
    plot(toutd0, z_u, 'LineWidth',2,'Color',[255 102 51]/255)

    grid on
    xlabel('t [s]')
    ylabel('Position [m]')
    legend('z_s','z_u','Location','best')

    axis tight
    
    figure
    sgtitle('Velocities of the masses')
    subplot(211)
    plot(toutd0, z_s_dot, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('z_s_dot [m/s]')
    axis tight
    
    subplot(212)
    plot(toutd0, z_u_dot, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('z_u_dot [m/s]')
    axis tight
    
    
    figure
    sgtitle('Errors between desired positions and real ones')
    subplot(211)
    plot(toutd0, z_s_err, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('e_s [m]')
    axis tight
    
    subplot(212)
    plot(toutd0, z_u_err, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('e_u [m]')
    axis tight
    
    
    %% ================= RMS PERFORMANCE =================
    
    disp(' ')
    disp('==============================================')
    disp('         LQR PERFORMANCE INDICES (RMS)       ')
    disp('==============================================')
    
    RMS_x1      = rms(x1);
    RMS_x1_ddot = rms(x1_ddot);
    RMS_x2      = rms(x2);
    
    fprintf('RMS Sprung Mass Position (x1)        = %.6f m\n', RMS_x1);
    fprintf('RMS Sprung Mass Acceleration (x1_ddot)= %.6f m/s^2\n', RMS_x1_ddot);
    fprintf('RMS Tire Deflection (x2)              = %.6f m\n', RMS_x2);
    
    disp('==============================================')

end

%% ================= NMPC SIMULATION =================

if simType == 3
    
    disp('Running Nonlinear MPC Simulation...')
    
    %% ===== Controller Definition =====
    
    nx = 4;
    ny = 1;
    
    mvIndex = 1;
    mdIndex = 2;
    
    nlobj = nlmpc(nx, ny, 'MV', mvIndex, 'MD', mdIndex);
    nlobj.Dimensions
    
    nlobj.Ts = ts;
    nlobj.Model.IsContinuousTime = true;
    nlobj.Model.StateFcn = 'quarterCarStates';
    
    % OutputFcn
    nlobj.Model.OutputFcn = @(x,u) [
        x(1)
    ];
    
    % x1 = z_s car position
    nlobj.States(1) = struct('Min', 0.408-0.002, 'Max', 0.408+0.002, ...
                             'Name', 'z_s', 'Units', 'm', 'ScaleFactor', 0.002);

    % x2 = z_u wheel position
    nlobj.States(2) = struct('Min', 0.315-1, 'Max', 0.315+1, ...
                             'Name', 'z_u', 'Units', 'm', 'ScaleFactor', 1);

    % x3 = dz_s car's velocity
    nlobj.States(3) = struct('Min', -3, 'Max', 3, ...
                             'Name', 'dz_s', 'Units', 'm/s', 'ScaleFactor', 3);

    % x4 = dz_u wheel's velocity
    nlobj.States(4) = struct('Min', -10, 'Max', 10, ...
                             'Name', 'dz_u', 'Units', 'm/s', 'ScaleFactor', 10);

    % OV1 = z_s car position
    nlobj.OV(1) = struct('Min', 0.408-0.002, 'Max', 0.408+0.002, ...
                         'MinECR',0.1,'MaxECR',1, ...
                         'Name','z_s','Units','m','ScaleFactor',0.002);

    % % OV1 = z_s car position
    % nlobj.OV(1) = struct('Min', 0.413-0.003, 'Max', 0.413+0.003, ...
    %                      'MinECR',0.1,'MaxECR',1, ...
    %                      'Name','z_s','Units','m','ScaleFactor',0.003);


    Fdot_max = 25000;      % N/s realistic
    Rate_limit = Fdot_max *ts;

    nlobj.MV(1) = struct( ...
        'Min', -15000, 'Max', 15000, ...
        'MinECR', 0, 'MaxECR', 0, ...
        'RateMin', -Rate_limit, ...
        'RateMax',  Rate_limit, ...
        'RateMinECR', 0, 'RateMaxECR', 0, ...
        'Name', 'F_attiva', 'Units', 'N', ...
        'ScaleFactor', 15000 ...
    );


    %% ===== Measured Disturbance (Road) =====

    if roadType == 1      % BUMP
    
        nlobj.MD(1) = struct( ...
            'Name', 'z_R', ...
            'Units', 'm', ...
            'ScaleFactor', 0.05 ...
        );    
    elseif roadType == 2  % OFF-ROAD
    
        nlobj.MD(1) = struct( ...
           'Name', 'z_R', ...
            'Units', 'm', ...
            'ScaleFactor', 0.1 ...
        ); 

    end


    % OutputVariables weight
    nlobj.Weights.OutputVariables = 2;  % [z_s]

    % MV → active force
    nlobj.Weights.ManipulatedVariables = 0.01;
    nlobj.Weights.ManipulatedVariablesRate = 0.1;

    % Slack variables
    nlobj.Weights.ECR = 1e2;

    nlobj.Optimization.MVInterpolationOrder = 1;

    nlobj.Optimization.UseSuboptimalSolution = true;

    %nlobj.Jacobians.StateFcn = [];
    %nlobj.Jacobians.OutputFcn = [];
    % MATLAB computes numerically, hardcoded woul be faster but gives some problems

    %maybe integrate passivity
    
    %% ===== Initial Conditions =====
    
    x = x_eq'+x0';
    lastmv = 0;
    yref = 0.408;
    % yref = 0.413;
    u0 = [0; 0];
    
    validateFcns(nlobj,x_eq',u0(1),u0(2));
    
    %% ===== Load road profile =====
    
    if roadType == 1
        load('road_profile_bump.mat');
        if disturbanceFlag == 1
            load('noise_signal_bump.mat');
            disp('noise uploaded')
        end
    else
        load('off_road_profile.mat');
        if disturbanceFlag == 1
            load('noise_signal_off_road.mat');
            disp('noise uploaded')
        end
    end
    
    Nsteps = tf/ts;
    
    xHistory = x;
    mvHistory = lastmv;
    
    %% ===== Simulation Loop =====
    
    for k = 1:Nsteps

        % z_r at the current instant
        md = z_r_disc(k);
        if disturbanceFlag == 1
            md = md + noise_disc(k);
        end
        % computing optimal input
        mv = nlmpcmove(nlobj, x, lastmv, yref, md);
        u_full = [mv; md];

        if disturbanceFlag == 1
            md = md - noise_disc(k);
        end
        % simulating the system
        dyn = @(t,x) quarterCarStates(x, [mv; md]);
        [~, x_temp] = ode45(dyn, [0 ts], x);

        % take the last value
        x = x_temp(end,:)';

        % save the data
        xHistory = [xHistory x];
        mvHistory = [mvHistory mv];

        lastmv = mv;
    end
    
    %% ===== Prepare Data =====
    
    t = 0:ts:(length(xHistory)-1)*ts;
    
    z_s = xHistory(1,:);
    z_u = xHistory(2,:);
    dz_s = xHistory(3,:);
    dz_u = xHistory(4,:);
    
    Force = mvHistory;
    
    z_r = z_r_disc(1:length(t));
    if disturbanceFlag == 1
        z_r = z_r + noise_disc(1:length(t));
    end
    
    z_s_err = z_s - yref;
    
    z_u_eq = 0.315;
    z_u_err = z_r + z_u_eq - z_u;
    
    %% ===== Performance Signals (ALLINEATI AGLI ALTRI) =====
    
    x1 = z_s;
    x1_ddot = gradient(dz_s, ts);   % accelerazione numerica
    x2 = z_u - z_r;                 % deflessione ruota
    
    %% ==================== NMPC PLOTS ====================

    %% inputs

    figure
    sgtitle('Inputs of the system')

    subplot(211)
    plot(t, z_r, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('z_r [m]')
    axis tight

    subplot(212)
    plot(t, Force, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('F(t) [N]')
    axis tight


    %% positions

    figure
    sgtitle('Positions of the masses')

    plot(t, z_s, 'LineWidth',2,'Color',[52 175 64]/255)
    hold on
    plot(t, z_u, 'LineWidth',2,'Color',[255 102 51]/255)

    grid on
    xlabel('t [s]')
    ylabel('Position [m]')
    legend('z_s','z_u','Location','best')
    axis tight


    %% velocities

    figure
    sgtitle('Velocities of the masses')

    subplot(211)
    plot(t, dz_s, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('dz_s [m/s]')
    axis tight

    subplot(212)
    plot(t, dz_u, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('dz_u [m/s]')
    axis tight


    %% errors

    figure
    sgtitle('Errors between desired positions and real ones')

    subplot(211)
    plot(t, z_s_err, 'LineWidth',2,'Color',[52 175 64]/255)
    grid on
    xlabel('t [s]')
    ylabel('e_s [m]')
    axis tight

    subplot(212)
    plot(t, z_u_err, 'LineWidth',2,'Color',[255 102 51]/255)
    grid on
    xlabel('t [s]')
    ylabel('e_u [m]')
    axis tight    
    
    %% ================= RMS PERFORMANCE =================
    
    disp(' ')
    disp('==============================================')
    disp('        NLMPC PERFORMANCE INDICES (RMS)       ')
    disp('==============================================')
    
    RMS_x1      = rms(x_eq(1)-x1);
    RMS_x1_ddot = rms(x1_ddot);
    RMS_x2      = rms(x2);
    
    fprintf('RMS Sprung Mass Position (x1)        = %.6f m\n', RMS_x1);
    fprintf('RMS Sprung Mass Acceleration (x1_ddot)= %.6f m/s^2\n', RMS_x1_ddot);
    fprintf('RMS Tire Deflection (x2)              = %.6f m\n', RMS_x2);
    
    disp('==============================================')
    
end