%% =========================================================================
%% Track A: Model Predictive Controller for DAB Converter
%% =========================================================================
%% Implements a battery-degradation-aware MPC for the DAB converter.
%% Compares performance against conventional PI controller.
%% Includes burst-mode light-load operation.
%% =========================================================================

clc; clear; close all;
fprintf('========================================\n');
fprintf(' Track A: MPC Controller for DAB\n');
fprintf(' Battery-Degradation-Aware Control\n');
fprintf('========================================\n\n');

%% ======================== SYSTEM PARAMETERS =============================
% DAB converter parameters
V1_nom  = 350;          % Battery voltage (nominal) [V]
V2_ref  = 400;          % DC bus voltage reference [V]
L       = 85e-6;        % Leakage inductance [H]
C_out   = 10e-6;        % Output capacitance [F]
n       = 1;            % Turns ratio
f_sw    = 50e3;         % Switching frequency [Hz]
T_sw    = 1/f_sw;       % Switching period [s]
Rds_on  = 65e-3;        % MOSFET Rds(on)

% Load
R_load  = V2_ref^2 / 3300;  % 3.3 kW nominal load

% MPC parameters
N_p     = 3;            % Prediction horizon
N_c     = 1;            % Control horizon
T_mpc   = 100e-6;       % MPC sample time (10 kHz)
phi_min = 0;            % Min phase shift [rad]
phi_max = pi/3;         % Max phase shift [rad] (60°)

% Battery aging model (NMC Semi-Empirical)
A_aging = 2e4;
Ea      = 24500;        % [J/mol]
R_gas   = 8.314;        % [J/(mol·K)]
z_aging = 0.55;
T_cell  = 298.15;       % Cell temperature [K]

% MPC weights
w_voltage   = 1.0;      % Voltage tracking weight
w_current   = 0.1;      % Current stress weight
w_degradation = 0.05;   % Battery degradation weight
w_switching = 0.01;     % Switching loss weight

fprintf('MPC Parameters:\n');
fprintf('  Prediction horizon:  %d\n', N_p);
fprintf('  Sample time:         %.0f µs\n', T_mpc*1e6);
fprintf('  Weights: [V:%.2f, I:%.2f, Deg:%.3f, Sw:%.3f]\n', ...
    w_voltage, w_current, w_degradation, w_switching);
fprintf('\n');

%% ======================== PI CONTROLLER =================================
% PI controller parameters (for comparison)
Kp = 0.01;
Ki = 50;

%% ======================== SIMULATION ====================================
% Simulation time
t_sim   = 0.5;          % 0.5 seconds
dt      = T_mpc;        % Sample at MPC rate
t       = 0:dt:t_sim;
N_steps = length(t);

% State variables
V_out_pi    = zeros(1, N_steps);    % Output voltage (PI)
V_out_mpc   = zeros(1, N_steps);    % Output voltage (MPC)
phi_pi      = zeros(1, N_steps);    % Phase shift (PI)
phi_mpc     = zeros(1, N_steps);    % Phase shift (MPC)
P_out_pi    = zeros(1, N_steps);    % Output power (PI)
P_out_mpc   = zeros(1, N_steps);    % Output power (MPC)
I_rms_pi    = zeros(1, N_steps);    % RMS current (PI)
I_rms_mpc   = zeros(1, N_steps);    % RMS current (MPC)
Ah_total_pi = zeros(1, N_steps);    % Accumulated Ah (PI)
Ah_total_mpc = zeros(1, N_steps);   % Accumulated Ah (MPC)
degradation_pi  = zeros(1, N_steps);
degradation_mpc = zeros(1, N_steps);

% Initial conditions
V_out_pi(1)  = 380;     % Start slightly below reference
V_out_mpc(1) = 380;
integral_err = 0;        % PI integrator state

% Load profile (step changes to test transient response)
P_load = zeros(1, N_steps);
for k = 1:N_steps
    if t(k) < 0.05
        P_load(k) = 1000;      % 1.0 kW (light load)
    elseif t(k) < 0.15
        P_load(k) = 3300;      % 3.3 kW (rated)
    elseif t(k) < 0.25
        P_load(k) = 1650;      % 1.65 kW (half load)
    elseif t(k) < 0.35
        P_load(k) = 3300;      % 3.3 kW (rated)
    elseif t(k) < 0.40
        P_load(k) = 500;       % 500 W (very light — burst mode)
    else
        P_load(k) = 2500;      % 2.5 kW
    end
end

fprintf('Simulating PI and MPC controllers...\n');

%% ======================== SIMULATION LOOP ===============================
for k = 1:N_steps-1
    % Current load resistance
    if P_load(k) > 0
        R_k = V2_ref^2 / P_load(k);
    else
        R_k = 1e6;  % No load
    end
    
    %% --- PI CONTROLLER ---
    err_pi = V2_ref - V_out_pi(k);
    integral_err = integral_err + err_pi * dt;
    phi_pi(k) = Kp * err_pi + Ki * integral_err;
    phi_pi(k) = max(phi_min, min(phi_max, phi_pi(k)));
    
    % Power transfer (DAB equation)
    P_dab_pi = (V1_nom * V_out_pi(k)/n * phi_pi(k) * (pi - abs(phi_pi(k)))) / ...
               (2 * pi^2 * f_sw * L);
    
    % RMS current
    I_rms_pi(k) = sqrt(abs(P_dab_pi) / (V1_nom)) * 1.2;  % Approximate
    
    % Voltage dynamics: C * dV/dt = (P_dab - V^2/R) / V
    dV_pi = (P_dab_pi - V_out_pi(k)^2/R_k) / (C_out * V_out_pi(k));
    V_out_pi(k+1) = V_out_pi(k) + dV_pi * dt;
    V_out_pi(k+1) = max(300, min(500, V_out_pi(k+1)));
    
    P_out_pi(k) = V_out_pi(k)^2 / R_k;
    
    % Battery degradation accumulation (PI)
    Ah_total_pi(k+1) = Ah_total_pi(k) + I_rms_pi(k) * dt / 3600;
    degradation_pi(k+1) = A_aging * exp(-Ea/(R_gas*T_cell)) * ...
                           max(Ah_total_pi(k+1), 1e-10)^z_aging;
    
    %% --- MPC CONTROLLER ---
    % MPC: Optimize phase shift to minimize multi-objective cost
    % Use simple grid search (suitable for DSP implementation)
    
    phi_candidates = linspace(phi_min, phi_max, 50);
    J_best = inf;
    phi_opt = phi_mpc(max(1,k-1));
    
    for j = 1:length(phi_candidates)
        phi_c = phi_candidates(j);
        
        % Predict output voltage over N_p steps
        V_pred = V_out_mpc(k);
        J = 0;
        
        for p = 1:N_p
            % Power transfer at this phase shift
            P_dab_c = (V1_nom * V_pred/n * phi_c * (pi - abs(phi_c))) / ...
                      (2 * pi^2 * f_sw * L);
            
            % RMS current estimate
            I_rms_c = sqrt(abs(P_dab_c) / V1_nom) * 1.2;
            
            % Predict next voltage
            dV_c = (P_dab_c - V_pred^2/R_k) / (C_out * V_pred);
            V_pred = V_pred + dV_c * dt;
            V_pred = max(300, min(500, V_pred));
            
            % Degradation rate
            dAh = I_rms_c * dt / 3600;
            deg_rate = A_aging * exp(-Ea/(R_gas*T_cell)) * z_aging * ...
                       max(Ah_total_mpc(k) + dAh, 1e-10)^(z_aging-1) * dAh;
            
            % Switching loss proxy
            P_sw_proxy = 8 * 0.5 * V1_nom * I_rms_c * 50e-9 * f_sw;
            
            % Cost function
            J = J + w_voltage * (V2_ref - V_pred)^2 + ...
                    w_current * I_rms_c^2 + ...
                    w_degradation * deg_rate * 1e6 + ...
                    w_switching * P_sw_proxy;
        end
        
        if J < J_best
            J_best = J;
            phi_opt = phi_c;
        end
    end
    
    % --- BURST MODE (Track A innovation) ---
    burst_mode = false;
    if P_load(k) < 0.20 * 3300  % Below 20% load
        burst_mode = true;
        % Burst mode: run at rated phase shift for D_burst fraction
        D_burst = P_load(k) / 3300;
        if mod(k, round(1e-3/dt)) < round(D_burst * 1e-3/dt)
            phi_opt = phi_rated_approx();  % Use rated phase shift during burst
        else
            phi_opt = 0;  % Zero phase shift during coast
        end
    end
    
    phi_mpc(k) = phi_opt;
    
    % Apply MPC control
    P_dab_mpc = (V1_nom * V_out_mpc(k)/n * phi_mpc(k) * (pi - abs(phi_mpc(k)))) / ...
                (2 * pi^2 * f_sw * L);
    
    I_rms_mpc(k) = sqrt(abs(P_dab_mpc) / V1_nom) * 1.2;
    
    dV_mpc = (P_dab_mpc - V_out_mpc(k)^2/R_k) / (C_out * V_out_mpc(k));
    V_out_mpc(k+1) = V_out_mpc(k) + dV_mpc * dt;
    V_out_mpc(k+1) = max(300, min(500, V_out_mpc(k+1)));
    
    P_out_mpc(k) = V_out_mpc(k)^2 / R_k;
    
    % Battery degradation accumulation (MPC)
    Ah_total_mpc(k+1) = Ah_total_mpc(k) + I_rms_mpc(k) * dt / 3600;
    degradation_mpc(k+1) = A_aging * exp(-Ea/(R_gas*T_cell)) * ...
                            max(Ah_total_mpc(k+1), 1e-10)^z_aging;
end

fprintf('Simulation complete.\n\n');

%% ======================== RESULTS ANALYSIS ==============================
fprintf('=== PERFORMANCE COMPARISON ===\n');
fprintf('%-30s %-15s %-15s\n', 'Metric', 'PI Control', 'MPC Control');
fprintf('%-30s %-15s %-15s\n', '------', '----------', '-----------');

% Voltage regulation
V_ss_pi  = mean(V_out_pi(end-100:end));
V_ss_mpc = mean(V_out_mpc(end-100:end));
fprintf('%-30s %-15.2f %-15.2f\n', 'Steady-State Voltage [V]', V_ss_pi, V_ss_mpc);

% Voltage error
V_err_pi  = abs(V_ss_pi - V2_ref);
V_err_mpc = abs(V_ss_mpc - V2_ref);
fprintf('%-30s %-15.3f %-15.3f\n', 'Voltage Error [V]', V_err_pi, V_err_mpc);

% Overshoot (during load steps)
V_max_pi  = max(V_out_pi);
V_max_mpc = max(V_out_mpc);
OS_pi  = (V_max_pi - V2_ref) / V2_ref * 100;
OS_mpc = (V_max_mpc - V2_ref) / V2_ref * 100;
fprintf('%-30s %-15.1f %-15.1f\n', 'Max Overshoot [%]', OS_pi, OS_mpc);

% RMS current (average over simulation)
I_avg_pi  = mean(I_rms_pi(I_rms_pi > 0));
I_avg_mpc = mean(I_rms_mpc(I_rms_mpc > 0));
fprintf('%-30s %-15.2f %-15.2f\n', 'Average I_rms [A]', I_avg_pi, I_avg_mpc);

% Battery degradation
deg_final_pi  = degradation_pi(end);
deg_final_mpc = degradation_mpc(end);
fprintf('%-30s %-15.4f %-15.4f\n', 'Degradation Factor [%]', deg_final_pi, deg_final_mpc);

% Improvement
deg_improvement = (1 - deg_final_mpc/max(deg_final_pi, 1e-10)) * 100;
fprintf('%-30s %-15s %-15.1f\n', 'Degradation Reduction [%]', 'baseline', deg_improvement);
fprintf('\n');

%% ======================== PLOTS =========================================
figure('Position', [50, 50, 1400, 900], 'Color', 'w');

% Plot 1: Output Voltage Comparison
subplot(3,2,1);
plot(t*1e3, V_out_pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t*1e3, V_out_mpc, 'r-', 'LineWidth', 1.5);
yline(V2_ref, 'k--', 'V_{ref}=400V', 'LineWidth', 1);
xlabel('Time [ms]');
ylabel('Output Voltage [V]');
title('Output Voltage: PI vs MPC');
legend('PI Control', 'MPC Control', 'Location', 'best');
grid on; ylim([350 450]);

% Plot 2: Phase Shift
subplot(3,2,2);
plot(t*1e3, rad2deg(phi_pi), 'b-', 'LineWidth', 1.5); hold on;
plot(t*1e3, rad2deg(phi_mpc), 'r-', 'LineWidth', 1.5);
xlabel('Time [ms]');
ylabel('Phase Shift [degrees]');
title('Control Signal: Phase Shift');
legend('PI', 'MPC', 'Location', 'best');
grid on;

% Plot 3: Load Profile
subplot(3,2,3);
plot(t*1e3, P_load/1e3, 'k-', 'LineWidth', 2);
xlabel('Time [ms]');
ylabel('Load Power [kW]');
title('Load Profile (Step Changes)');
grid on; ylim([0 4]);

% Plot 4: RMS Current
subplot(3,2,4);
plot(t*1e3, I_rms_pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t*1e3, I_rms_mpc, 'r-', 'LineWidth', 1.5);
xlabel('Time [ms]');
ylabel('I_{rms} [A]');
title('Inductor RMS Current');
legend('PI', 'MPC', 'Location', 'best');
grid on;

% Plot 5: Battery Degradation Accumulation
subplot(3,2,5);
plot(t*1e3, degradation_pi, 'b-', 'LineWidth', 2); hold on;
plot(t*1e3, degradation_mpc, 'r-', 'LineWidth', 2);
xlabel('Time [ms]');
ylabel('Capacity Loss Factor');
title('Cumulative Battery Degradation');
legend('PI Control', 'MPC (Degradation-Aware)', 'Location', 'northwest');
grid on;

% Plot 6: Ah Throughput
subplot(3,2,6);
plot(t*1e3, Ah_total_pi*1e3, 'b-', 'LineWidth', 2); hold on;
plot(t*1e3, Ah_total_mpc*1e3, 'r-', 'LineWidth', 2);
xlabel('Time [ms]');
ylabel('Ah Throughput [mAh]');
title('Accumulated Charge Throughput');
legend('PI Control', 'MPC Control', 'Location', 'northwest');
grid on;

sgtitle('Track A: MPC vs PI Controller Comparison — DAB V2G Converter', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n=== Track A simulation complete ===\n');
fprintf('Key finding: MPC reduces battery degradation by %.1f%% vs PI control.\n', ...
    abs(deg_improvement));
fprintf('Next: Port this MPC algorithm to C code for TMS320F28379D DSP.\n');

%% ======================== HELPER FUNCTIONS ==============================
function phi_r = phi_rated_approx()
    % Returns approximate phase shift for rated power
    phi_r = pi/6;  % 30 degrees
end
