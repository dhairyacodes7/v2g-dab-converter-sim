%% =========================================================================
%% Track C: Climate-Robust Thermal Derating Controller
%% =========================================================================
%% Simulates the DAB converter under elevated ambient temperatures (25-55°C)
%% and implements a thermal-aware power derating algorithm.
%% =========================================================================

clc; clear; close all;
fprintf('========================================\n');
fprintf(' Track C: Climate-Robust V2G Converter\n');
fprintf(' Thermal-Aware Power Derating\n');
fprintf('========================================\n\n');

%% ======================== PARAMETERS ====================================
% Converter
P_rated     = 3300;         % Rated power [W]
V_bat       = 350;          % Battery voltage [V]
V_dc        = 400;          % DC bus voltage [V]
f_sw        = 50e3;         % Switching frequency [Hz]
Rds_on      = 65e-3;        % SiC MOSFET Rds(on) [Ohms]

% Thermal model (4th-order Cauer network per MOSFET)
% Junction → Case → Heatsink → Ambient
R_th_jc     = 0.70;         % Junction-to-case [°C/W]
R_th_cs     = 0.50;         % Case-to-sink (TIM) [°C/W]
R_th_sa     = 2.00;         % Sink-to-ambient (heatsink+fan) [°C/W]
R_th_total  = R_th_jc + R_th_cs + R_th_sa;

% Thermal capacitances (for transient modeling)
C_th_j      = 0.005;        % Junction thermal capacitance [J/°C]
C_th_c      = 0.50;         % Case thermal capacitance [J/°C]
C_th_s      = 5.00;         % Heatsink thermal capacitance [J/°C]

% Limits
T_j_max     = 150;          % Absolute max junction temp [°C]
T_j_derate_start = 125;     % Start derating at this temp [°C]
T_j_shutdown = 145;         % Emergency shutdown [°C]

% Simulation
dt          = 0.001;         % Time step [s]. Euler needs dt < 2*R_th_jc*C_th_j = 7 ms
t_end       = 300;           % 5 minutes simulation [s]
t           = 0:dt:t_end;
N           = length(t);

fprintf('Thermal resistance (total): %.2f °C/W\n', R_th_total);
fprintf('Derating starts at:        %d °C\n', T_j_derate_start);
fprintf('Shutdown at:               %d °C\n', T_j_shutdown);
fprintf('\n');

%% ======================== POWER LOSS MODEL ==============================
% Power dissipation as function of output power
calc_losses = @(P_out) (8 * Rds_on * (P_out/(V_bat*0.9))^2 / 4) + ... % Conduction
                       5 + ...                                          % Core loss
                       4 + ...                                          % Gate drive
                       3;                                               % Inductor

%% ======================== DERATING FUNCTION =============================
% Linear derating between T_derate_start and T_j_max
derate = @(T_j) min(1.0, max(0.0, (T_j_max - T_j) / (T_j_max - T_j_derate_start)));

%% ======================== SIMULATION: MULTIPLE AMBIENTS =================
T_ambient_list = [25, 35, 45, 50, 55];  % Test ambients
colors = {'b', 'g', [0.8 0.5 0], 'r', 'm'};

% Storage
T_j_all     = zeros(length(T_ambient_list), N);
P_out_all   = zeros(length(T_ambient_list), N);
P_limit_all = zeros(length(T_ambient_list), N);

for ai = 1:length(T_ambient_list)
    T_amb = T_ambient_list(ai);
    
    % State variables (3-node thermal model)
    T_j = T_amb;        % Junction temperature
    T_c = T_amb;        % Case temperature
    T_s = T_amb;        % Heatsink temperature
    
    P_out = P_rated;    % Start at rated power
    
    fprintf('--- Simulating T_ambient = %d°C ---\n', T_amb);
    
    for k = 1:N
        % Load profile: constant rated power demand
        P_demand = P_rated;
        
        % Apply derating
        k_derate = derate(T_j);
        P_limit = P_rated * k_derate;
        P_out = min(P_demand, P_limit);
        
        % Emergency shutdown
        if T_j >= T_j_shutdown
            P_out = 0;
        end
        
        % Power dissipation (per MOSFET, then total for 8)
        P_loss = calc_losses(P_out);
        P_mosfet = P_loss / 8;  % Per MOSFET
        
        % Thermal dynamics (3-node Cauer model)
        % dT_j/dt = (P_mosfet - (T_j-T_c)/R_th_jc) / C_th_j
        % dT_c/dt = ((T_j-T_c)/R_th_jc - (T_c-T_s)/R_th_cs) / C_th_c
        % dT_s/dt = ((T_c-T_s)/R_th_cs - (T_s-T_amb)/R_th_sa) / C_th_s
        
        dTj = (P_mosfet - (T_j - T_c)/R_th_jc) / C_th_j;
        dTc = ((T_j - T_c)/R_th_jc - (T_c - T_s)/R_th_cs) / C_th_c;
        dTs = ((T_c - T_s)/R_th_cs - (T_s - T_amb)/R_th_sa) / C_th_s;
        
        T_j = T_j + dTj * dt;
        T_c = T_c + dTc * dt;
        T_s = T_s + dTs * dt;
        
        % Store
        T_j_all(ai, k) = T_j;
        P_out_all(ai, k) = P_out;
        P_limit_all(ai, k) = P_limit;
    end
    
    % Report steady state
    T_j_ss = T_j_all(ai, end);
    P_ss   = P_out_all(ai, end);
    fprintf('  Steady-state T_j:    %.1f°C\n', T_j_ss);
    fprintf('  Steady-state Power:  %.0f W (%.0f%% of rated)\n', P_ss, P_ss/P_rated*100);
    if T_j_ss >= T_j_shutdown
        fprintf('  STATUS: ⛔ SHUTDOWN\n');
    elseif T_j_ss >= T_j_derate_start
        fprintf('  STATUS: ⚠️  DERATED to %.0f%%\n', P_ss/P_rated*100);
    else
        fprintf('  STATUS: ✅ FULL POWER\n');
    end
    fprintf('\n');
end

%% ======================== TRANSIENT SCENARIO ============================
% Simulate a realistic V2G scenario in tropical climate (45°C)
fprintf('=== Tropical V2G Scenario (T_amb=45°C) ===\n');

T_amb_tropic = 45;
T_j_tropic = T_amb_tropic;
T_c_tropic = T_amb_tropic;
T_s_tropic = T_amb_tropic;

t_scen = 0:dt:600;  % 10 minutes
N_scen = length(t_scen);
T_j_scen = zeros(1, N_scen);
P_scen = zeros(1, N_scen);
mode_scen = zeros(1, N_scen);  % 0=G2V, 1=V2G

for k = 1:N_scen
    % Scenario: alternate between G2V and V2G every 2 minutes
    if mod(t_scen(k), 240) < 120
        P_demand = 3300;        % G2V: full charging
        mode_scen(k) = 0;
    else
        P_demand = -2500;       % V2G: discharge 2.5kW to grid
        mode_scen(k) = 1;
    end
    
    k_derate = derate(T_j_tropic);
    P_limit = P_rated * k_derate;
    P_actual = sign(P_demand) * min(abs(P_demand), P_limit);
    
    if T_j_tropic >= T_j_shutdown
        P_actual = 0;
    end
    
    P_loss = calc_losses(abs(P_actual));
    P_mosfet = P_loss / 8;
    
    dTj = (P_mosfet - (T_j_tropic - T_c_tropic)/R_th_jc) / C_th_j;
    dTc = ((T_j_tropic - T_c_tropic)/R_th_jc - (T_c_tropic - T_s_tropic)/R_th_cs) / C_th_c;
    dTs = ((T_c_tropic - T_s_tropic)/R_th_cs - (T_s_tropic - T_amb_tropic)/R_th_sa) / C_th_s;
    
    T_j_tropic = T_j_tropic + dTj * dt;
    T_c_tropic = T_c_tropic + dTc * dt;
    T_s_tropic = T_s_tropic + dTs * dt;
    
    T_j_scen(k) = T_j_tropic;
    P_scen(k) = P_actual;
end

fprintf('Peak T_j during V2G cycling: %.1f°C\n', max(T_j_scen));
if any(T_j_scen > T_j_derate_start)
    derate_text = 'Yes';
else
    derate_text = 'No';
end
fprintf('Thermal derating triggered: %s\n', derate_text);
fprintf('\n');

%% ======================== PLOTS =========================================
figure('Position', [50, 50, 1400, 900], 'Color', 'w');

% Plot 1: Junction Temperature vs Time (all ambients)
subplot(2,3,1);
for ai = 1:length(T_ambient_list)
    plot(t, T_j_all(ai,:), 'Color', colors{ai}, 'LineWidth', 2); hold on;
end
yline(T_j_derate_start, 'r--', 'Derating Start', 'LineWidth', 1.5);
yline(T_j_shutdown, 'r-', 'Shutdown', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('T_{junction} [°C]');
title('Junction Temperature Rise');
legend(arrayfun(@(x) sprintf('T_{amb}=%d°C', x), T_ambient_list, 'UniformOutput', false), ...
    'Location', 'southeast');
grid on;

% Plot 2: Output Power vs Time (all ambients)
subplot(2,3,2);
for ai = 1:length(T_ambient_list)
    plot(t, P_out_all(ai,:)/1e3, 'Color', colors{ai}, 'LineWidth', 2); hold on;
end
yline(P_rated/1e3, 'k--', 'Rated', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Output Power [kW]');
title('Power Output (with Derating)');
legend(arrayfun(@(x) sprintf('%d°C', x), T_ambient_list, 'UniformOutput', false), ...
    'Location', 'east');
grid on; ylim([0 4]);

% Plot 3: Steady-State Derating Curve
subplot(2,3,3);
T_amb_sweep = 20:1:60;
P_ss_sweep = zeros(size(T_amb_sweep));
T_j_ss_sweep = zeros(size(T_amb_sweep));
for i = 1:length(T_amb_sweep)
    % Iterate to find steady-state
    T_j_iter = T_amb_sweep(i);
    for iter = 1:100
        k_d = derate(T_j_iter);
        P_out_iter = P_rated * k_d;
        P_loss_iter = calc_losses(P_out_iter);
        T_j_iter = T_amb_sweep(i) + (P_loss_iter/8) * R_th_total;
    end
    P_ss_sweep(i) = P_out_iter;
    T_j_ss_sweep(i) = T_j_iter;
end
yyaxis left;
plot(T_amb_sweep, P_ss_sweep/1e3, 'b-', 'LineWidth', 2);
ylabel('Max Power [kW]');
yyaxis right;
plot(T_amb_sweep, T_j_ss_sweep, 'r--', 'LineWidth', 2);
ylabel('T_{junction} [°C]');
xlabel('Ambient Temperature [°C]');
title('Derating Curve (Steady State)');
grid on;
xline(25, 'g:', 'Lab'); xline(45, 'm:', 'India'); xline(55, 'r:', 'Extreme');

% Plot 4: Tropical V2G Scenario - Temperature
subplot(2,3,4);
plot(t_scen/60, T_j_scen, 'r-', 'LineWidth', 2);
yline(T_j_derate_start, 'r--', 'Derating', 'LineWidth', 1);
xlabel('Time [min]'); ylabel('T_{junction} [°C]');
title('Tropical Scenario (45°C Ambient)');
grid on;

% Plot 5: Tropical V2G Scenario - Power
subplot(2,3,5);
area(t_scen/60, P_scen/1e3, 'FaceColor', [0.3 0.6 0.9], 'FaceAlpha', 0.6);
xlabel('Time [min]'); ylabel('Power [kW]');
title('V2G Power Profile (G2V↔V2G)');
grid on;
text(1, 2.5, 'G2V (Charging)', 'FontSize', 10);
text(3, -1.5, 'V2G (Discharging)', 'FontSize', 10);

% Plot 6: Derating Factor vs T_junction
subplot(2,3,6);
T_j_range = 80:1:155;
k_range = arrayfun(derate, T_j_range);
area(T_j_range, k_range*100, 'FaceColor', [0.2 0.7 0.3], 'FaceAlpha', 0.4);
xlabel('Junction Temperature [°C]'); ylabel('Power Limit [% of rated]');
title('Derating Function');
grid on;
xline(T_j_derate_start, 'r--', 'Start'); xline(T_j_shutdown, 'r-', 'Shutdown');

sgtitle('Track C: Climate-Robust V2G Converter with Thermal Derating', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('=== Track C simulation complete ===\n');
