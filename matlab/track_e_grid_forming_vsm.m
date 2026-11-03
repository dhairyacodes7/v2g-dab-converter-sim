%% =========================================================================
%% Track E: Grid-Forming VSM at DC-DC Converter Level
%% =========================================================================
%% Implements Virtual Synchronous Machine (VSM) emulation in the DAB
%% converter to provide inertial response on the DC bus.
%% =========================================================================

clc; clear; close all;
fprintf('========================================\n');
fprintf(' Track E: Grid-Forming V2G\n');
fprintf(' Virtual Synchronous Machine (VSM)\n');
fprintf('========================================\n\n');

%% ======================== PARAMETERS ====================================
% DAB converter
V_bat       = 350;          % Battery voltage [V]
V_dc_nom    = 400;          % Nominal DC bus voltage [V]
P_rated     = 3300;         % Rated power [W]
L           = 85e-6;        % Leakage inductance [H]
f_sw        = 50e3;
C_dc        = 100e-6;       % DC bus capacitance [F]

% VSM parameters
J_virtual   = 0.5;          % Virtual inertia [kg·m² equivalent]
D_damping   = 50;           % Damping coefficient [W·s/V]
omega_nom   = 2*pi*50;      % Nominal angular frequency [rad/s] (50 Hz grid)
V_dc_droop  = 10;           % Voltage droop: ΔV_dc per unit power imbalance [V/pu]

% Grid-side inverter (simplified)
P_grid_nom  = 3000;         % Grid power demand [W]

% Simulation
dt          = 0.0001;        % 100 µs time step
t_end       = 5;             % 5 seconds
t           = 0:dt:t_end;
N           = length(t);

fprintf('VSM Parameters:\n');
fprintf('  Virtual inertia J:     %.2f kg·m²\n', J_virtual);
fprintf('  Damping D:             %.0f W·s/V\n', D_damping);
fprintf('  DC bus capacitance:    %.0f µF\n', C_dc*1e6);
fprintf('\n');

%% ======================== LOAD PROFILE ==================================
% Scenario: step load changes simulating grid events
P_load = zeros(1, N);
for k = 1:N
    if t(k) < 0.5
        P_load(k) = 2000;          % Normal load
    elseif t(k) < 1.5
        P_load(k) = 3300;          % Load step up (grid peak)
    elseif t(k) < 2.5
        P_load(k) = 1000;          % Load step down
    elseif t(k) < 3.0
        P_load(k) = 0;             % Islanding event (grid disconnected)
    elseif t(k) < 4.0
        P_load(k) = 2500;          % Grid reconnection
    else
        P_load(k) = 2000;          % Return to normal
    end
end

%% ======================== STANDARD DROOP CONTROL ========================
fprintf('Simulating Standard Droop Control...\n');

V_dc_droop_ctrl = zeros(1, N);
P_conv_droop    = zeros(1, N);
omega_droop     = zeros(1, N);
V_dc_droop_ctrl(1) = V_dc_nom;
omega_droop(1) = omega_nom;

for k = 1:N-1
    % Standard droop: V_dc = V_nom - k_droop * (P - P_nom)
    P_error = P_load(k) - P_grid_nom;
    
    % Droop control (fast, no inertia)
    V_dc_ref = V_dc_nom - V_dc_droop * (P_error / P_rated);
    
    % PI voltage control
    Kp_droop = 0.5;
    Ki_droop = 100;
    
    V_err = V_dc_ref - V_dc_droop_ctrl(k);
    P_conv_droop(k) = P_load(k) + Kp_droop * V_err * V_dc_droop_ctrl(k);
    P_conv_droop(k) = max(0, min(P_rated, P_conv_droop(k)));
    
    % DC bus dynamics
    I_conv = P_conv_droop(k) / max(V_dc_droop_ctrl(k), 1);
    I_load = P_load(k) / max(V_dc_droop_ctrl(k), 1);
    
    dV = (I_conv - I_load) / C_dc;
    V_dc_droop_ctrl(k+1) = V_dc_droop_ctrl(k) + dV * dt;
    V_dc_droop_ctrl(k+1) = max(350, min(450, V_dc_droop_ctrl(k+1)));
    
    % Frequency (mapped from DC voltage)
    omega_droop(k+1) = omega_nom * (V_dc_droop_ctrl(k+1) / V_dc_nom);
end

%% ======================== VSM CONTROL ===================================
fprintf('Simulating VSM Grid-Forming Control...\n');

V_dc_vsm   = zeros(1, N);
P_conv_vsm = zeros(1, N);
omega_vsm  = zeros(1, N);
delta_vsm  = zeros(1, N);     % Virtual rotor angle
P_inertia  = zeros(1, N);     % Inertial power contribution

V_dc_vsm(1) = V_dc_nom;
omega_vsm(1) = omega_nom;
delta_vsm(1) = 0;

for k = 1:N-1
    % Power imbalance
    P_error = P_load(k) - P_conv_vsm(k);
    
    % VSM Swing Equation:
    % J * dω/dt = P_ref - P_load - D * (ω - ω_nom)
    P_ref_vsm = P_grid_nom;  % Reference power (from energy management)
    
    domega = (P_ref_vsm - P_load(k) - D_damping * (omega_vsm(k) - omega_nom)) / ...
             (J_virtual * omega_vsm(k));
    omega_vsm(k+1) = omega_vsm(k) + domega * dt;
    
    % Clamp frequency
    omega_vsm(k+1) = max(omega_nom*0.95, min(omega_nom*1.05, omega_vsm(k+1)));
    
    % Update virtual angle
    delta_vsm(k+1) = delta_vsm(k) + (omega_vsm(k+1) - omega_nom) * dt;
    
    % Map virtual frequency to DC bus voltage
    % V_dc = V_nom * (ω / ω_nom)
    V_dc_vsm(k+1) = V_dc_nom * (omega_vsm(k+1) / omega_nom);
    V_dc_vsm(k+1) = max(350, min(450, V_dc_vsm(k+1)));
    
    % Converter power output (follows VSM dynamics)
    P_conv_vsm(k+1) = P_ref_vsm + D_damping * (omega_nom - omega_vsm(k+1)) + ...
                       J_virtual * omega_vsm(k+1) * domega;
    P_conv_vsm(k+1) = max(0, min(P_rated, P_conv_vsm(k+1)));
    
    % Inertial power contribution
    P_inertia(k+1) = J_virtual * omega_vsm(k+1) * domega;
end

fprintf('Simulation complete.\n\n');

%% ======================== METRICS =======================================
fprintf('=== PERFORMANCE COMPARISON ===\n');
fprintf('%-35s %-15s %-15s\n', 'Metric', 'Droop', 'VSM');
fprintf('%-35s %-15s %-15s\n', '------', '-----', '---');

% Rate of Change of Frequency (RoCoF)
rocof_droop = max(abs(diff(omega_droop/omega_nom*50) / dt));
rocof_vsm   = max(abs(diff(omega_vsm/omega_nom*50) / dt));
fprintf('%-35s %-15.2f %-15.2f\n', 'Max RoCoF [Hz/s]', rocof_droop, rocof_vsm);

% Frequency nadir (maximum deviation)
f_droop = omega_droop / (2*pi);
f_vsm   = omega_vsm / (2*pi);
f_nadir_droop = max(abs(f_droop - 50));
f_nadir_vsm   = max(abs(f_vsm - 50));
fprintf('%-35s %-15.3f %-15.3f\n', 'Frequency Nadir [Hz deviation]', f_nadir_droop, f_nadir_vsm);

% Voltage deviation
V_dev_droop = max(abs(V_dc_droop_ctrl - V_dc_nom));
V_dev_vsm   = max(abs(V_dc_vsm - V_dc_nom));
fprintf('%-35s %-15.1f %-15.1f\n', 'Max V_dc Deviation [V]', V_dev_droop, V_dev_vsm);

% Settling time (after load step at t=0.5s)
idx_step = find(t >= 0.5, 1);
idx_settle_droop = find(abs(V_dc_droop_ctrl(idx_step:end) - V_dc_nom) < 2, 1) * dt;
idx_settle_vsm   = find(abs(V_dc_vsm(idx_step:end) - V_dc_nom) < 2, 1) * dt;
fprintf('%-35s %-15.3f %-15.3f\n', 'Settling Time [s]', ...
    idx_settle_droop, idx_settle_vsm);

% Inertial energy contribution
E_inertia = trapz(t, abs(P_inertia));
fprintf('%-35s %-15s %-15.1f\n', 'Inertial Energy [J]', 'N/A', E_inertia);
fprintf('\n');

fprintf('Key Advantage of VSM:\n');
fprintf('  RoCoF reduction: %.0f%%\n', (1 - rocof_vsm/max(rocof_droop,1e-6))*100);
fprintf('  The VSM provides virtual inertia that slows frequency excursions,\n');
fprintf('  giving the grid operator more time to respond to disturbances.\n\n');

%% ======================== PLOTS =========================================
figure('Position', [50, 50, 1400, 900], 'Color', 'w');

% Plot 1: DC Bus Voltage
subplot(2,3,1);
plot(t, V_dc_droop_ctrl, 'b-', 'LineWidth', 2); hold on;
plot(t, V_dc_vsm, 'r-', 'LineWidth', 2);
yline(V_dc_nom, 'k--', 'V_{nom}');
xlabel('Time [s]'); ylabel('V_{DC} [V]');
title('DC Bus Voltage Response');
legend('Droop Control', 'VSM Control', 'Location', 'best');
grid on;

% Plot 2: Virtual Frequency
subplot(2,3,2);
plot(t, omega_droop/(2*pi), 'b-', 'LineWidth', 2); hold on;
plot(t, omega_vsm/(2*pi), 'r-', 'LineWidth', 2);
yline(50, 'k--', '50 Hz');
xlabel('Time [s]'); ylabel('Frequency [Hz]');
title('Virtual Grid Frequency');
legend('Droop', 'VSM', 'Location', 'best');
grid on;

% Plot 3: Load Profile
subplot(2,3,3);
area(t, P_load/1e3, 'FaceColor', [0.8 0.9 1.0], 'FaceAlpha', 0.5); hold on;
plot(t, P_load/1e3, 'k-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Load Power [kW]');
title('Load Profile');
grid on;
% Annotate events
text(0.25, 2.5, 'Normal', 'FontSize', 9);
text(0.75, 3.5, 'Peak', 'FontSize', 9);
text(1.75, 1.5, 'Light', 'FontSize', 9);
text(2.7, 0.3, 'Island', 'FontSize', 9, 'Color', 'r');
text(3.4, 3.0, 'Reconnect', 'FontSize', 9);

% Plot 4: Converter Power Output
subplot(2,3,4);
plot(t, P_conv_droop/1e3, 'b-', 'LineWidth', 2); hold on;
plot(t(1:end), P_conv_vsm/1e3, 'r-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Converter Power [kW]');
title('Converter Output Power');
legend('Droop', 'VSM', 'Location', 'best');
grid on;

% Plot 5: Inertial Power Contribution (VSM only)
subplot(2,3,5);
plot(t, P_inertia/1e3, 'r-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Inertial Power [kW]');
title('VSM Inertial Power Contribution');
grid on;
yline(0, 'k-');

% Plot 6: Rate of Change of Frequency (RoCoF)
subplot(2,3,6);
rocof_droop_t = [0, diff(omega_droop/(2*pi)) / dt];
rocof_vsm_t = [0, diff(omega_vsm/(2*pi)) / dt];
plot(t, rocof_droop_t, 'b-', 'LineWidth', 1.5); hold on;
plot(t, rocof_vsm_t, 'r-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('RoCoF [Hz/s]');
title('Rate of Change of Frequency');
legend('Droop', 'VSM', 'Location', 'best');
grid on;

sgtitle('Track E: Grid-Forming VSM at DC-DC Converter Level', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('=== Track E simulation complete ===\n');
