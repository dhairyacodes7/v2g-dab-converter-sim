%% =========================================================================
%% Track D: Second-Life Battery Adaptive Converter
%% =========================================================================
%% Implements SOH estimation using coulomb counting + impedance estimation
%% and adapts CC/CV charging profile + V2G limits based on battery health.
%% =========================================================================

clc; clear; close all;
fprintf('========================================\n');
fprintf(' Track D: Second-Life Battery\n');
fprintf(' Adaptive V2G Converter\n');
fprintf('========================================\n\n');

%% ======================== BATTERY MODEL =================================
% Simplified equivalent circuit model (ECM) for NMC cell
% V_cell = OCV(SOC) - I*R_internal - V_RC
% R_internal increases with aging

% Fresh battery parameters
Q_nom_fresh = 60;           % Nominal capacity [Ah] (fresh)
R0_fresh    = 0.020;        % Internal resistance fresh [Ohms]
R1_fresh    = 0.010;        % RC pair resistance [Ohms]
C1_fresh    = 1000;         % RC pair capacitance [F]
V_max       = 4.2;          % Max cell voltage [V]
V_min       = 2.8;          % Min cell voltage [V]
N_series    = 96;           % Cells in series (for ~400V pack)
N_parallel  = 1;

% Aging factors (SOH = Q_actual / Q_nominal)
soh_levels  = [1.0, 0.9, 0.8, 0.7, 0.6];  % Test at various SOH
soh_labels  = {'100% (Fresh)', '90%', '80% (EOL-EV)', '70%', '60% (Critical)'};

% OCV curve (NMC typical)
soc_points = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
ocv_points = [3.0, 3.4, 3.55, 3.62, 3.68, 3.73, 3.80, 3.90, 4.00, 4.10, 4.20];
ocv_func   = @(soc) interp1(soc_points, ocv_points, max(0,min(1,soc)), 'pchip');

fprintf('Battery Pack: %d cells in series\n', N_series);
fprintf('Pack voltage range: %.0fV – %.0fV\n', V_min*N_series, V_max*N_series);
fprintf('\n');

%% ======================== SOH ESTIMATION ================================
fprintf('=== SOH ESTIMATION MODULE ===\n');

% Method 1: Impedance-Based SOH Estimation
% R_internal increases linearly with aging: R = R0_fresh * (2 - SOH)
est_R_from_soh = @(soh) R0_fresh * (2 - soh);
est_soh_from_R = @(R) 2 - R / R0_fresh;

% Method 2: Capacity-Based SOH
% Measure actual capacity from full charge cycle
est_soh_capacity = @(Q_actual) Q_actual / Q_nom_fresh;

% Demonstrate impedance estimation
fprintf('SOH-Impedance Relationship:\n');
for i = 1:length(soh_levels)
    soh = soh_levels(i);
    R_est = est_R_from_soh(soh);
    fprintf('  SOH=%.0f%% → R_internal=%.1f mΩ (per cell)\n', soh*100, R_est*1e3);
end
fprintf('\n');

%% ======================== ADAPTIVE CONTROL PROFILES =====================
fprintf('=== ADAPTIVE CHARGING PROFILES ===\n');

% CC/CV parameters adapted based on SOH
adaptive_profiles = struct();
for i = 1:length(soh_levels)
    soh = soh_levels(i);
    Q_actual = Q_nom_fresh * soh;
    R_int = est_R_from_soh(soh);
    
    % Adaptive rules
    if soh >= 0.9
        I_cc = 1.0 * Q_actual;      % 1C rate
        V_cv = V_max;                 % Full CV voltage
        soc_min = 0.10;              % Deep discharge OK
        soc_max = 1.00;
        v2g_enabled = true;
        v2g_power_limit = 1.0;       % 100% V2G power
    elseif soh >= 0.8
        I_cc = 0.7 * Q_actual;      % 0.7C rate (gentle)
        V_cv = V_max - 0.05;         % Slight voltage reduction
        soc_min = 0.15;
        soc_max = 0.95;
        v2g_enabled = true;
        v2g_power_limit = 0.8;       % 80% V2G power
    elseif soh >= 0.7
        I_cc = 0.5 * Q_actual;      % 0.5C rate
        V_cv = V_max - 0.10;
        soc_min = 0.20;
        soc_max = 0.90;
        v2g_enabled = true;
        v2g_power_limit = 0.5;       % 50% V2G power
    else
        I_cc = 0.3 * Q_actual;      % 0.3C rate (very gentle)
        V_cv = V_max - 0.15;
        soc_min = 0.25;
        soc_max = 0.85;
        v2g_enabled = false;          % V2G DISABLED
        v2g_power_limit = 0.0;
    end
    
    adaptive_profiles(i).soh = soh;
    adaptive_profiles(i).I_cc = I_cc;
    adaptive_profiles(i).V_cv = V_cv;
    adaptive_profiles(i).soc_range = [soc_min, soc_max];
    adaptive_profiles(i).v2g_enabled = v2g_enabled;
    adaptive_profiles(i).v2g_limit = v2g_power_limit;
    
    fprintf('SOH=%.0f%% | I_cc=%.1fA (%.1fC) | V_cv=%.2fV | SOC=[%.0f%%–%.0f%%] | V2G=%s (%.0f%%)\n', ...
        soh*100, I_cc, I_cc/Q_actual, V_cv, soc_min*100, soc_max*100, ...
        char('YES'*v2g_enabled + 'NO '*~v2g_enabled), v2g_power_limit*100);
end
fprintf('\n');

%% ======================== CHARGING SIMULATION ===========================
fprintf('=== Simulating Adaptive Charging at Different SOH ===\n');

dt_sim = 1;              % 1 second time step
t_charge = 0:dt_sim:7200; % 2 hours max
N_sim = length(t_charge);

% Results storage
soc_all     = zeros(length(soh_levels), N_sim);
v_cell_all  = zeros(length(soh_levels), N_sim);
i_charge_all = zeros(length(soh_levels), N_sim);
t_full      = zeros(length(soh_levels), 1);

for si = 1:length(soh_levels)
    soh = soh_levels(si);
    Q_actual = Q_nom_fresh * soh;
    R_int = est_R_from_soh(soh);
    R1 = R1_fresh * (2 - soh);
    
    prof = adaptive_profiles(si);
    
    soc = 0.20;          % Start at 20% SOC
    V_rc = 0;            % RC pair voltage
    charged = false;
    
    for k = 1:N_sim
        if charged
            soc_all(si, k) = soc;
            v_cell_all(si, k) = ocv_func(soc);
            i_charge_all(si, k) = 0;
            continue;
        end
        
        V_ocv = ocv_func(soc);
        
        % CC phase
        if V_ocv + prof.I_cc * R_int < prof.V_cv
            I_charge = prof.I_cc;
        else
            % CV phase: I = (V_cv - V_ocv) / R_int
            I_charge = max(0, (prof.V_cv - V_ocv) / R_int);
        end
        
        % Cutoff
        if I_charge < 0.05 * Q_actual || soc >= prof.soc_range(2)
            I_charge = 0;
            if ~charged
                charged = true;
                t_full(si) = t_charge(k);
                fprintf('  SOH=%.0f%%: Fully charged at t=%.0f min (SOC=%.1f%%)\n', ...
                    soh*100, t_charge(k)/60, soc*100);
            end
        end
        
        % Update SOC
        soc = soc + I_charge * dt_sim / (Q_actual * 3600);
        
        % Update RC pair
        V_rc = V_rc * exp(-dt_sim / (R1 * C1_fresh)) + ...
               I_charge * R1 * (1 - exp(-dt_sim / (R1 * C1_fresh)));
        
        % Terminal voltage
        V_cell = V_ocv + I_charge * R_int + V_rc;
        
        soc_all(si, k) = soc;
        v_cell_all(si, k) = V_cell;
        i_charge_all(si, k) = I_charge;
    end
end
fprintf('\n');

%% ======================== V2G CYCLING SIMULATION ========================
fprintf('=== V2G Cycling: Standard vs Adaptive Control ===\n');

N_v2g_cycles = 500;
soh_test = 0.85;  % Test at 85% SOH
Q_actual = Q_nom_fresh * soh_test;

% Standard control: fixed parameters regardless of SOH
soc_std = 0.80;       % Start at 80%
Q_loss_std = 0;

% Adaptive control: adjusted parameters
soc_adp = 0.80;
Q_loss_adp = 0;

deg_std = zeros(1, N_v2g_cycles);
deg_adp = zeros(1, N_v2g_cycles);

for cycle = 1:N_v2g_cycles
    % Standard: always 1C, full SOC range
    I_std = 1.0 * Q_actual;
    dod_std = 0.6;  % 60% DOD
    
    % Adaptive: reduced C-rate and DOD for aged battery
    I_adp = 0.7 * Q_actual;
    dod_adp = 0.4;  % 40% DOD
    
    % Degradation per cycle (simplified Rainflow model)
    % ΔQ = k * DOD^beta * I^alpha
    k_deg = 1e-5;
    beta = 2.0;
    alpha = 0.5;
    
    dQ_std = k_deg * dod_std^beta * I_std^alpha;
    dQ_adp = k_deg * dod_adp^beta * I_adp^alpha;
    
    Q_loss_std = Q_loss_std + dQ_std;
    Q_loss_adp = Q_loss_adp + dQ_adp;
    
    deg_std(cycle) = Q_loss_std;
    deg_adp(cycle) = Q_loss_adp;
end

fprintf('After %d V2G cycles (SOH_start=85%%):\n', N_v2g_cycles);
fprintf('  Standard control: %.2f%% additional capacity loss\n', Q_loss_std*100);
fprintf('  Adaptive control: %.2f%% additional capacity loss\n', Q_loss_adp*100);
fprintf('  Reduction: %.0f%%\n', (1 - Q_loss_adp/Q_loss_std)*100);
fprintf('\n');

%% ======================== DSP SOH ESTIMATOR CODE ========================
fprintf('=== GENERATED C CODE: SOH Estimator ===\n\n');
fprintf('typedef struct {\n');
fprintf('  float soh;           // State of Health [0-1]\n');
fprintf('  float R_internal;    // Estimated internal resistance [Ohms]\n');
fprintf('  float Q_actual;      // Estimated remaining capacity [Ah]\n');
fprintf('  float I_cc_limit;    // Adaptive CC current limit [A]\n');
fprintf('  float V_cv_limit;    // Adaptive CV voltage limit [V]\n');
fprintf('  float soc_min;       // Min SOC limit\n');
fprintf('  float soc_max;       // Max SOC limit\n');
fprintf('  float v2g_power_pct; // V2G power limit [0-1]\n');
fprintf('  int   v2g_enabled;   // V2G enable flag\n');
fprintf('} BatteryAdaptive_t;\n\n');

fprintf('void update_adaptive_profile(BatteryAdaptive_t *bat) {\n');
fprintf('  // Estimate SOH from internal resistance\n');
fprintf('  bat->soh = 2.0f - bat->R_internal / %.4ff;\n', R0_fresh);
fprintf('  bat->soh = fmaxf(0.0f, fminf(1.0f, bat->soh));\n');
fprintf('  bat->Q_actual = %.1ff * bat->soh;\n', Q_nom_fresh);
fprintf('  \n');
fprintf('  if (bat->soh >= 0.9f) {\n');
fprintf('    bat->I_cc_limit = 1.0f * bat->Q_actual;\n');
fprintf('    bat->V_cv_limit = %.2ff;\n', V_max);
fprintf('    bat->soc_min = 0.10f; bat->soc_max = 1.00f;\n');
fprintf('    bat->v2g_enabled = 1; bat->v2g_power_pct = 1.0f;\n');
fprintf('  } else if (bat->soh >= 0.8f) {\n');
fprintf('    bat->I_cc_limit = 0.7f * bat->Q_actual;\n');
fprintf('    bat->V_cv_limit = %.2ff;\n', V_max-0.05);
fprintf('    bat->soc_min = 0.15f; bat->soc_max = 0.95f;\n');
fprintf('    bat->v2g_enabled = 1; bat->v2g_power_pct = 0.8f;\n');
fprintf('  } else if (bat->soh >= 0.7f) {\n');
fprintf('    bat->I_cc_limit = 0.5f * bat->Q_actual;\n');
fprintf('    bat->V_cv_limit = %.2ff;\n', V_max-0.10);
fprintf('    bat->soc_min = 0.20f; bat->soc_max = 0.90f;\n');
fprintf('    bat->v2g_enabled = 1; bat->v2g_power_pct = 0.5f;\n');
fprintf('  } else {\n');
fprintf('    bat->I_cc_limit = 0.3f * bat->Q_actual;\n');
fprintf('    bat->V_cv_limit = %.2ff;\n', V_max-0.15);
fprintf('    bat->soc_min = 0.25f; bat->soc_max = 0.85f;\n');
fprintf('    bat->v2g_enabled = 0; bat->v2g_power_pct = 0.0f;\n');
fprintf('  }\n');
fprintf('}\n');

%% ======================== PLOTS =========================================
figure('Position', [50, 50, 1400, 900], 'Color', 'w');

% Plot 1: Adaptive Charging Profiles (SOC vs Time)
subplot(2,3,1);
for si = 1:length(soh_levels)
    plot(t_charge/60, soc_all(si,:)*100, 'LineWidth', 2); hold on;
end
xlabel('Time [min]'); ylabel('SOC [%]');
title('Charging Profiles at Various SOH');
legend(soh_labels, 'Location', 'southeast');
grid on; xlim([0 120]);

% Plot 2: Charging Current vs Time
subplot(2,3,2);
for si = 1:length(soh_levels)
    plot(t_charge/60, i_charge_all(si,:), 'LineWidth', 2); hold on;
end
xlabel('Time [min]'); ylabel('Charge Current [A]');
title('Adaptive CC/CV Current');
legend(soh_labels, 'Location', 'northeast');
grid on; xlim([0 120]);

% Plot 3: Cell Voltage vs Time
subplot(2,3,3);
for si = 1:length(soh_levels)
    plot(t_charge/60, v_cell_all(si,:), 'LineWidth', 2); hold on;
end
xlabel('Time [min]'); ylabel('Cell Voltage [V]');
title('Cell Voltage During Charging');
legend(soh_labels, 'Location', 'southeast');
grid on; xlim([0 120]);

% Plot 4: V2G Degradation Comparison
subplot(2,3,4);
plot(1:N_v2g_cycles, deg_std*100, 'b-', 'LineWidth', 2); hold on;
plot(1:N_v2g_cycles, deg_adp*100, 'r-', 'LineWidth', 2);
xlabel('V2G Cycles'); ylabel('Additional Capacity Loss [%]');
title('V2G Degradation: Standard vs Adaptive');
legend('Standard Control', 'Adaptive Control', 'Location', 'northwest');
grid on;

% Plot 5: SOH vs Adaptive Parameters
subplot(2,3,5);
soh_range = 0.5:0.01:1.0;
I_limit = zeros(size(soh_range));
V2G_limit = zeros(size(soh_range));
for i = 1:length(soh_range)
    if soh_range(i) >= 0.9
        I_limit(i) = 1.0; V2G_limit(i) = 1.0;
    elseif soh_range(i) >= 0.8
        I_limit(i) = 0.7; V2G_limit(i) = 0.8;
    elseif soh_range(i) >= 0.7
        I_limit(i) = 0.5; V2G_limit(i) = 0.5;
    else
        I_limit(i) = 0.3; V2G_limit(i) = 0.0;
    end
end
plot(soh_range*100, I_limit*100, 'b-', 'LineWidth', 2); hold on;
plot(soh_range*100, V2G_limit*100, 'r--', 'LineWidth', 2);
xlabel('SOH [%]'); ylabel('Limit [% of max]');
title('Adaptive Parameter Map');
legend('Charge C-rate Limit', 'V2G Power Limit', 'Location', 'northwest');
grid on;

% Plot 6: OCV Curve
subplot(2,3,6);
soc_fine = linspace(0, 1, 100);
ocv_fine = arrayfun(ocv_func, soc_fine);
plot(soc_fine*100, ocv_fine * N_series, 'b-', 'LineWidth', 2);
xlabel('SOC [%]'); ylabel('Pack Voltage [V]');
title(sprintf('OCV Curve (%ds cells)', N_series));
grid on;

sgtitle('Track D: Second-Life Battery Adaptive V2G Converter', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('=== Track D simulation complete ===\n');
