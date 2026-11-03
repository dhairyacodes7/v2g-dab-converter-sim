%% =========================================================================
%% MASTER SIMULATION RUNNER — All 5 Tracks
%% =========================================================================
%% Run this script to execute all simulations in sequence.
%% Each track generates its own figure window.
%% =========================================================================

clc; clear; close all;
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   V2G BIDIRECTIONAL DC-DC CONVERTER — FULL SIMULATION   ║\n');
fprintf('║   All 5 Innovation Tracks                               ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% Run Design Calculator
fprintf('▶ [1/6] Running Design Calculator...\n');
fprintf('────────────────────────────────────\n');
run('dab_design_calculator.m');
fprintf('\n\n');

%% Run Power Stage Simulation
fprintf('▶ [2/6] Running DAB Simulation...\n');
fprintf('────────────────────────────────────\n');
run('dab_simulation.m');
fprintf('\n\n');

%% Run Track A: MPC Controller
fprintf('▶ [3/6] Running Track A: MPC + Battery Health...\n');
fprintf('────────────────────────────────────\n');
run('dab_mpc_controller.m');
fprintf('\n\n');

%% Run Track B: Anomaly Detection
fprintf('▶ [4/6] Running Track B: Cybersecurity...\n');
fprintf('────────────────────────────────────\n');
run('track_b_anomaly_detection.m');
fprintf('\n\n');

%% Run Track C: Thermal Derating
fprintf('▶ [5/6] Running Track C: Thermal Derating...\n');
fprintf('────────────────────────────────────\n');
run('track_c_thermal_derating.m');
fprintf('\n\n');

%% Run Track D: Second-Life Battery
fprintf('▶ [6/6] Running Track D: Second-Life Battery...\n');
fprintf('────────────────────────────────────\n');
run('track_d_second_life_battery.m');
fprintf('\n\n');

%% Run Track E: Grid-Forming VSM
fprintf('▶ [7/7] Running Track E: Grid-Forming VSM...\n');
fprintf('────────────────────────────────────\n');
run('track_e_grid_forming_vsm.m');
fprintf('\n\n');

%% Summary
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║                  ALL SIMULATIONS COMPLETE                ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║ Figure 1: Design Calculator Plots                       ║\n');
fprintf('║ Figure 2: DAB Power Stage Waveforms & Efficiency        ║\n');
fprintf('║ Figure 3: Track A — MPC vs PI Controller Comparison     ║\n');
fprintf('║ Figure 4: Track B — Anomaly Detection & ROC Curve       ║\n');
fprintf('║ Figure 5: Track C — Thermal Derating & Climate Analysis ║\n');
fprintf('║ Figure 6: Track D — 2nd Life Battery Adaptive Control   ║\n');
fprintf('║ Figure 7: Track E — Grid-Forming VSM vs Droop           ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║ Total figures generated: 7                              ║\n');
fprintf('║ Total plots: ~40                                        ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
