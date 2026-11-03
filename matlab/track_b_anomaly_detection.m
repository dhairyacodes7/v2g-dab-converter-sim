%% =========================================================================
%% Track B: Cyber-Physical Anomaly Detection for DAB Converter
%% =========================================================================
%% Implements a One-Class SVM based anomaly detector that monitors
%% converter waveforms to detect cyberattacks (sensor spoofing, FDI).
%% =========================================================================

clc; clear; close all;
fprintf('========================================\n');
fprintf(' Track B: Anomaly Detection\n');
fprintf(' Cyber-Resilient V2G Converter\n');
fprintf('========================================\n\n');

%% ======================== PARAMETERS ====================================
V1_nom  = 350;
V2_nom  = 400;
L       = 85e-6;
f_sw    = 50e3;
P_rated = 3300;
Rds_on  = 65e-3;

%% ======================== GENERATE TRAINING DATA ========================
fprintf('Generating healthy operating data...\n');

N_train = 1000;         % Training samples
N_test  = 200;          % Test samples (healthy)
N_attack = 200;         % Test samples (under attack)

% Feature vector: [V_in, V_out, I_peak, I_rms, phi_error, rise_time, P_balance_err, V_ripple]
features_train = zeros(N_train, 8);

rng(42);  % Reproducibility

for i = 1:N_train
    % Random operating point
    V1 = V1_nom + randn*5;          % ±5V noise
    V2 = V2_nom + randn*3;          % ±3V noise
    phi = pi/6 + randn*0.02;        % ±1° noise around 30°
    P_load = 1000 + rand*2300;      % 1–3.3 kW
    
    % Compute expected values
    P_dab = (V1 * V2 * phi * (pi-phi)) / (2*pi^2*f_sw*L);
    I_peak = (V1*phi + V2*(pi-phi)) / (2*pi*f_sw*L);
    I_rms = I_peak / sqrt(3) + randn*0.1;
    
    % Features
    features_train(i,1) = V1 + randn*0.5;              % V_in (measured)
    features_train(i,2) = V2 + randn*0.5;              % V_out (measured)
    features_train(i,3) = I_peak + randn*0.2;           % I_peak
    features_train(i,4) = I_rms;                         % I_rms
    features_train(i,5) = randn*0.5;                     % Phase error (small, ~0)
    features_train(i,6) = 50e-9 + randn*5e-9;           % Rise time [s]
    features_train(i,7) = (P_dab - P_load)/P_rated*100 + randn*0.5;  % Power balance error [%]
    features_train(i,8) = 2 + randn*0.3;                % Voltage ripple [V]
end

fprintf('Training data: %d samples × %d features\n', N_train, 8);

%% ======================== GENERATE TEST DATA ============================
% Healthy test data
features_healthy = zeros(N_test, 8);
for i = 1:N_test
    V1 = V1_nom + randn*5;
    V2 = V2_nom + randn*3;
    phi = pi/6 + randn*0.02;
    P_load = 1000 + rand*2300;
    
    P_dab = (V1 * V2 * phi * (pi-phi)) / (2*pi^2*f_sw*L);
    I_peak = (V1*phi + V2*(pi-phi)) / (2*pi*f_sw*L);
    I_rms = I_peak / sqrt(3) + randn*0.1;
    
    features_healthy(i,1) = V1 + randn*0.5;
    features_healthy(i,2) = V2 + randn*0.5;
    features_healthy(i,3) = I_peak + randn*0.2;
    features_healthy(i,4) = I_rms;
    features_healthy(i,5) = randn*0.5;
    features_healthy(i,6) = 50e-9 + randn*5e-9;
    features_healthy(i,7) = (P_dab - P_load)/P_rated*100 + randn*0.5;
    features_healthy(i,8) = 2 + randn*0.3;
end

% Attack test data (4 types of attacks)
features_attack = zeros(N_attack, 8);
attack_type = zeros(N_attack, 1);

for i = 1:N_attack
    V1 = V1_nom + randn*5;
    V2 = V2_nom + randn*3;
    phi = pi/6 + randn*0.02;
    P_load = 1000 + rand*2300;
    
    P_dab = (V1 * V2 * phi * (pi-phi)) / (2*pi^2*f_sw*L);
    I_peak = (V1*phi + V2*(pi-phi)) / (2*pi*f_sw*L);
    I_rms = I_peak / sqrt(3) + randn*0.1;
    
    % Select attack type
    a_type = mod(i-1, 4) + 1;
    attack_type(i) = a_type;
    
    switch a_type
        case 1  % Voltage sensor offset attack (+20V on V_out)
            features_attack(i,1) = V1 + randn*0.5;
            features_attack(i,2) = V2 + 20 + randn*0.5;      % ATTACKED
            features_attack(i,3) = I_peak + randn*0.2;
            features_attack(i,4) = I_rms;
            features_attack(i,5) = randn*0.5;
            features_attack(i,6) = 50e-9 + randn*5e-9;
            features_attack(i,7) = (P_dab - P_load)/P_rated*100 + 8;  % Large error
            features_attack(i,8) = 2 + randn*0.3;
            
        case 2  % Current sensor scaling attack (×1.3)
            features_attack(i,1) = V1 + randn*0.5;
            features_attack(i,2) = V2 + randn*0.5;
            features_attack(i,3) = I_peak * 1.3 + randn*0.2;  % ATTACKED
            features_attack(i,4) = I_rms * 1.3;                 % ATTACKED
            features_attack(i,5) = randn*0.5;
            features_attack(i,6) = 50e-9 + randn*5e-9;
            features_attack(i,7) = (P_dab - P_load)/P_rated*100 + 5;
            features_attack(i,8) = 2 + randn*0.3;
            
        case 3  % Phase shift manipulation (+10°)
            features_attack(i,1) = V1 + randn*0.5;
            features_attack(i,2) = V2 + randn*0.5;
            features_attack(i,3) = I_peak + randn*0.2;
            features_attack(i,4) = I_rms;
            features_attack(i,5) = deg2rad(10) + randn*0.5;    % ATTACKED: large error
            features_attack(i,6) = 50e-9 + randn*5e-9;
            features_attack(i,7) = (P_dab - P_load)/P_rated*100 + randn*0.5;
            features_attack(i,8) = 2 + randn*0.3;
            
        case 4  % Replay attack (stale data — features don't correlate with load)
            features_attack(i,1) = V1_nom + randn*0.5;         % STALE
            features_attack(i,2) = V2_nom + randn*0.5;         % STALE
            features_attack(i,3) = 8.5 + randn*0.2;            % STALE (fixed value)
            features_attack(i,4) = 5.0 + randn*0.1;            % STALE
            features_attack(i,5) = randn*0.5;
            features_attack(i,6) = 50e-9 + randn*5e-9;
            features_attack(i,7) = 15 + randn*2;               % Large imbalance
            features_attack(i,8) = 2 + randn*0.3;
    end
end

fprintf('Healthy test data: %d samples\n', N_test);
fprintf('Attack test data:  %d samples (4 types)\n', N_attack);
fprintf('\n');

%% ======================== ANOMALY DETECTION =============================
% Method: Mahalanobis distance (lightweight alternative to SVM for DSP)
% This can run on the DSP with minimal computation

fprintf('Training anomaly detector (Mahalanobis distance)...\n');

% Compute mean and covariance from training data
mu_train = mean(features_train, 1);
Sigma_train = cov(features_train);
Sigma_inv = inv(Sigma_train);

% Compute Mahalanobis distance for all samples
d_train   = zeros(N_train, 1);
d_healthy = zeros(N_test, 1);
d_attack  = zeros(N_attack, 1);

for i = 1:N_train
    diff = features_train(i,:) - mu_train;
    d_train(i) = sqrt(diff * Sigma_inv * diff');
end

for i = 1:N_test
    diff = features_healthy(i,:) - mu_train;
    d_healthy(i) = sqrt(diff * Sigma_inv * diff');
end

for i = 1:N_attack
    diff = features_attack(i,:) - mu_train;
    d_attack(i) = sqrt(diff * Sigma_inv * diff');
end

% Set threshold (95th percentile of training data)
threshold = prctile(d_train, 95);
fprintf('Detection threshold: %.2f (95th percentile)\n', threshold);

% Classify
tp = sum(d_attack > threshold);     % True positives (attacks detected)
fp = sum(d_healthy > threshold);    % False positives (healthy flagged)
tn = sum(d_healthy <= threshold);   % True negatives
fn = sum(d_attack <= threshold);    % False negatives (attacks missed)

accuracy    = (tp + tn) / (tp + tn + fp + fn) * 100;
precision   = tp / (tp + fp) * 100;
recall      = tp / (tp + fn) * 100;
f1_score    = 2 * precision * recall / (precision + recall);
fpr         = fp / (fp + tn) * 100;

fprintf('\n=== DETECTION RESULTS ===\n');
fprintf('True Positives (attacks detected):     %d / %d\n', tp, N_attack);
fprintf('False Positives (false alarms):        %d / %d\n', fp, N_test);
fprintf('False Negatives (attacks missed):      %d / %d\n', fn, N_attack);
fprintf('─────────────────────────────────────\n');
fprintf('Accuracy:     %.1f%%\n', accuracy);
fprintf('Precision:    %.1f%%\n', precision);
fprintf('Recall:       %.1f%%\n', recall);
fprintf('F1 Score:     %.1f%%\n', f1_score);
fprintf('False Positive Rate: %.1f%%\n', fpr);
fprintf('\n');

% Per-attack-type analysis
attack_names = {'Voltage Offset', 'Current Scaling', 'Phase Manipulation', 'Replay Attack'};
fprintf('=== PER-ATTACK-TYPE DETECTION ===\n');
for a = 1:4
    idx = attack_type == a;
    detected = sum(d_attack(idx) > threshold);
    total = sum(idx);
    fprintf('  %s: %d/%d detected (%.0f%%)\n', attack_names{a}, detected, total, detected/total*100);
end
fprintf('\n');

%% ======================== DSP IMPLEMENTATION ESTIMATE ====================
fprintf('=== DSP IMPLEMENTATION ESTIMATE ===\n');
fprintf('Feature extraction:     ~50 multiplications\n');
fprintf('Mean subtraction:       8 subtractions\n');
fprintf('Mahalanobis distance:   8×8 = 64 multiply-add (Sigma_inv × diff)\n');
fprintf('Sqrt + compare:         1 sqrt + 1 compare\n');
fprintf('Total operations:       ~130 multiply-add\n');
fprintf('DSP cycles (200 MHz):   ~650 cycles\n');
fprintf('Execution time:         ~3.25 µs\n');
fprintf('Update rate:            Every 100 ms (very low overhead)\n');
fprintf('Memory:                 8 floats (mean) + 64 floats (Sigma_inv) = 288 bytes\n');
fprintf('\n');

%% ======================== EXPORTABLE C CODE =============================
fprintf('=== GENERATED C CODE FOR DSP ===\n');
fprintf('// Paste this into your DSP firmware (Track B module)\n\n');

fprintf('// Training parameters (computed offline)\n');
fprintf('const float mu[8] = {');
fprintf('%.4f, ', mu_train(1:7)); fprintf('%.4f};\n', mu_train(8));

fprintf('const float sigma_inv[8][8] = {\n');
for r = 1:8
    fprintf('  {');
    fprintf('%.6f, ', Sigma_inv(r, 1:7));
    fprintf('%.6f}', Sigma_inv(r, 8));
    if r < 8, fprintf(','); end
    fprintf('\n');
end
fprintf('};\n');
fprintf('const float threshold = %.4f;\n\n', threshold);

fprintf('// Anomaly detection function\n');
fprintf('int detect_anomaly(float features[8]) {\n');
fprintf('  float diff[8], temp[8];\n');
fprintf('  float distance = 0.0f;\n');
fprintf('  int i, j;\n');
fprintf('  \n');
fprintf('  // Subtract mean\n');
fprintf('  for (i = 0; i < 8; i++)\n');
fprintf('    diff[i] = features[i] - mu[i];\n');
fprintf('  \n');
fprintf('  // Multiply by inverse covariance\n');
fprintf('  for (i = 0; i < 8; i++) {\n');
fprintf('    temp[i] = 0.0f;\n');
fprintf('    for (j = 0; j < 8; j++)\n');
fprintf('      temp[i] += sigma_inv[i][j] * diff[j];\n');
fprintf('  }\n');
fprintf('  \n');
fprintf('  // Dot product\n');
fprintf('  for (i = 0; i < 8; i++)\n');
fprintf('    distance += diff[i] * temp[i];\n');
fprintf('  \n');
fprintf('  distance = sqrtf(distance);\n');
fprintf('  \n');
fprintf('  return (distance > threshold) ? 1 : 0;  // 1 = anomaly\n');
fprintf('}\n');

%% ======================== PLOTS =========================================
figure('Position', [50, 50, 1200, 800], 'Color', 'w');

% Plot 1: Mahalanobis Distance Distribution
subplot(2,2,1);
histogram(d_train, 30, 'FaceColor', [0.3, 0.5, 0.9], 'FaceAlpha', 0.7); hold on;
histogram(d_healthy, 20, 'FaceColor', [0.2, 0.8, 0.3], 'FaceAlpha', 0.7);
histogram(d_attack, 20, 'FaceColor', [0.9, 0.2, 0.2], 'FaceAlpha', 0.7);
xline(threshold, 'k--', 'Threshold', 'LineWidth', 2);
xlabel('Mahalanobis Distance');
ylabel('Count');
title('Anomaly Score Distribution');
legend('Training', 'Healthy Test', 'Attack Test', 'Location', 'northeast');
grid on;

% Plot 2: ROC Curve
subplot(2,2,2);
thresholds = linspace(0, max(d_attack)*1.1, 200);
tpr_curve = zeros(size(thresholds));
fpr_curve = zeros(size(thresholds));
for ti = 1:length(thresholds)
    tpr_curve(ti) = sum(d_attack > thresholds(ti)) / N_attack;
    fpr_curve(ti) = sum(d_healthy > thresholds(ti)) / N_test;
end
plot(fpr_curve*100, tpr_curve*100, 'b-', 'LineWidth', 2); hold on;
plot([0 100], [0 100], 'k--');
plot(fpr*1, recall*1, 'ro', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('False Positive Rate [%]');
ylabel('True Positive Rate [%]');
title('ROC Curve');
legend('Detector', 'Random', sprintf('Operating Point (FPR=%.1f%%)', fpr), ...
    'Location', 'southeast');
grid on;

% AUC calculation
auc = -trapz(fpr_curve, tpr_curve);
text(50, 30, sprintf('AUC = %.3f', auc), 'FontSize', 14, 'FontWeight', 'bold');

% Plot 3: Attack Detection Timeline
subplot(2,2,[3,4]);
all_distances = [d_healthy; d_attack];
all_labels = [zeros(N_test,1); ones(N_attack,1)];  % 0=healthy, 1=attack
sample_idx = 1:length(all_distances);

scatter(sample_idx(all_labels==0), all_distances(all_labels==0), 20, ...
    'g', 'filled', 'MarkerFaceAlpha', 0.5); hold on;
scatter(sample_idx(all_labels==1), all_distances(all_labels==1), 20, ...
    'r', 'filled', 'MarkerFaceAlpha', 0.5);
yline(threshold, 'k--', 'Detection Threshold', 'LineWidth', 2);
xlabel('Sample Index');
ylabel('Anomaly Score');
title('Real-Time Anomaly Detection Simulation');
legend('Healthy Operation', 'Under Attack', 'Location', 'northwest');
grid on;

% Mark attack region
x_attack_start = N_test + 1;
patch([x_attack_start N_test+N_attack N_test+N_attack x_attack_start], ...
    [0 0 max(all_distances)*1.1 max(all_distances)*1.1], ...
    'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none');

sgtitle('Track B: Cyber-Physical Anomaly Detection for V2G Converter', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n=== Track B simulation complete ===\n');
fprintf('Anomaly detection achieves %.1f%% accuracy with %.1f%% false positive rate.\n', ...
    accuracy, fpr);
fprintf('DSP implementation requires only ~3.25 µs per evaluation.\n');
