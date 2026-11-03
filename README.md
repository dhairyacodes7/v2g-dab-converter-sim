# V2G DAB Converter: MATLAB Simulation Study

A simulation study of a 3.3 kW bidirectional Dual Active Bridge (DAB) DC-DC converter for vehicle-to-grid (V2G) use. I did this project alone.

**This is simulation only. No converter was built, so none of these numbers were measured on hardware.** The scripts use simple models with assumed values, and Track B uses data the script generates itself. I am sharing the results as the code prints them, including the weak ones.

## Design values

| Parameter | Value |
|---|---|
| Topology | Dual Active Bridge, single phase shift |
| Rated power | 3.3 kW |
| Battery side | 250–450 V (350 V nominal) |
| DC bus | 400 V |
| Switching frequency | 50 kHz |
| Transformer ratio | 1:1 |

## Results

These are the values the scripts printed in MATLAB R2025b on 3 Nov 2026.

| Part | What it compares | Result | Read this with care |
|---|---|---|---|
| **Power stage** | Phase shift sweep, 5° to 60°, 85 µH | 3,304 W at 50°, peak current 14.38 A | It also prints efficiency, but I don't quote it because its loss estimate leaves out switching loss. |
| **Track A** | MPC vs PI, 0.5 s run with load steps | Battery wear 1.5% lower; overshoot 17.5% → 11.3% | MPC is worse in two ways: 0.19 V steady-state error (PI has 0) and higher RMS current (3.15 A vs 2.98 A). |
| **Track B** | Mahalanobis anomaly detector on generated data (200 healthy and 200 attack samples) | Accuracy 85.5%, precision 94.4%, recall 75.5%, false alarms 4.5% | It catches voltage offset, current scaling and replay attacks (50/50 each), but only **1 of 50** phase attacks. |
| **Track C** | 3-node thermal model at 25–55 °C ambient | Junction 35.5–65.5 °C, full power at every ambient | Derating never triggers. The loss model leaves out switching loss, so this does not really test the derating logic. |
| **Track D** | Adaptive vs standard charging, 500 cycles at 85% SOH | Extra capacity loss 1.29% → 0.48% (63% lower) | This comes straight from an assumed formula, not battery data. The adaptive case also moves less energy per cycle (40% vs 60% depth of discharge). |
| **Track E** | Virtual synchronous machine (VSM) vs droop | Max rate of change of frequency 4,734.85 → 2.39 Hz/s | VSM is worse on frequency deviation (2.500 vs 1.136 Hz) and DC bus deviation (20.0 vs 9.1 V). The 2.500 Hz is the ±5% limit in the code, and the droop model has no inertia, so don't trust the "100% reduction" the script prints. |

## Known problems

1. **Peak current in `dab_design_calculator.m` is wrong.** It prints 66.47 A. Working out the same values by hand gives about 14.1 A, which also matches the 9.43 A average input current it prints. The RMS current, losses, 86.62% efficiency and negative heatsink value all use this number, so ignore them until it is fixed.
2. **Two inductor values.** The calculator gives 58.92 µH, but the other scripts use 85 µH. With 85 µH, a 30° phase shift gives 2,288 W, so rated power needs about 50°.
3. **LTspice measurements fail.** `ltspice/dab_converter.asc` runs, but all 5 of its `.meas` lines fail, so it gives no result numbers yet.
4. **Track B warning.** MATLAB warns that the covariance matrix is close to singular (RCOND about 5e-20), so the detector needs a better set of features.
5. **The firmware is only a skeleton.** `dsp-firmware/dab_v2g_main.h` was written for a TI TMS320F28379D, but it has never been compiled or run on one.

## Files

| Path | What it is |
|---|---|
| `matlab/run_all_simulations.m` | Runs everything below |
| `matlab/dab_design_calculator.m` | Design calculations (see known problem 1) |
| `matlab/dab_simulation.m` | Power stage sweep and waveforms |
| `matlab/dab_mpc_controller.m` | Track A: MPC vs PI |
| `matlab/track_b_anomaly_detection.m` | Track B: anomaly detection |
| `matlab/track_c_thermal_derating.m` | Track C: thermal model and derating |
| `matlab/track_d_second_life_battery.m` | Track D: second-life battery control |
| `matlab/track_e_grid_forming_vsm.m` | Track E: VSM vs droop |
| `ltspice/dab_converter.asc` | LTspice netlist of the power stage |
| `dsp-firmware/dab_v2g_main.h` | Firmware skeleton |

## Run it

Tested in MATLAB R2025b. In MATLAB:

```matlab
cd matlab
run_all_simulations
```

It takes about 10 seconds and opens the figure windows.
