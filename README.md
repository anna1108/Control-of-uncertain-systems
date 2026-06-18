# Control of Uncertain Systems

MATLAB/Simulink project for the design and analysis of robust controllers for an AGV (Automated Guided Vehicle) — specifically a differential-drive forklift with front steering.

## System Description

The plant is a nonlinear AGV model linearized about a straight-line equilibrium. The state vector is:

| Variable | Description |
|---|---|
| `y` | lateral position |
| `θ` | vehicle heading angle |
| `φ̇` | rear wheel angular velocity |
| `ψ` | steering angle |
| `ψ̇` | steering angular velocity |

Inputs are the driving torque `τ_φ` (rear wheel) and the steering torque `τ_ψ`. The two outputs fed back are the lateral position `y` and the rear wheel velocity `φ̇`.

Parametric uncertainty is modeled on the rear/front wheel radii (`R_rear`, `R_front`, ±5%) and the actuator gains and time constants (±10–20%). An input-multiplicative uncertainty weight `WI` is fitted to actuator sample data.

## Repository Structure

| File | Purpose |
|---|---|
| `Parameter_CSI.m` | System parameters, actuator models, uncertainty weights, linearized matrices, reachability/observability checks |
| `Dynamics_CSI.m` | Nonlinear dynamics function (used by Simulink S-function blocks) |
| `Linearization_forklift.m` | Symbolic Jacobian linearization of the nonlinear model |
| `LQG_controller_REG_FINAL.m` | LQG regulator design (state feedback + Kalman filter), 1-DOF and 2-DOF |
| `LQG_controller_SERV_FINAL.m` | LQG servo-controller design (1-DOF and 2-DOF, via `lqg` command) |
| `Hinf_controller_FINAL.m` | Mix-sensitivity H∞ controller via `mixsyn` |
| `Hinf_controller_hinfsyn_FINAL.m` | Mix-sensitivity H∞ controller via `hinfsyn` (generalized plant form) |
| `D_K_controller2_FINAL.m` | D-K iteration for µ-synthesis |
| `mu_analysis_DK_FINAL.m` | µ-analysis of the D-K controller (RS, NP, RP) |
| `mu_analysis_Hinf_FINAL.m` | µ-analysis of the H∞ controller (RS, NP, RP) |
| `mu_analysis_Klqg_FINAL_FINAL.m` | µ-analysis of the LQG controller (RS, NP, RP) |
| `Non_Linearizzato_DK.slx` | Simulink model: nonlinear plant + D-K controller *(see note below)* |
| `Non_Linearizzato_Hinf.slx` | Simulink model: nonlinear plant + H∞ controller *(see note below)* |
| `Non_Linearizzato_LQG.slx` | Simulink model: nonlinear plant + LQG controller *(see note below)* |
| `SIM_LIN.slx` | Simulink model: linearized plant simulation |

## Controllers Implemented

### LQG (Linear Quadratic Gaussian)
Two variants: a classical regulator (`LQG_controller_REG_FINAL.m`) that manually assembles the LQR + Kalman filter, and a servo-controller (`LQG_controller_SERV_FINAL.m`) using MATLAB's `lqg` command. Both are provided in 1-DOF (feedback only) and 2-DOF (feedforward + feedback) configurations.

### H∞ Mix-Sensitivity
Two synthesis approaches: `mixsyn` (direct) and `hinfsyn` (explicit generalized plant). Performance weight `WP` is a first-order filter shaped to meet bandwidth and sensitivity peak requirements.

### D-K Iteration (µ-synthesis)
Iterative procedure alternating between H∞ synthesis and D-scaling to minimize the structured singular value µ. Two iterations are performed.

## µ-Analysis

Each controller is evaluated for:
- **NP** (Nominal Performance): µ of the nominal closed-loop `N₂₂`
- **RS** (Robust Stability): µ w.r.t. the uncertainty block only (`N₁₁`)
- **RP** (Robust Performance): µ w.r.t. the full structured uncertainty

## Requirements

- MATLAB with **Robust Control Toolbox**
- **Control System Toolbox**
- **Simulink** (for `.slx` models)

## Usage

Run `Parameter_CSI.m` first to populate the workspace, then run any controller or analysis script:

```matlab
Parameter_CSI               % initialize workspace
LQG_controller_SERV_FINAL   % design LQG servo-controller and plot results
mu_analysis_Klqg_FINAL_FINAL % run mu-analysis on LQG controller
```

> **Note on Simulink models**: The `.slx` files (`Non_Linearizzato_*.slx`) are binary and reference the S-function `Dynamics_CSI` by its old name `Dinamica_CSI`. After renaming the `.m` file you must open each Simulink model, find the S-Function block, and update the function name to `Dynamics_CSI` to restore simulation.
