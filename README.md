# Control of Uncertain Systems: AGV Controller Design

This repository contains the MATLAB and Simulink code for the *Control of Uncertain Systems* course project. The goal is to design and analyze feedback controllers for an Automated Guided Vehicle (AGV) with uncertain parameters, using three synthesis techniques: **LQG**, **H∞**, and **DK-iteration (μ-synthesis)**.

---

## System Description

The system is a unicycle-like AGV whose motion is controlled by two torques acting on the rear motor-wheel:
- τ_φ: rolling torque (drives translation)
- τ_ψ: steering torque (controls heading)

The vehicle is modeled by nonlinear kinematics and dynamics, then linearized around the equilibrium condition of straight-line motion at constant speed v_x = 1 m/s.

<p align="center">
  <img src="figures/foto AGV.png" width="400" alt="AGV diagram"/>
  <br><em>Figure 1: Automated Guided Vehicle (AGV) — geometry and reference frames</em>
</p>

The observed outputs are the lateral position y and the rear wheel angular velocity φ̇. The choice of these two outputs guarantees full reachability and observability of the linearized system.

---

## Uncertainty Modeling

Two sources of uncertainty are considered:

**1. Parametric uncertainty (system)**  
The rear and front wheel radii R_p and R_a are modeled as `ureal` objects with ±5% variation. The resulting uncertainty structure is an 8×8 diagonal matrix extracted via `lftdata`.

**2. Actuator uncertainty**  
Each actuator is modeled with a multiplicative input uncertainty:

$$G_{\tau_\varphi} = \bar{G}_{\tau_\varphi}(1 + W_{I,\tau_\varphi}\,\Delta_I)$$

The uncertainty weight W_I is fitted from 100 random samples of the perturbed actuator transfer function using MATLAB's `ucover` command.

<p align="center">
  <img src="figures/schema moltiplicativo.png" width="550" alt="System block diagram"/>
  <br><em>Figure 2: Full system block diagram with performance weights W_P, W_u, W_T and uncertainty channels</em>
</p>

---

## Controllers

### 1: LQG Controller

The LQG controller is designed on the linearized nominal system under the separation principle: an LQR state-feedback gain and a Kalman filter are designed independently and then combined.

Two implementations are provided:
- **Regulator with integral action** (`LQG_controller_REG_FINAL.m`): plant augmented with integrators, gains computed via `lqr` + `kalman`.
- **Servo-controller** (`LQG_controller_SERV_FINAL.m`): designed directly with MATLAB's `lqg` command, available in both 1-DOF and 2-DOF configurations.

The μ-analysis reveals that the LQG controller is nominally performant but not robustly stable or performant under the full uncertainty set.

<p align="center">
  <img src="figures/mu analisi LQG.png" width="500" alt="μ-analysis LQG"/>
  <br><em>Figure 3: μ-analysis of the LQG controller — muRP and muRS exceed unity</em>
</p>

---

### 2: H∞ Controller

The H∞ controller minimizes the H∞ norm of the weighted sensitivity function. The performance weight W_P is a diagonal transfer matrix with entries of the form:

$$W_{P_i}(s) = \frac{s/M_i + \omega_{B_i}}{s + A_i\,\omega_{B_i}}$$

Two synthesis methods are implemented:
- **`mixsyn`** (`Hinf_controller_FINAL.m`): standard mixed-sensitivity formulation
- **`hinfsyn` + `connect`** (`Hinf_controller_hinfsyn_FINAL.m`): explicit generalized plant construction

Both yield similar closed-loop performance. The μ-analysis shows the H∞ controller is also not robust under the full uncertainty.

<p align="center">
  <img src="figures/mu analisi Hinf.png" width="500" alt="μ-analysis H∞"/>
  <br><em>Figure 4: μ-analysis of the H∞ controller</em>
</p>

---

### 3: DK-Iteration (μ-Synthesis)

Since no direct optimal μ-controller design method exists, the DK-iteration is employed. It alternates between:

1. **K-step**: H∞ synthesis on the scaled plant `D·P·D⁻¹` (via `hinfsyn`)
2. **D-step**: computation of the μ upper bound via `mussv`
3. **Fitting step**: rational approximation of D(jω) via `fitmagfrd` (4th-order minimum-phase fit)

Two iterations are sufficient to bring the structured singular value below unity, achieving robust performance.

<p align="center">
  <img src="figures/DK risultati mu.png" width="500" alt="DK-iteration convergence"/>
  <br><em>Figure 5: Structured singular value across DK-iteration steps — μ < 1 achieved at step 2</em>
</p>

<p align="center">
  <img src="figures/mu analisi DK.png" width="500" alt="μ-analysis DK controller"/>
  <br><em>Figure 6: μ-analysis of the DK-iteration controller — robust stability and performance satisfied</em>
</p>

---

## Region of Asymptotic Stability (RAS)

For each controller, the RAS of the closed-loop nonlinear system is estimated empirically in Simulink by increasing the step disturbance amplitude on y until divergence occurs:

| Controller | RAS threshold (step on y) |
|:----------:|:-------------------------:|
| LQG        | < 0.09                    |
| H∞         | < 2.0                     |
| DK-iter.   | < 0.05 (robust version)   |

The H∞ controller shows the widest basin of attraction on the nonlinear system, while the DK-iteration controller (optimized for robustness under uncertainty) exhibits a narrower RAS due to more conservative performance requirements.

---

## Requirements

- **MATLAB** R2020b or later (earlier versions may work)
- **Toolboxes**: Control System Toolbox, Robust Control Toolbox
- **Simulink** (for `.slx` simulation files)

---

## Usage

1. Run `Parameter_CSI.m` first — it initializes all system parameters, builds the uncertain model, and defines `G_tot_nom`, `G_tot`, `WI_tot`, and related quantities used by all other scripts.
2. Run any controller script (e.g., `LQG_controller_REG_FINAL.m`) to synthesize and simulate the corresponding controller.
3. Run the corresponding `mu_analysis_*.m` script to perform the μ-analysis.
4. Open the Simulink models (`Nonlinear_*.slx`) for time-domain closed-loop simulation on the nonlinear plant.

> **Note:** all scripts call `Parameter_CSI` internally at startup, so they can also be run standalone.

---

## Authors

Anna Coscetti  
*Control of Uncertain Systems* — MSc course project