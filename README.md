# 🚗 Nonlinear Active Suspension Control

A simulation and control framework for a **nonlinear quarter-car active suspension system**, focused on improving ride comfort and road holding through advanced control strategies.

The project compares:

- Passive Suspension
- Active Suspension with LQR (based on the linearized system)
- Active Suspension with Nonlinear MPC (NLMPC)

under different road conditions, uncertainties and disturbances.

---

## ✨ Highlights

- Nonlinear quarter-car modeling
- Lyapunov stability analysis
- Passive vs Active suspension comparison
- LQR controller synthesis
- Nonlinear Model Predictive Control
- Robustness analysis
- Parametric uncertainty tests
- Sensor noise and disturbance rejection
- Simulink + MATLAB implementation

---

## 📸 Project Overview

The suspension system is evaluated on multiple road profiles:

- Smooth road bump
- Aggressive off-road scenario

with the objective of minimizing:

- Vehicle body oscillations and acceleration in vertical direction (comfort)
- Tire-road detachment (safety)

while respecting actuator limits and nonlinear dynamics.

---

## 🧠 Control Strategies

### 🔹 Passive Suspension

Baseline nonlinear suspension model used as reference for performance evaluation.

### 🔹 LQR Control

Linear Quadratic Regulator designed around the linearized model with integral action for:

- zero steady-state error
- robustness to uncertainties
- improved comfort

### 🔹 Nonlinear MPC

Advanced predictive controller implemented using MATLAB `nlmpc`.

Features include:

- nonlinear prediction model
- actuator constraints
- state constraints
- input rate limits
- real-time optimization
- disturbance handling

---

## 📊 Results

The simulations show significant improvements in ride comfort using active suspension systems.

### Example Improvements

| Controller | Road Profile | Comfort Improvement |
|---|---|---|
| LQR | Road Bump | ~96% |
| LQR | Off-Road | ~94% |
| NLMPC | Road Bump | ~99% |
| NLMPC | Off-Road | ~99% |

NLMPC demonstrated the best overall behavior, especially in terms of:

- reduced vehicle acceleration
- smoother response
- robustness to uncertainties
- predictive disturbance compensation

---

## 🛡️ Robustness Analysis

The controllers were also tested under realistic scenarios such as:

- additional passenger mass
- wheel mass variations
- noisy road estimation
- actuator saturation

The project investigates how active suspensions degrade under imperfect real-world conditions and compares controller resilience.

---

## 🧪 Technologies and Requirements

- MATLAB R2024a
- Simulink
- Control System Toolbox
- Model Predictive Control Toolbox
- Optimization Toolbox
- Symbolic Toolbox

---

## 📂 Repository Structure

In each folder you'll find the respective scripts used in the report.
The main file to run is present in the 'chapter3' folder and it's user-oriented so you can easly run all the scenarios without the need to modify any code.

```text
.
├── chapter1/
├── chapter2/
├── chapter3/
├── report and ppt/
└── README.md
