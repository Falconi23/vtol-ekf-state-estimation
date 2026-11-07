# VTOL EKF State Estimation

**A 20-State Extended Kalman Filter for the Sapienza Flight Team VTOL**

A custom 20-state Extended Kalman Filter that replaced the legacy estimator of the team's VTOL tricopter, fusing IMU, GNSS, barometer and AHRS data in real time on the autopilot competing in the UAS Challenge 2026.

## The competition

The UAS Challenge is an international student competition run by the Institution of Mechanical Engineers (IMechE) in the UK. University teams design, build and fly an unmanned aircraft system that has to carry out its missions autonomously, and they are judged both on the engineering design and on flight performance. A reliable state estimate is what makes that autonomy possible.

## The problem

Every decision of the autopilot, in vertical and in fixed-wing flight, starts from the state estimate. The previous estimator used a 26-state rigid-body model driven by the commanded force and torque, so it needed actuator inputs and airframe mass and inertia data. The new filter is purely kinematic: it needs only the sensors, and it is generated as C code and compiled into a Simulink S-Function, so the same filter runs in the Software-In-The-Loop model and on the autopilot.

## Approach

| | |
|---|---|
| **State (20)** | position (3), attitude quaternion (4), velocity (3), body angular rate (3), acceleration (3), GNSS altitude bias (1), accelerometer bias (3) |
| **Measurements (17)** | GNSS position (3), barometric altitude (1), GNSS velocity (3), accelerometer (3), AHRS quaternion (4), gyroscope (3) |
| **Process model** | Kinematic model with quaternion kinematics `q_dot = ½ Ω(ω) q`; angular rate, acceleration and biases as random walks |
| **Discretization** | Explicit classical RK4, quaternion renormalized after every prediction |
| **Measurement model** | GNSS lever arm on position and velocity, vertical GNSS bias, barometer lever arm, accelerometer as body-frame specific force with bias |
| **Jacobians** | Built symbolically by CasADi automatic differentiation of the full discrete step and of `h(x)` |
| **Update** | Kalman gain with a regularized innovation covariance, Joseph-form covariance update with explicit symmetrization, quaternion renormalized again after the correction |
| **Implementation** | One CasADi function `(x̂, y, P) → (x̂⁺, P⁺)`, generated as C and compiled with `mex` into the autopilot's Simulink S-Function |

## Results

| | |
|---|---|
| EKF state vector | 20 states |
| Measurement vector | 17 dimensions (IMU, GNSS, barometer, AHRS) |
| Legacy estimator | replaced by the new 20-state EKF |
| Deployment | filter compiled to C and run in real time on the UAS Challenge autopilot |
| Validation | closed-loop Software-In-The-Loop with the team autopilot, across vertical and fixed-wing flight modes |

## Repository layout

```
snippets/01_state_and_measurements.m   state and measurement vectors, dimensions, noise covariance structure
snippets/02_process_model_rk4.m        kinematic model, quaternion kinematics, RK4 step, normalization
snippets/03_measurement_model.m        h(x) with lever arms and biases, Jacobians via CasADi
snippets/04_predict_update.m           prediction, Kalman gain, Joseph-form update, quaternion renormalization
snippets/05_codegen.m                  EKF step as one function, C code generation, S-Function build
```

These are excerpts of my estimator work, not a runnable project: the full autopilot (Simulink models, SITL framework, airframe parameters) lives in the team's private repository. Each file starts with a header that says where it comes from and what was left out.

Parts of the pipeline that teammates wrote are not copied here:

- **S-Function wrapper** (`ekf_sfun.c`, team code): the C wrapper that exposes the generated filter as a Simulink block.
- **CasADi code-generation setup** (team code): the `CodeGenerator` configuration with Simulink data types.
- **Airframe parameters and tuning** (sample time, sensor lever arms, noise covariances) are not published and appear as placeholders.

## Context

Sapienza Flight Team (SASA), Sapienza University of Rome, 2025 – 2026.

**Author:** Federico Falconi · [LinkedIn](https://www.linkedin.com/in/federico-falconi/)
