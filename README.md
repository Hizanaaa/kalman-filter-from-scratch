# Kalman Filter From Scratch

This repository contains a complete implementation of a Linear Kalman Filter (KF) and an Extended Kalman Filter (EKF) built entirely from scratch for a simulated 2D robot localization problem.

## Objectives

- Implement the Kalman Filter without external filtering libraries
- Simulate robot motion with noisy sensors
- Estimate the robot state using probabilistic filtering
- Analyze filter performance under different failure scenarios
- Extend the implementation to an Extended Kalman Filter

## Repository Structure

```
src/        Source code
docs/       Mathematical derivations
tests/      Unit tests
data/       Simulated data
results/    Generated plots
```

## Status

Project initialization completed.

## Results

### Multi-Run Evaluation

The linear Kalman Filter was evaluated across 50 independent random
simulations, with 100 time steps per simulation.

| Metric | Result |
|---|---:|
| GPS RMSE | 0.7055 ± 0.0380 m |
| Kalman RMSE | 0.4908 ± 0.0352 m |
| RMSE Improvement | 30.44 ± 3.17% |

The results show that the Kalman Filter consistently improves localization
accuracy compared with raw GPS measurements.

### GPS Dropout

A GPS dropout experiment was performed by removing GPS measurements for
steps 40–69 of a 100-step simulation.

| Metric | Before Dropout | During Dropout | After Recovery |
|---|---:|---:|---:|
| Position Error | 0.4923 m | 13.4045 m | 1.3628 m |
| Covariance Trace | 0.7526 | 1102.0521 | 1.8278 |

During the dropout, the filter relies entirely on its motion model.
Consequently, both estimation error and uncertainty increase.

When GPS becomes available again, the measurement update rapidly reduces
uncertainty and brings the estimate back toward the true trajectory.

See [Failure Modes](docs/failure_modes.md) for the detailed analysis.


## Extended Kalman Filter

The project was extended from the linear Kalman Filter to an Extended
Kalman Filter (EKF) for nonlinear robot motion.

### Nonlinear State

The EKF uses the state:

\[
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta \\
v \\
\omega
\end{bmatrix}
\]

where:

- \(x, y\): robot position
- \(\theta\): robot heading
- \(v\): linear velocity
- \(\omega\): angular velocity

### Nonlinear Motion Model

The robot follows a unicycle motion model:

\[
x_{k+1}
=
x_k + v_k\cos(\theta_k)\Delta t
\]

\[
y_{k+1}
=
y_k + v_k\sin(\theta_k)\Delta t
\]

\[
\theta_{k+1}
=
\theta_k + \omega_k\Delta t
\]

\[
v_{k+1}=v_k
\]

\[
\omega_{k+1}=\omega_k
\]

The nonlinear terms \(\sin(\theta)\) and \(\cos(\theta)\) prevent the
motion model from being represented by a single constant state-transition
matrix.

### Jacobian

The EKF linearizes the nonlinear motion model around the current state.

The Jacobian is:

\[
F_k =
\begin{bmatrix}
1 & 0 & -v\sin(\theta)\Delta t & \cos(\theta)\Delta t & 0 \\
0 & 1 & v\cos(\theta)\Delta t & \sin(\theta)\Delta t & 0 \\
0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}
\]

Unlike the linear Kalman Filter, this Jacobian changes with the estimated
state at every timestep.

### EKF Prediction

The state is propagated through the nonlinear motion model:

\[
\hat{x}_k^- = f(\hat{x}_{k-1})
\]

The covariance is propagated using the Jacobian:

\[
P_k^- =
F_kP_{k-1}F_k^T + Q
\]

### GPS Measurement Model

GPS measures only the robot's \(x\) and \(y\) position:

\[
z_k =
\begin{bmatrix}
x_k \\
y_k
\end{bmatrix}
+ v_k
\]

The measurement Jacobian is:

\[
H =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0
\end{bmatrix}
\]

The innovation is:

\[
y_k = z_k - H\hat{x}_k^-
\]

The innovation covariance is:

\[
S_k = HP_k^-H^T + R
\]

The Kalman gain is:

\[
K_k = P_k^-H^TS_k^{-1}
\]

The state update is:

\[
\hat{x}_k =
\hat{x}_k^- + K_ky_k
\]

and the covariance update is:

\[
P_k =
(I-K_kH)P_k^-
\]

### Angle Normalization

Because heading is periodic, the estimated angle is normalized to:

\[
[-\pi,\pi]
\]

after each prediction step.

This prevents equivalent headings such as \(0\) and \(2\pi\) from being
treated as numerically different states.

## EKF Results

The EKF was evaluated using a nonlinear circular robot trajectory with
noisy GPS measurements.

### Single Run

| Metric | Result |
|---|---:|
| GPS RMSE | 0.6224 m |
| EKF RMSE | 0.4793 m |
| Improvement | 23.00% |

### Multi-Run Evaluation

The EKF was evaluated over 50 independent simulations, with 100 time steps
per simulation.

| Metric | Result |
|---|---:|
| GPS RMSE | 0.7002 ± 0.0378 m |
| EKF RMSE | 0.5596 ± 0.0406 m |
| RMSE Improvement | 20.12 ± 3.05% |

The multi-run experiment shows that the EKF consistently improves
localization accuracy across different noise realizations rather than
depending on a single favorable simulation.

### EKF Trajectory

The nonlinear experiment compares the ground-truth trajectory, noisy GPS
measurements, and EKF estimate.

The EKF follows the curved ground-truth trajectory while reducing the
variation introduced by noisy GPS measurements.

![EKF Trajectory](results/ekf_trajectory.png)
