# Kalman & Extended Kalman Filter Localization Project

## 1. Linear Kalman Filter

The first implementation uses a constant-velocity model.

### State

The robot state is:

$$x = \begin{bmatrix} x \\ y \\ v_x \\ v_y \end{bmatrix}$$

where:

- $x, y$ are position
- $v_x, v_y$ are velocity

### Motion Model

For timestep $\Delta t$:

$$x_{k+1} = x_k + v_{x,k} \Delta t$$
$$y_{k+1} = y_k + v_{y,k} \Delta t$$
$$v_{x,k+1} = v_{x,k}$$
$$v_{y,k+1} = v_{y,k}$$

The corresponding transition matrix is:

$$F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### GPS Measurement Model

GPS measures only position:

$$z_k = \begin{bmatrix} x_k \\ y_k \end{bmatrix} + v_k$$

with:

$$H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

### Linear KF Multi-Run Evaluation

The filter was evaluated over:

- 50 independent simulations
- 100 steps per simulation

| Metric | Result |
|---|---|
| GPS RMSE | 0.7055 ± 0.0380 m |
| Kalman RMSE | 0.4908 ± 0.0352 m |
| RMSE Improvement | 30.44 ± 3.17% |

The results show that the Kalman Filter consistently improves localization accuracy compared with raw GPS measurements.

---

## 2. Extended Kalman Filter

The nonlinear experiment uses a unicycle-style robot model.

### State

The original nonlinear EKF uses:

$$x = \begin{bmatrix} x \\ y \\ \theta \\ v \\ \omega \end{bmatrix}$$

where:

- $x, y$: position
- $\theta$: heading
- $v$: linear velocity
- $\omega$: angular velocity

### Nonlinear Motion Model

$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t$$
$$y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t$$
$$\theta_{k+1} = \theta_k + \omega_k \Delta t$$
$$v_{k+1} = v_k$$
$$\omega_{k+1} = \omega_k$$

Because the motion model contains $\sin(\theta)$ and $\cos(\theta)$, the EKF linearizes the model around the current state.

### Jacobian

The state-transition Jacobian is:

$$F_k = \begin{bmatrix}
1 & 0 & -v\sin(\theta)\Delta t & \cos(\theta)\Delta t & 0 \\
0 & 1 & v\cos(\theta)\Delta t & \sin(\theta)\Delta t & 0 \\
0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

The Jacobian changes with the estimated state at every timestep.

### Prediction

The nonlinear state prediction is:

$$\hat{x}_k^- = f(\hat{x}_{k-1})$$

and covariance is propagated using:

$$P_k^- = F_k P_{k-1} F_k^T + Q$$

### GPS Update

GPS observes $x$ and $y$:

$$H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 \end{bmatrix}$$

The innovation is:

$$y_k = z_k - H \hat{x}_k^-$$

The innovation covariance is:

$$S_k = H P_k^- H^T + R$$

The Kalman gain is:

$$K_k = P_k^- H^T S_k^{-1}$$

The state update is:

$$\hat{x}_k = \hat{x}_k^- + K_k y_k$$

The covariance update is:

$$P_k = (I - K_k H) P_k^-$$

Heading is normalized to $[-\pi, \pi]$ after prediction.

### EKF Evaluation

**Single Run**

| Metric | Result |
|---|---|
| GPS RMSE | 0.6224 m |
| EKF RMSE | 0.4793 m |
| Improvement | 23.00% |

**Multi-Run Evaluation**

50 independent simulations with 100 steps per simulation:

| Metric | Result |
|---|---|
| GPS RMSE | 0.7002 ± 0.0378 m |
| EKF RMSE | 0.5596 ± 0.0406 m |
| RMSE Improvement | 20.12 ± 3.05% |

The multi-run evaluation indicates that the improvement is not dependent on a single favorable noise realization.

---

## 3. Odometry Sensor

The project also introduces a simulated odometry sensor.

The sensor measures:

$$u = \begin{bmatrix} v \\ \omega \end{bmatrix}$$

where $v$ is linear velocity and $\omega$ is angular velocity.

The measurements contain:

- Gaussian measurement noise
- Slowly changing velocity bias
- Slowly changing angular-velocity bias

The bias follows a random-walk model:

$$b_k = b_{k-1} + w_k$$

This allows the experiment to reproduce realistic accumulated odometry drift.

---

## 4. Odometry-Driven EKF

For sensor fusion, a separate 3-state EKF is used.

### State

$$x = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}$$

Unlike the original 5-state EKF, velocity and angular velocity are treated as measured control inputs rather than state variables.

The control input is:

$$u = \begin{bmatrix} v_{odom} \\ \omega_{odom} \end{bmatrix}$$

### Motion Model

$$x_{k+1} = x_k + v_{odom} \cos(\theta_k) \Delta t$$
$$y_{k+1} = y_k + v_{odom} \sin(\theta_k) \Delta t$$
$$\theta_{k+1} = \theta_k + \omega_{odom} \Delta t$$

### Jacobian

The state Jacobian is:

$$F = \begin{bmatrix}
1 & 0 & -v\sin(\theta)\Delta t \\
0 & 1 & v\cos(\theta)\Delta t \\
0 & 0 & 1
\end{bmatrix}$$

Odometry uncertainty is propagated using a control-noise Jacobian $G$:

$$P_{k+1} = F P_k F^T + G Q_u G^T$$

This explicitly accounts for uncertainty in the measured motion input.

---

## 5. GPS + Odometry Fusion

With both GPS and odometry available, the corrected 3-state EKF achieves:

| Metric | Result |
|---|---|
| GPS RMSE | 0.6224 m |
| EKF RMSE | 0.5607 m |
| RMSE Improvement | 9.91% |

The improvement is modest because GPS is already available at every timestep. The main advantage of odometry becomes more apparent during GPS outages.

---

## 6. Failure Mode Analysis

The project evaluates estimator behavior when GPS becomes unavailable.

### Linear KF GPS Dropout

GPS is removed for steps 40–69.

| Metric | Before Dropout | During Dropout | After Recovery |
|---|---|---|---|
| Position Error | 0.4923 m | 13.4045 m | 1.3628 m |
| Covariance Trace | 0.7526 | 1102.0521 | 1.8278 |

Without GPS, the linear filter relies on its motion model. Prediction errors accumulate and uncertainty increases.

When GPS returns, the measurement update reduces uncertainty and pulls the estimate back toward the true trajectory.

### Odometry Drift + GPS Dropout

The nonlinear sensor-fusion experiment uses:

- 100 total steps
- GPS dropout from steps 40–69
- Noisy GPS
- Noisy odometry
- Slowly drifting odometry bias

**Results**

| Metric | Before Dropout | During Dropout | After Recovery |
|---|---|---|---|
| Position Error | 0.1131 m | 1.7065 m | 0.5383 m |
| Covariance Trace | 0.5730 | 1288.0796 | 1.2440 |

During the outage, the EKF relies on odometry alone.

The position error increases: $0.1131 \rightarrow 1.7065$ m

while uncertainty increases: $0.5730 \rightarrow 1288.0796$

When GPS returns, the filter reduces its uncertainty and the position error falls to 0.5383 m.

This demonstrates an important property of probabilistic localization: the estimator should become less confident when reliable absolute measurements disappear.

### Failure-Mode Visualizations

- Trajectory
- Position Error
- Covariance Growth

See `docs/failure_modes.md` for the detailed analysis.

---

## 7. Results Summary

| Experiment | GPS RMSE | Filter RMSE | Improvement |
|---|---|---|---|
| Linear KF | 0.7055 m | 0.4908 m | 30.44% |
| Nonlinear EKF | 0.7002 m | 0.5596 m | 20.12% |
| GPS + Odometry EKF | 0.6224 m | 0.5607 m | 9.91% |

The results demonstrate that filtering improves localization compared with raw GPS measurements, while failure-mode experiments show how uncertainty and estimation error evolve when absolute measurements are unavailable.

---

## 8. Automated Tests

The project includes automated tests for the odometry sensor and odometry-driven EKF.

Run:

```
pytest
```

Current test result:

```
3 passed
```

The tests verify:

- Odometry measurement generation
- EKF nonlinear prediction
- GPS measurement update behavior

---

## 9. Reproducibility

Experiments use fixed random seeds where appropriate so that individual experiments can be reproduced.

The main evaluation scripts include:

```
src/evaluate.py
src/evaluate_ekf.py
src/ekf_experiment.py
src/odometry_experiment.py
src/failure_modes.py
```

---

## 10. Limitations

This is a simulation-based localization project and does not represent every failure mode encountered by a physical robot.

Current limitations include:

- Simplified robot dynamics
- Simplified GPS measurement model
- Simplified odometry error model
- No real IMU data
- No wheel-slip model
- No GPS multipath or realistic urban canyon effects
- Fixed filter noise parameters
- No online fault-detection mechanism

The covariance growth observed during GPS outages is therefore an estimator behavior under the chosen simulated noise model, rather than a prediction of real-world uncertainty.

---

## 11. Future Work

Potential extensions include:

- IMU integration
- Wheel encoder modeling
- Adaptive noise estimation
- GPS outlier rejection
- Innovation-based fault detection
- Sensor fault classification
- Robust Kalman filtering
- Interacting Multiple Model (IMM) filtering
- Real-world ROS 2 integration
- Evaluation on real robot datasets