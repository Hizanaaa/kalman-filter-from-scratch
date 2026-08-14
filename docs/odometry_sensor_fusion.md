# Odometry Sensor Fusion

This document describes the odometry sensor model and the 3-state Extended Kalman Filter used to combine odometry with GPS for nonlinear robot localization.

This formulation is separate from the original five-state EKF.

The original EKF estimates:

$$[x, y, \theta, v, \omega]^T$$

whereas the odometry-fusion EKF estimates:

$$[x, y, \theta]^T$$

and uses measured odometry velocity and angular velocity as control inputs.

---

## 1. Motivation

GPS provides absolute position information, but it is noisy and can temporarily become unavailable.

Odometry provides short-term motion information at a high rate, but its measurements accumulate error over time.

The two sensors therefore have complementary characteristics:

| Sensor | Strength | Weakness |
|---|---|---|
| GPS | Absolute position | Noise and possible outages |
| Odometry | Short-term motion | Accumulated drift |

The objective is to combine both sources so that the robot can continue estimating its motion when GPS measurements are temporarily unavailable.

---

## 2. Odometry Sensor Model

The simulated odometry sensor measures the robot's linear and angular velocity.

The measurement is:

$$\mathbf{u}_k = \begin{bmatrix} v_k^{odom} \\ \omega_k^{odom} \end{bmatrix}$$

where:

- $v_k^{odom}$: measured linear velocity
- $\omega_k^{odom}$: measured angular velocity

The measurements are noisy versions of the true robot motion.

---

### 2.1 Measurement Noise

The measured velocity is modeled as:

$$v_k^{odom} = v_k^{true} + b_{v,k} + n_{v,k}$$

and measured angular velocity as:

$$\omega_k^{odom} = \omega_k^{true} + b_{\omega,k} + n_{\omega,k}$$

where:

- $b_v$: velocity bias
- $b_\omega$: angular velocity bias
- $n_v$: velocity measurement noise
- $n_\omega$: angular velocity measurement noise

The random measurement noise is modeled as approximately Gaussian.

---

### 2.2 Bias Drift

The odometry bias is allowed to change gradually over time.

The bias follows a random-walk model:

$$b_k = b_{k-1} + w_k$$

where $w_k$ is a small random process disturbance.

Therefore, even if the instantaneous odometry measurements appear accurate, small errors can accumulate over many timesteps.

This produces realistic long-term odometry drift.

---

### 2.3 Sensor Test

The odometry sensor was tested independently before being integrated into the EKF.

Example measurements include:

```
Step 00 | True v=2.000 | Measured v=2.038
        | True omega=0.100 | Measured omega=0.108

Step 01 | True v=2.000 | Measured v=2.003
        | True omega=0.100 | Measured omega=0.094
```

The measurements fluctuate around the true values, demonstrating that the sensor model introduces noise without completely corrupting the underlying motion signal.

---

## 3. Odometry-Driven EKF

The sensor-fusion EKF uses a three-dimensional state:

$$x_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}$$

where:

- $x_k$: robot x-position
- $y_k$: robot y-position
- $\theta_k$: robot heading

Unlike the original five-state EKF, $v$ and $\omega$ are not estimated as state variables.

Instead, they are supplied by the odometry sensor as control inputs.

---

## 4. Control Input

The control vector is:

$$u_k = \begin{bmatrix} v_k^{odom} \\ \omega_k^{odom} \end{bmatrix}$$

The EKF therefore uses the measured robot motion to predict the next state.

This is important because odometry errors are now directly connected to state uncertainty.

---

## 5. Nonlinear Motion Model

The robot follows a unicycle model:

$$x_{k+1} = x_k + v_k^{odom} \cos(\theta_k) \Delta t$$
$$y_{k+1} = y_k + v_k^{odom} \sin(\theta_k) \Delta t$$
$$\theta_{k+1} = \theta_k + \omega_k^{odom} \Delta t$$

In vector form:

$$x_{k+1} = f(x_k, u_k) + w_k$$

where:

$$f(x, u) = \begin{bmatrix} x + v\cos(\theta)\Delta t \\ y + v\sin(\theta)\Delta t \\ \theta + \omega \Delta t \end{bmatrix}$$

---

## 6. State Jacobian

The EKF linearizes the motion model around the current state.

The state Jacobian is:

$$F_k = \frac{\partial f}{\partial x}$$

which gives:

$$F_k = \begin{bmatrix}
1 & 0 & -v_k \sin(\theta_k) \Delta t \\
0 & 1 & v_k \cos(\theta_k) \Delta t \\
0 & 0 & 1
\end{bmatrix}$$

The Jacobian depends on the current heading and measured velocity.

Therefore it changes as the robot moves.

---

## 7. Control-Noise Jacobian

Because velocity and angular velocity are measured rather than perfectly known, their uncertainty must also be propagated.

The control Jacobian is:

$$G_k = \frac{\partial f}{\partial u}$$

For:

$$u = \begin{bmatrix} v \\ \omega \end{bmatrix}$$

the Jacobian is:

$$G_k = \begin{bmatrix}
\cos(\theta_k) \Delta t & 0 \\
\sin(\theta_k) \Delta t & 0 \\
0 & \Delta t
\end{bmatrix}$$

This matrix describes how uncertainty in the odometry measurements affects the predicted robot state.

---

## 8. Odometry Process Noise

Let the covariance of the odometry control measurements be:

$$Q_u = \begin{bmatrix} \sigma_v^2 & 0 \\ 0 & \sigma_\omega^2 \end{bmatrix}$$

where:

- $\sigma_v^2$: variance of velocity measurement noise
- $\sigma_\omega^2$: variance of angular velocity measurement noise

The process uncertainty contributed by odometry is:

$$Q_k^{odom} = G_k Q_u G_k^T$$

The covariance prediction therefore becomes:

$$P_k^- = F_k P_{k-1} F_k^T + G_k Q_u G_k^T$$

This is different from simply adding a fixed covariance matrix because the effect of odometry uncertainty depends on the robot's current orientation.

---

## 9. GPS Measurement Model

GPS measures the robot's x and y position:

$$z_k = \begin{bmatrix} x_k \\ y_k \end{bmatrix} + v_k$$

The measurement function is:

$$h(x) = \begin{bmatrix} x \\ y \end{bmatrix}$$

The measurement Jacobian is therefore:

$$H = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}$$

GPS does not directly measure heading.

---

## 10. Prediction Step

At every timestep, the EKF first receives odometry measurements:

$$v_k^{odom}, \omega_k^{odom}$$

The state is predicted using:

$$\hat{x}_k^- = f(\hat{x}_{k-1}, u_k)$$

and covariance is propagated using:

$$P_k^- = F_k P_{k-1} F_k^T + G_k Q_u G_k^T$$

This allows the filter to continue estimating the robot's motion even when GPS is unavailable.

---

## 11. GPS Measurement Update

When GPS is available, the predicted state is corrected.

The innovation is:

$$y_k = z_k - H \hat{x}_k^-$$

The innovation covariance is:

$$S_k = H P_k^- H^T + R$$

The Kalman gain is:

$$K_k = P_k^- H^T S_k^{-1}$$

The updated state is:

$$\hat{x}_k = \hat{x}_k^- + K_k y_k$$

The covariance is:

$$P_k = (I - K_k H) P_k^-$$

Therefore:

```
Odometry
   |
   v
Prediction
   |
   +-------------------+
   |                   |
   | GPS available?    |
   |                   |
   +----Yes------------+
   |                   |
   v                   v
State prediction    GPS update
   |                   |
   +---------+---------+
             |
             v
       Updated estimate
```

---

## 12. GPS + Odometry Performance

With both GPS and odometry available, the sensor-fusion EKF produced:

| Metric | Result |
|---|---|
| GPS RMSE | 0.6224 m |
| EKF RMSE | 0.5607 m |
| RMSE Improvement | 9.91% |

The improvement is smaller than the linear KF improvement because GPS is already available at every timestep.

The main benefit of odometry becomes more important when GPS measurements are unavailable.

---

## 13. GPS Dropout Experiment

GPS measurements were removed for steps 40–69.

This creates a 30-step GPS outage.

During this period, the EKF must rely on odometry to propagate the state.

The results were:

| Metric | Before Dropout | During Dropout | After Recovery |
|---|---|---|---|
| Position Error | 0.1131 m | 1.7065 m | 0.5383 m |
| Covariance Trace | 0.5730 | 1288.0796 | 1.2440 |

---

## 14. Position Error During GPS Outage

Before the outage:

$$e_{before} = 0.1131 \text{ m}$$

During the outage:

$$e_{during} = 1.7065 \text{ m}$$

The increase occurs because odometry errors accumulate while there is no absolute position measurement available to correct them.

The odometry sensor is useful for short-term motion prediction, but it is not sufficient for indefinite absolute localization.

---

## 15. Covariance Growth

The covariance trace changes from **0.5730** before the outage to **1288.0796** during the outage.

This large increase reflects the filter's growing uncertainty.

Importantly, covariance growth is not itself a failure of the EKF.

It is an expected consequence of prediction without reliable absolute measurements.

The filter is explicitly representing the fact that its estimate is becoming less certain.

---

## 16. GPS Recovery

When GPS measurements return, the EKF receives an absolute position measurement again.

The position error decreases from **1.7065 m** during the outage to **0.5383 m** after recovery.

The covariance trace also decreases from **1288.0796** to **1.2440**.

This demonstrates the corrective role of GPS in the sensor-fusion system.

The EKF does not need to restart after the outage.

Instead, the measurement update uses the newly available GPS information to correct the existing prediction.

---

## 17. Why Both Sensors Are Needed

The experiment demonstrates the complementary nature of GPS and odometry.

**GPS**

Provides:

- Absolute position
- Long-term reference
- Drift correction

But:

- Measurements are noisy
- GPS can temporarily fail

**Odometry**

Provides:

- Continuous motion information
- Short-term prediction
- Motion estimates during GPS outages

But:

- Noise accumulates
- Bias causes drift
- It cannot provide an absolute position reference

The EKF combines the strengths of both.

---

## 18. Failure-Mode Interpretation

The experiment demonstrates the following sequence:

```
GPS + Odometry
      |
      v
Low error + low uncertainty
      |
      v
GPS dropout
      |
      v
Odometry-only prediction
      |
      +----> Position error increases
      |
      +----> Covariance increases
      |
      v
GPS recovery
      |
      v
Measurement update
      |
      +----> Position error decreases
      |
      +----> Covariance decreases
```

This behavior is one of the key demonstrations of the project.

The estimator does not simply produce a position estimate; it also maintains a quantitative representation of uncertainty.

---

## 19. Failure-Mode Visualizations

The experiment generates three visualizations.

**Trajectory** — `results/odometry_dropout_trajectory.png`

This compares:

- Ground-truth trajectory
- GPS measurements
- EKF trajectory

**Position Error** — `results/odometry_dropout_error.png`

This shows how localization error changes before, during, and after the GPS outage.

**Covariance** — `results/odometry_dropout_covariance.png`

This shows the growth of EKF uncertainty during the GPS outage and its reduction after GPS recovery.

---

## 20. Implementation

The odometry sensor is implemented in:

```
src/simulation/sensors.py
```

The nonlinear simulator is implemented in:

```
src/simulation/nonlinear_simulator.py
```

The odometry-driven EKF is implemented in:

```
src/filters/odometry_ekf.py
```

The complete experiment is implemented in:

```
src/odometry_experiment.py
```

---

## 21. Limitations

The current odometry model is simulated and therefore does not reproduce all real-world effects.

It does not currently model:

- wheel slip
- encoder quantization
- wheel-radius mismatch
- wheelbase calibration errors
- mechanical backlash
- terrain-dependent errors
- IMU measurements
- asynchronous sensor timestamps

The simulated bias drift is intended to demonstrate the general behavior of accumulating odometry error rather than reproduce a specific physical odometry sensor.

---

## 22. Future Extensions

Possible extensions include:

- Wheel encoder simulation
- IMU integration
- GPS outlier rejection
- Innovation-based fault detection
- Adaptive process-noise estimation
- Sensor fault detection
- Robust EKF updates
- Real robot experiments
- ROS 2 integration
- Evaluation using real-world localization datasets
