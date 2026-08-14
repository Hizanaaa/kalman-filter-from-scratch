# Extended Kalman Filter

This document describes the mathematical formulation of the Extended Kalman Filter (EKF) used for nonlinear robot localization.

The EKF extends the Linear Kalman Filter to systems whose motion model cannot be represented by a single constant linear state-transition matrix.

In this project, the robot follows a nonlinear unicycle motion model.

---

## 1. Why an Extended Kalman Filter?

The Linear Kalman Filter assumes that the system can be described using linear equations:

$$x_k = F x_{k-1} + w_k$$

However, a mobile robot moving according to its heading has nonlinear dynamics.

For example:

$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t$$

and:

$$y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t$$

The terms $\cos(\theta)$ and $\sin(\theta)$ make the motion model nonlinear.

The Extended Kalman Filter handles this by locally linearizing the nonlinear model around the current state.

---

## 2. State Representation

The original nonlinear EKF uses the five-dimensional state:

$$\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \\ v_k \\ \omega_k \end{bmatrix}$$

where:

- $x_k$: x-position
- $y_k$: y-position
- $\theta_k$: robot heading
- $v_k$: linear velocity
- $\omega_k$: angular velocity

The filter therefore estimates both the robot's pose and its motion variables.

---

## 3. Nonlinear Motion Model

The robot follows a unicycle model.

The nonlinear state-transition function is:

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k) + \mathbf{w}_k$$

where:

$$f(\mathbf{x}_k) = \begin{bmatrix}
x_k + v_k \cos(\theta_k) \Delta t \\
y_k + v_k \sin(\theta_k) \Delta t \\
\theta_k + \omega_k \Delta t \\
v_k \\
\omega_k
\end{bmatrix}$$

The individual equations are:

$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t$$

$$y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t$$

$$\theta_{k+1} = \theta_k + \omega_k \Delta t$$

$$v_{k+1} = v_k$$

$$\omega_{k+1} = \omega_k$$

The last two equations assume that velocity and angular velocity remain approximately constant over one timestep.

---

## 4. Jacobian of the Motion Model

Because $f(x)$ is nonlinear, the EKF uses a Jacobian to approximate the nonlinear model locally as a linear system.

The Jacobian is:

$$F_k = \frac{\partial f}{\partial x}$$

For the unicycle model:

$$F_k = \begin{bmatrix}
1 & 0 & -v_k \sin(\theta_k) \Delta t & \cos(\theta_k) \Delta t & 0 \\
0 & 1 & v_k \cos(\theta_k) \Delta t & \sin(\theta_k) \Delta t & 0 \\
0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

The Jacobian changes at every timestep because it depends on $v_k$ and $\theta_k$.

This is the key difference from the Linear Kalman Filter, whose transition matrix remains constant for a fixed timestep.

---

## 5. Deriving the Jacobian

Consider the x-position equation:

$$f_1 = x + v\cos(\theta)\Delta t$$

Its derivatives are:

$$\frac{\partial f_1}{\partial x} = 1$$

$$\frac{\partial f_1}{\partial y} = 0$$

$$\frac{\partial f_1}{\partial \theta} = -v\sin(\theta)\Delta t$$

$$\frac{\partial f_1}{\partial v} = \cos(\theta)\Delta t$$

$$\frac{\partial f_1}{\partial \omega} = 0$$

For the y-position equation:

$$f_2 = y + v\sin(\theta)\Delta t$$

the derivatives are:

$$\frac{\partial f_2}{\partial x} = 0$$

$$\frac{\partial f_2}{\partial y} = 1$$

$$\frac{\partial f_2}{\partial \theta} = v\cos(\theta)\Delta t$$

$$\frac{\partial f_2}{\partial v} = \sin(\theta)\Delta t$$

$$\frac{\partial f_2}{\partial \omega} = 0$$

For heading:

$$f_3 = \theta + \omega\Delta t$$

so:

$$\frac{\partial f_3}{\partial \theta} = 1$$

and:

$$\frac{\partial f_3}{\partial \omega} = \Delta t$$

The velocity and angular-velocity states are modeled as constant:

$$\frac{\partial f_4}{\partial v} = 1$$

$$\frac{\partial f_5}{\partial \omega} = 1$$

These derivatives form the complete EKF Jacobian.

---

## 6. EKF Prediction

The EKF first propagates the state through the nonlinear model:

$$\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1})$$

The covariance is propagated using the Jacobian evaluated at the current state:

$$P_k^- = F_k P_{k-1} F_k^T + Q$$

where:

- $P_k^-$: predicted covariance
- $F_k$: nonlinear motion Jacobian
- $Q$: process covariance

Unlike the Linear Kalman Filter, the Jacobian must be recalculated as the state changes.

---

## 7. GPS Measurement Model

The GPS sensor measures the robot's position:

$$\mathbf{z}_k = \begin{bmatrix} x_k \\ y_k \end{bmatrix} + \mathbf{v}_k$$

The measurement function is:

$$h(\mathbf{x}) = \begin{bmatrix} x \\ y \end{bmatrix}$$

Its measurement Jacobian is:

$$H = \frac{\partial h}{\partial x} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 \end{bmatrix}$$

The measurement model is linear even though the motion model is nonlinear.

---

## 8. EKF Measurement Update

After prediction, the EKF compares the GPS measurement with the predicted GPS position.

The innovation is:

$$\mathbf{y}_k = \mathbf{z}_k - H \hat{\mathbf{x}}_k^-$$

The innovation covariance is:

$$S_k = H P_k^- H^T + R$$

The Kalman gain is:

$$K_k = P_k^- H^T S_k^{-1}$$

The state is updated using:

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k \mathbf{y}_k$$

The covariance is updated using:

$$P_k = (I - K_k H) P_k^-$$

Therefore, the EKF retains the same general predict-update structure as the Linear Kalman Filter, but uses a nonlinear state model and a state-dependent Jacobian during prediction.

---

## 9. Angle Normalization

Robot heading is periodic.

For example, $\theta = 0$ and $\theta = 2\pi$ represent the same physical orientation.

Without normalization, the numerical value of the angle can grow outside a useful range.

The implementation normalizes heading to $[-\pi, \pi]$ after prediction.

A common normalization operation is:

$$\theta = (\theta + \pi) \bmod 2\pi - \pi$$

This ensures that equivalent orientations are represented consistently.

---

## 10. EKF Prediction-Update Cycle

The complete EKF operates as follows:

```
Previous State
      |
      v
Nonlinear Motion Model
      |
      v
Predicted State
      |
      v
Calculate Jacobian
      |
      v
Propagate Covariance
      |
      v
GPS Measurement
      |
      v
Calculate Innovation
      |
      v
Calculate Kalman Gain
      |
      v
Update State
      |
      v
Update Covariance
      |
      v
Normalize Heading
```

---

## Mathematical Formulation

### Prediction

$$\hat{x}_k^- = f(\hat{x}_{k-1})$$

$$P_k^- = F_k P_{k-1} F_k^T + Q$$

### Update

$$y_k = z_k - H \hat{x}_k^-$$

$$S_k = H P_k^- H^T + R$$

$$K_k = P_k^- H^T S_k^{-1}$$

$$\hat{x}_k = \hat{x}_k^- + K_k y_k$$

$$P_k = (I - K_k H) P_k^-$$

---

## 11. Linear KF vs EKF

| Property | Linear KF | EKF |
|---|---|---|
| Motion model | Linear | Nonlinear |
| State transition | Constant matrix | State-dependent Jacobian |
| Trigonometric motion | No | Yes |
| Heading state | No | Yes |
| Covariance propagation | $FPF^T + Q$ | $F_k P F_k^T + Q$ |
| GPS update | Linear | Linear in this implementation |
| Linearization | Not required | Required |

The EKF is therefore an extension of the Kalman Filter rather than a completely different filtering framework.

---

## 12. Experimental Evaluation

The nonlinear EKF was evaluated using a simulated robot following a curved trajectory.

### Single Run

| Metric | Result |
|---|---|
| GPS RMSE | 0.6224 m |
| EKF RMSE | 0.4793 m |
| Improvement | 23.00% |

### Multi-Run Evaluation

The EKF was evaluated over:

- 50 independent simulations
- 100 steps per simulation

The results were:

| Metric | Result |
|---|---|
| GPS RMSE | 0.7002 ± 0.0378 m |
| EKF RMSE | 0.5596 ± 0.0406 m |
| RMSE Improvement | 20.12 ± 3.05% |

The multi-run results indicate that the EKF improves localization across different noise realizations rather than only under one particular simulation.

---

## 13. Interpretation

The EKF is particularly useful when the robot's motion depends on its orientation.

A linear constant-velocity model assumes that motion can be described using fixed relationships between position and velocity.

The nonlinear model instead accounts for the robot's heading:

- $v\cos(\theta)$ determines motion in the x-direction
- $v\sin(\theta)$ determines motion in the y-direction

This allows the estimator to represent curved trajectories more naturally.

However, because the EKF relies on local linearization, its accuracy depends on the quality of the current state estimate and the validity of the local approximation.

---

## 14. Implementation

The original nonlinear EKF is implemented in:

```
src/filters/ekf.py
```

The nonlinear simulation is implemented in:

```
src/simulation/nonlinear_simulator.py
```

The EKF experiment is implemented in:

```
src/ekf_experiment.py
```

The multi-run evaluation is implemented in:

```
src/evaluate_ekf.py
```

---

## 15. Relation to the Odometry EKF

The project also contains a separate EKF designed specifically for odometry sensor fusion.

The two formulations should not be confused.

### Original EKF

State:

$$[x, y, \theta, v, \omega]^T$$

Velocity and angular velocity are part of the estimated state.

### Odometry EKF

State:

$$[x, y, \theta]^T$$

Odometry provides:

$$u = [v_{odom}, \omega_{odom}]^T$$

as control input.

The odometry EKF therefore propagates uncertainty from the measured motion inputs rather than estimating $v$ and $\omega$ as state variables.

The odometry formulation is documented separately in the project's failure mode and sensor-fusion documentation.

---

## 16. Limitations

The EKF implementation uses a simplified unicycle model and simulated sensor measurements.

It does not currently model:

- wheel slip
- complex vehicle dynamics
- IMU bias
- GPS multipath
- asynchronous sensors
- measurement outliers
- severe nonlinearities
- automatic adaptive noise estimation

These provide opportunities for future extensions of the project.
