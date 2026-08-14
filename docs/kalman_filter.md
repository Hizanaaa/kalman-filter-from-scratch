# Linear Kalman Filter

This document describes the mathematical formulation and implementation of the Linear Kalman Filter used for 2D robot localization.

The filter estimates the position and velocity of a robot using a constant velocity motion model and noisy GPS position measurements.

---

## 1. Problem Formulation

The robot moves in a two-dimensional environment.

The available sensor is GPS, which provides noisy measurements of the robot's position:

$$z_k = \begin{bmatrix} x_k \\ y_k \end{bmatrix} + v_k$$

where:

- $x_k, y_k$ are the true robot coordinates
- $v_k$ is measurement noise

The objective is to estimate the robot's position and velocity while reducing the effect of GPS measurement noise.

The Kalman Filter combines:

1. A mathematical model of robot motion.
2. Noisy sensor measurements.
3. An estimate of uncertainty in both the model and measurements.

---

## 2. State Representation

The robot state is represented as:

$$\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ v_{x,k} \\ v_{y,k} \end{bmatrix}$$

where:

- $x_k$: x-position
- $y_k$: y-position
- $v_{x,k}$: velocity in the x-direction
- $v_{y,k}$: velocity in the y-direction

The filter therefore estimates four quantities at every timestep.

---

## 3. Constant-Velocity Motion Model

The robot is assumed to move with approximately constant velocity over a single timestep.

For a timestep $\Delta t$:

$$x_{k+1} = x_k + v_{x,k} \Delta t$$

$$y_{k+1} = y_k + v_{y,k} \Delta t$$

$$v_{x,k+1} = v_{x,k}$$

$$v_{y,k+1} = v_{y,k}$$

These equations can be written in matrix form:

$$\mathbf{x}_{k+1} = F \mathbf{x}_k$$

where the state transition matrix is:

$$F = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

For the implementation, $\Delta t = 1.0$ seconds.

---

## 4. Process Noise

The constant-velocity model is an approximation.

In a real robot, velocity can change because of:

- acceleration
- braking
- turning
- wheel slip
- disturbances
- model inaccuracies

These effects are represented using process noise.

The process covariance matrix is:

$$Q$$

In the current implementation, the process covariance is initialized as:

$$Q = qI_4$$

where:

- $q$ is the process variance
- $I_4$ is the $4 \times 4$ identity matrix

The process noise determines how much uncertainty is assigned to the motion model.

A larger $Q$ means the filter trusts the motion model less.

A smaller $Q$ means the filter assumes the motion model is more reliable.

---

## 5. State Prediction

The first stage of the Kalman Filter is prediction.

Given the previous state estimate:

$$\hat{\mathbf{x}}_{k-1}$$

the predicted state is:

$$\hat{\mathbf{x}}_k^- = F \hat{\mathbf{x}}_{k-1}$$

The superscript $-$ indicates that this is the estimate before incorporating the new measurement.

The predicted covariance is:

$$P_k^- = F P_{k-1} F^T + Q$$

where:

- $P_{k-1}$: previous state covariance
- $F$: state transition matrix
- $Q$: process covariance

The covariance describes the filter's uncertainty about its predicted state.

---

## 6. GPS Measurement Model

GPS measures only position.

Therefore the measurement vector is:

$$\mathbf{z}_k = \begin{bmatrix} x_k \\ y_k \end{bmatrix}$$

The measurement model is:

$$\mathbf{z}_k = H \mathbf{x}_k + \mathbf{v}_k$$

where:

$$H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

The matrix $H$ selects the position components from the full state.

Velocity is not directly measured by GPS in this simplified model.

---

## 7. Measurement Noise

GPS measurements contain noise.

The measurement covariance is represented by:

$$R$$

In the implementation:

$$R = rI_2$$

where $r$ is the GPS measurement variance.

A larger $R$ means the GPS measurements are considered less reliable.

A smaller $R$ means the filter places more trust in GPS.

---

## 8. Innovation

After prediction, the filter compares the actual GPS measurement with the predicted measurement.

The predicted measurement is:

$$\hat{\mathbf{z}}_k = H \hat{\mathbf{x}}_k^-$$

The innovation, or measurement residual, is:

$$\mathbf{y}_k = \mathbf{z}_k - H \hat{\mathbf{x}}_k^-$$

The innovation represents the difference between:

- what GPS measured
- what the filter predicted GPS should measure

A large innovation means the measurement and prediction disagree strongly.

---

## 9. Innovation Covariance

The uncertainty of the innovation is:

$$S_k = H P_k^- H^T + R$$

This combines:

- uncertainty in the predicted state
- uncertainty in the GPS measurement

---

## 10. Kalman Gain

The Kalman gain determines how strongly the measurement should influence the state estimate.

It is calculated as:

$$K_k = P_k^- H^T S_k^{-1}$$

The Kalman gain automatically balances model confidence and sensor confidence.

If the prediction is very uncertain and GPS is reliable, the measurement has more influence.

If the prediction is reliable and GPS is noisy, the measurement has less influence.

---

## 11. State Update

The predicted state is corrected using the innovation:

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k \mathbf{y}_k$$

or equivalently:

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k \left( \mathbf{z}_k - H \hat{\mathbf{x}}_k^- \right)$$

This produces the final state estimate for timestep $k$.

---

## 12. Covariance Update

After incorporating the measurement, the state uncertainty is reduced:

$$P_k = (I - K_k H) P_k^-$$

where $I$ is the identity matrix.

The covariance update is important because it allows the filter to maintain an explicit representation of its confidence.

---

## 13. Complete Kalman Filter Cycle

The complete filtering process is:

```
Previous State Estimate
        |
        v
     PREDICT
        |
        +----> Predicted State
        |
        +----> Predicted Covariance
        |
        v
   GPS Measurement
        |
        v
     UPDATE
        |
        +----> Innovation
        |
        +----> Kalman Gain
        |
        v
 Updated State Estimate
        |
        v
 Updated Covariance
```

## Mathematical Formulation

### Prediction

$$\hat{x}_k^- = F \hat{x}_{k-1}$$

$$P_k^- = F P_{k-1} F^T + Q$$

### Measurement Update

$$y_k = z_k - H \hat{x}_k^-$$

$$S_k = H P_k^- H^T + R$$

$$K_k = P_k^- H^T S_k^{-1}$$

$$\hat{x}_k = \hat{x}_k^- + K_k y_k$$

$$P_k = (I - K_k H) P_k^-$$

The filter repeats these two stages for every measurement timestep.

---

## 14. Why the Filter Improves GPS Localization

Raw GPS measurements contain random noise.

If the robot is moving approximately according to the assumed motion model, the filter can use the previous state to predict where the robot should be.

Instead of directly using $z_k$ as the robot position, the Kalman Filter produces a weighted estimate using both:

- **Prediction**
- **Measurement**

The weighting is determined by the Kalman gain.

This allows the estimate to be smoother than raw GPS while still responding to new measurements.

---

## 15. Experimental Evaluation

The linear Kalman Filter was evaluated using:

- 50 independent simulation runs
- 100 timesteps per run
- Noisy GPS measurements
- A constant-velocity motion model

The resulting localization performance was:

| Metric | Result |
|---|---|
| GPS RMSE | 0.7055 ± 0.0380 m |
| Kalman RMSE | 0.4908 ± 0.0352 m |
| RMSE Improvement | 30.44 ± 3.17% |

The Kalman Filter therefore reduced the average localization error relative to raw GPS measurements.

The multi-run evaluation is important because it demonstrates that the improvement is not based on a single random simulation.

---

## 16. GPS Dropout

A separate experiment removes GPS measurements for steps 40–69.

During this period, the filter must rely entirely on its motion model.

The measured behavior was:

| Metric | Before Dropout | During Dropout | After Recovery |
|---|---|---|---|
| Position Error | 0.4923 m | 13.4045 m | 1.3628 m |
| Covariance Trace | 0.7526 | 1102.0521 | 1.8278 |

The large increase in covariance reflects the growing uncertainty of the prediction when no absolute position measurements are available.

When GPS returns, the measurement update reduces uncertainty and corrects the state estimate.

A detailed discussion is available in: *Failure Mode Analysis*.

---

## 17. Implementation

The Linear Kalman Filter implementation is located in:

```
src/filters/kalman_filter.py
```

The main simulation and evaluation scripts are:

```
src/main.py
src/evaluate.py
src/failure_modes.py
```

The implementation uses NumPy for matrix operations but does not use an external Kalman Filter library.

---

## 18. Limitations

The current linear model assumes constant velocity.

Therefore it does not explicitly model:

- acceleration
- turning
- nonlinear motion
- wheel slip
- real-world sensor effects

These limitations motivate the Extended Kalman Filter and sensor-fusion experiments implemented elsewhere in the project.
