# Kalman Filter Mathematical Derivation

This document derives the Linear Kalman Filter used in this project from the
underlying robot motion and measurement models.

The derivation covers:

- State-space representation
- Constant-velocity motion model
- State-transition matrix
- Process-noise model
- GPS measurement model
- Prediction equations
- Measurement prediction
- Innovation covariance
- Kalman gain
- State update
- Covariance update
- Interpretation of the Kalman gain

---

## 1. State-Space Representation

The robot state is defined as:

\[
\mathbf{x}_k =
\begin{bmatrix}
x_k \\
y_k \\
v_{x,k} \\
v_{y,k}
\end{bmatrix}
\]

where:

- \(x_k\): x-position at timestep \(k\)
- \(y_k\): y-position at timestep \(k\)
- \(v_{x,k}\): velocity along the x-axis
- \(v_{y,k}\): velocity along the y-axis

The general linear state-space model is:

\[
\mathbf{x}_k
=
F\mathbf{x}_{k-1}
+
\mathbf{w}_k
\]

where:

- \(F\): state-transition matrix
- \(\mathbf{w}_k\): process noise

The measurement model is:

\[
\mathbf{z}_k
=
H\mathbf{x}_k
+
\mathbf{v}_k
\]

where:

- \(H\): measurement matrix
- \(\mathbf{v}_k\): measurement noise

---

# 2. Derivation of the Motion Model

The robot is assumed to move with approximately constant velocity during
one timestep.

For a timestep \(\Delta t\):

\[
x_k
=
x_{k-1}
+
v_{x,k-1}\Delta t
\]

Similarly:

\[
y_k
=
y_{k-1}
+
v_{y,k-1}\Delta t
\]

Under the constant-velocity assumption:

\[
v_{x,k}=v_{x,k-1}
\]

and:

\[
v_{y,k}=v_{y,k-1}
\]

Therefore:

\[
\begin{bmatrix}
x_k\\
y_k\\
v_{x,k}\\
v_{y,k}
\end{bmatrix}
=
\begin{bmatrix}
x_{k-1}+v_{x,k-1}\Delta t\\
y_{k-1}+v_{y,k-1}\Delta t\\
v_{x,k-1}\\
v_{y,k-1}
\end{bmatrix}
\]

---

# 3. Derivation of the State-Transition Matrix

We want to express the motion model in the form:

\[
\mathbf{x}_k=F\mathbf{x}_{k-1}
\]

Starting with:

\[
x_k
=
1x_{k-1}
+
0y_{k-1}
+
\Delta t\,v_{x,k-1}
+
0v_{y,k-1}
\]

Therefore, the first row of \(F\) is:

\[
\begin{bmatrix}
1 & 0 & \Delta t & 0
\end{bmatrix}
\]

For the y-position:

\[
y_k
=
0x_{k-1}
+
1y_{k-1}
+
0v_{x,k-1}
+
\Delta t\,v_{y,k-1}
\]

Therefore, the second row is:

\[
\begin{bmatrix}
0 & 1 & 0 & \Delta t
\end{bmatrix}
\]

For x-velocity:

\[
v_{x,k}
=
0x_{k-1}
+
0y_{k-1}
+
1v_{x,k-1}
+
0v_{y,k-1}
\]

Therefore:

\[
\begin{bmatrix}
0 & 0 & 1 & 0
\end{bmatrix}
\]

For y-velocity:

\[
v_{y,k}
=
0x_{k-1}
+
0y_{k-1}
+
0v_{x,k-1}
+
1v_{y,k-1}
\]

Therefore:

\[
\begin{bmatrix}
0 & 0 & 0 & 1
\end{bmatrix}
\]

Combining all four rows:

\[
\boxed{
F=
\begin{bmatrix}
1 & 0 & \Delta t & 0\\
0 & 1 & 0 & \Delta t\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
}
\]

For the implementation:

\[
\Delta t=1
\]

so:

\[
F=
\begin{bmatrix}
1&0&1&0\\
0&1&0&1\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}
\]

---

# 4. Process Noise

The constant-velocity model is not perfectly accurate.

In a real system, the robot may accelerate, decelerate, or experience other
unmodeled effects.

These effects are represented using process noise:

\[
\mathbf{w}_k
\sim
\mathcal{N}(0,Q)
\]

where \(Q\) is the process-noise covariance matrix.

The state equation becomes:

\[
\boxed{
\mathbf{x}_k
=
F\mathbf{x}_{k-1}
+
\mathbf{w}_k
}
\]

The covariance \(Q\) represents uncertainty in the motion model.

If \(Q\) is too small, the filter may trust its motion model too strongly.

If \(Q\) is too large, the filter may place too little confidence in its
prediction.

---

# 5. State Covariance

The Kalman Filter maintains a covariance matrix:

\[
P_k
\]

For the four-dimensional state:

\[
P_k
\in
\mathbb{R}^{4\times4}
\]

It represents uncertainty and correlations between state variables.

A general form is:

\[
P=
\begin{bmatrix}
\sigma_x^2 & \operatorname{cov}(x,y) &
\operatorname{cov}(x,v_x) &
\operatorname{cov}(x,v_y)
\\
\operatorname{cov}(y,x) & \sigma_y^2 &
\operatorname{cov}(y,v_x) &
\operatorname{cov}(y,v_y)
\\
\operatorname{cov}(v_x,x) &
\operatorname{cov}(v_x,y) &
\sigma_{v_x}^2 &
\operatorname{cov}(v_x,v_y)
\\
\operatorname{cov}(v_y,x) &
\operatorname{cov}(v_y,y) &
\operatorname{cov}(v_y,v_x) &
\sigma_{v_y}^2
\end{bmatrix}
\]

The diagonal elements represent individual state variances.

The off-diagonal elements represent correlations between state variables.

---

# 6. Prediction of the State

Given the previous estimate:

\[
\hat{\mathbf{x}}_{k-1}
\]

the predicted state is:

\[
\boxed{
\hat{\mathbf{x}}_k^-
=
F\hat{\mathbf{x}}_{k-1}
}
\]

The superscript \(-\) means that the prediction has not yet incorporated the
new measurement.

---

# 7. Derivation of Predicted Covariance

The state prediction is:

\[
\mathbf{x}_k
=
F\mathbf{x}_{k-1}
+
\mathbf{w}_k
\]

The predicted covariance is:

\[
P_k^-
=
\operatorname{Cov}
\left(
F\mathbf{x}_{k-1}
+
\mathbf{w}_k
\right)
\]

Assuming the process noise is independent of the previous state:

\[
\operatorname{Cov}
(F\mathbf{x}_{k-1}
+
\mathbf{w}_k)
=
FP_{k-1}F^T+Q
\]

Therefore:

\[
\boxed{
P_k^-
=
FP_{k-1}F^T+Q
}
\]

This equation is important because uncertainty can increase during prediction.

---

# 8. GPS Measurement Model

GPS provides a noisy measurement of position.

The measurement vector is:

\[
\mathbf{z}_k=
\begin{bmatrix}
z_{x,k}\\
z_{y,k}
\end{bmatrix}
\]

The measurement model is:

\[
\mathbf{z}_k
=
H\mathbf{x}_k
+
\mathbf{v}_k
\]

GPS does not directly measure velocity in this simplified model.

Therefore:

\[
z_{x,k}=x_k+v_{x,k}^{gps}
\]

and:

\[
z_{y,k}=y_k+v_{y,k}^{gps}
\]

The measurement matrix must select the position components from the state.

Therefore:

\[
\boxed{
H=
\begin{bmatrix}
1&0&0&0\\
0&1&0&0
\end{bmatrix}
}
\]

---

# 9. GPS Measurement Noise

GPS measurement noise is modeled as:

\[
\mathbf{v}_k
\sim
\mathcal{N}(0,R)
\]

where \(R\) is the GPS measurement covariance.

For independent x and y measurement noise:

\[
R=
\begin{bmatrix}
\sigma_x^2&0\\
0&\sigma_y^2
\end{bmatrix}
\]

In the simplified implementation, equal measurement variance can be used:

\[
R=rI_2
\]

where \(r\) is the measurement variance.

---

# 10. Predicted Measurement

The predicted state is:

\[
\hat{\mathbf{x}}_k^-
\]

The corresponding predicted GPS measurement is:

\[
\boxed{
\hat{\mathbf{z}}_k
=
H\hat{\mathbf{x}}_k^-
}
\]

Since:

\[
H=
\begin{bmatrix}
1&0&0&0\\
0&1&0&0
\end{bmatrix}
\]

we obtain:

\[
\hat{\mathbf{z}}_k
=
\begin{bmatrix}
\hat{x}_k^-\\
\hat{y}_k^-
\end{bmatrix}
\]

Thus, the predicted measurement is simply the predicted robot position.

---

# 11. Innovation

The innovation measures the disagreement between the actual GPS measurement
and the predicted GPS measurement.

It is defined as:

\[
\boxed{
\mathbf{y}_k
=
\mathbf{z}_k
-
H\hat{\mathbf{x}}_k^-
}
\]

For GPS:

\[
\mathbf{y}_k
=
\begin{bmatrix}
z_{x,k}-\hat{x}_k^-\\
z_{y,k}-\hat{y}_k^-
\end{bmatrix}
\]

A large innovation indicates that the GPS measurement differs significantly
from the predicted position.

---

# 12. Innovation Covariance

The innovation itself has uncertainty.

The innovation covariance is:

\[
S_k
=
\operatorname{Cov}(\mathbf{y}_k)
\]

Since:

\[
\mathbf{y}_k
=
\mathbf{z}_k
-
H\hat{\mathbf{x}}_k^-
\]

and both the predicted state and measurement contain uncertainty:

\[
\boxed{
S_k
=
HP_k^-H^T+R
}
\]

The first term represents uncertainty from the predicted state.

The second term represents GPS measurement uncertainty.

---

# 13. Derivation of the Kalman Gain

The Kalman gain determines how much the measurement should influence the
predicted state.

It is defined as:

\[
\boxed{
K_k
=
P_k^-H^TS_k^{-1}
}
\]

Substituting:

\[
S_k=HP_k^-H^T+R
\]

gives:

\[
\boxed{
K_k
=
P_k^-H^T
\left(
HP_k^-H^T+R
\right)^{-1}
}
\]

The Kalman gain depends on both:

- predicted state uncertainty
- measurement uncertainty

Therefore, the filter automatically adjusts how much it trusts the GPS
measurement.

---

# 14. Interpretation of the Kalman Gain

Consider two limiting cases.

### Case 1: GPS is very noisy

If:

\[
R\rightarrow\infty
\]

then:

\[
K_k\rightarrow0
\]

The measurement has very little influence on the state estimate.

The filter primarily trusts its prediction.

### Case 2: GPS is very reliable

If:

\[
R\rightarrow0
\]

then the Kalman gain becomes larger.

The measurement has much greater influence on the updated estimate.

Therefore, the Kalman gain provides an automatic weighting mechanism between
prediction and measurement.

---

# 15. State Update

The corrected state is obtained by adding the weighted innovation to the
predicted state:

\[
\boxed{
\hat{\mathbf{x}}_k
=
\hat{\mathbf{x}}_k^-
+
K_k\mathbf{y}_k
}
\]

Substituting the innovation:

\[
\boxed{
\hat{\mathbf{x}}_k
=
\hat{\mathbf{x}}_k^-
+
K_k
\left(
\mathbf{z}_k
-
H\hat{\mathbf{x}}_k^-
\right)
}
\]

This is the fundamental correction step of the Kalman Filter.

The updated state lies between the prediction and the measurement, with the
relative influence determined by the Kalman gain.

---

# 16. Covariance Update

After incorporating the measurement, the uncertainty should generally
decrease.

The standard covariance update is:

\[
\boxed{
P_k
=
(I-K_kH)P_k^-
}
\]

where \(I\) is the identity matrix.

This update reflects the information gained from the measurement.

The measurement provides additional information about the robot's position,
so uncertainty in the state estimate is reduced.

---

# 17. Joseph Form

An alternative numerically robust covariance update is:

\[
\boxed{
P_k
=
(I-K_kH)P_k^-(I-K_kH)^T
+
K_kRK_k^T
}
\]

This form explicitly preserves the contribution of measurement noise and can
provide better numerical behavior in some implementations.

The simplified implementation uses the standard covariance update.

---

# 18. Complete Derivation

The complete Kalman Filter can therefore be summarized as follows.

## Prediction

State:

\[
\boxed{
\hat{x}_k^-=F\hat{x}_{k-1}
}
\]

Covariance:

\[
\boxed{
P_k^-=FP_{k-1}F^T+Q
}
\]

## Measurement Prediction

\[
\boxed{
\hat{z}_k=H\hat{x}_k^-
}
\]

## Innovation

\[
\boxed{
y_k=z_k-\hat{z}_k
}
\]

or:

\[
\boxed{
y_k=z_k-H\hat{x}_k^-
}
\]

## Innovation Covariance

\[
\boxed{
S_k=HP_k^-H^T+R
}
\]

## Kalman Gain

\[
\boxed{
K_k=P_k^-H^TS_k^{-1}
}
\]

## State Update

\[
\boxed{
\hat{x}_k=\hat{x}_k^-+K_ky_k
}
\]

## Covariance Update

\[
\boxed{
P_k=(I-K_kH)P_k^-
}
\]

---

# 19. Why Covariance Grows During GPS Dropout

The covariance prediction equation is:

\[
P_k^-
=
FP_{k-1}F^T+Q
\]

Notice that the measurement update is absent.

When GPS is available, the filter performs:

\[
\text{Prediction}
\rightarrow
\text{Measurement Update}
\]

The measurement update reduces uncertainty.

During a GPS outage, the filter performs only:

\[
\text{Prediction}
\rightarrow
\text{Prediction}
\rightarrow
\text{Prediction}
\rightarrow\cdots
\]

Therefore, process uncertainty continues to accumulate.

This explains the covariance growth observed in the GPS dropout experiment.

The behavior is not necessarily a failure of the filter.

Instead, it is the expected representation of increasing uncertainty when
absolute position measurements are unavailable.

---

# 20. Relationship Between Error and Covariance

The position error and covariance represent different quantities.

### Position error

Measures the difference between the estimated and true position:

\[
e_k
=
\left\|
\begin{bmatrix}
x_k^{estimate}\\
y_k^{estimate}
\end{bmatrix}
-
\begin{bmatrix}
x_k^{true}\\
y_k^{true}
\end{bmatrix}
\right\|
\]

This requires access to ground truth.

### Covariance

Represents the filter's internal estimate of uncertainty:

\[
P_k
\]

It does not require ground truth.

This distinction is important.

A filter can have:

- low error and high uncertainty
- low error and low uncertainty
- high error and low uncertainty
- high error and high uncertainty

A good estimator should ideally maintain uncertainty that reflects the actual
reliability of its estimate.

---

# 21. Connection to the Project Results

The linear GPS dropout experiment produced:

| Metric | Before Dropout | During Dropout | After Recovery |
|---|---:|---:|---:|
| Position Error | 0.4923 m | 13.4045 m | 1.3628 m |
| Covariance Trace | 0.7526 | 1102.0521 | 1.8278 |

The covariance trace is:

\[
\operatorname{tr}(P)
=
\sum_i P_{ii}
\]

and provides a simple scalar measure of total state variance.

During GPS dropout:

\[
0.7526
\rightarrow
1102.0521
\]

The large increase indicates substantial growth in estimated uncertainty.

After GPS recovery, the measurement update reduces the covariance:

\[
1102.0521
\rightarrow
1.8278
\]

At the same time, the position error decreases substantially.

This experimentally demonstrates the relationship between measurement
availability, uncertainty propagation, and state correction.

---

# 22. Summary

The Linear Kalman Filter can be understood as a repeated cycle of:

\[
\boxed{
\text{Predict}
\rightarrow
\text{Compare}
\rightarrow
\text{Weight}
\rightarrow
\text{Correct}
}
\]

The prediction step uses the robot's motion model.

The covariance prediction represents uncertainty introduced by the imperfect
motion model.

The measurement step introduces information from GPS.

The Kalman gain determines the relative influence of prediction and
measurement.

The update step produces the corrected state and reduced uncertainty.

This mathematical framework forms the foundation for the Extended Kalman
Filter used later in the project.