# Kalman Filter Equations

## 1. State Prediction

$$ \hat{x}_k^{-} = F\hat{x}_{k-1} $$

$$ P_k^{-} = FP_{k-1}F^T + Q $$

## 2. Measurement Prediction

$$ \hat{z}_k = H\hat{x}_k^{-} $$

## 3. Innovation

$$ y_k = z_k - H\hat{x}_k^{-} $$

## 4. Innovation Covariance

$$ S_k = HP_k^{-}H^T + R $$

## 5. Kalman Gain

$$ K_k = P_k^{-}H^TS_k^{-1} $$

## 6. State Update

$$ \hat{x}_k = \hat{x}_k^{-} + K_ky_k $$

## 7. Covariance Update

$$ P_k = (I-K_kH)P_k^{-} $$

---

# Matrices Used in This Project

## State

$$ x = \begin{bmatrix} x \\ y \\ v_x \\ v_y \end{bmatrix} $$

## State Transition Matrix

$$ F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} $$

## GPS Measurement Matrix

$$ H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix} $$

## Process Noise

$$ Q $$

## Measurement Noise

$$ R $$

---

# Complete Filter Cycle

$$ \boxed{ \text{Predict} \rightarrow \text{Measure} \rightarrow \text{Innovation} \rightarrow \text{Kalman Gain} \rightarrow \text{Update} } $$
