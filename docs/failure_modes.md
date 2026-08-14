# Failure Mode Analysis

## GPS Dropout

A GPS dropout was simulated to evaluate how the Kalman Filter behaves when
position measurements become temporarily unavailable.

### Experiment Setup

- Simulation length: 100 steps
- GPS dropout: steps 40–69
- Dropout duration: 30 steps
- Process model: constant velocity
- State: `[x, y, vx, vy]`
- GPS measurement: `[x, y]`

During the dropout, the filter performs prediction steps but does not receive
GPS measurements.

Therefore, the covariance evolves according to:

\[
P_k = F P_{k-1} F^T + Q
\]

without the corrective measurement update.

## Results

| Metric | Before Dropout | During Dropout | After Recovery |
|---|---:|---:|---:|
| Position Error (m) | 0.4923 | 13.4045 | 1.3628 |
| Covariance Trace | 0.7526 | 1102.0521 | 1.8278 |

### Interpretation

Before the GPS outage, the filter maintains a relatively small position
error and covariance.

During the 30-step outage, uncertainty increases dramatically because the
filter is relying entirely on its motion model. Small model errors accumulate
over time, causing the position estimate to diverge from the ground truth.

When GPS measurements become available again, the measurement update provides
new information about the robot's position. The covariance therefore drops
rapidly and the position estimate begins recovering.

### Key Observation

The Kalman Filter does not remain accurate indefinitely without measurements.

Instead, it explicitly represents increasing uncertainty:

\[
\text{No measurements}
\Rightarrow
\text{Increasing uncertainty}
\Rightarrow
\text{Increasing estimation error}
\]

This is an important property of probabilistic state estimation.

# GPS and Sensor Fail Analysis

This document evaluates how the localization system behaves when GPS
measurements become unavailable and the robot must rely on odometry.

The experiments focus on two important properties of a Kalman-filter-based
localization system:

1. Uncertainty growth when absolute position measurements are unavailable.
2. Recovery when GPS measurements become available again.

---

## 1. GPS Dropout with Linear Kalman Filter

The first failure-mode experiment evaluates the linear Kalman Filter under a
GPS outage.

### Configuration

- GPS dropout: steps 40–69
- Total simulation steps: 100
- GPS measurements: position `(x, y)`
- Motion model: constant velocity

### Results

| Metric | Result |
|---|---:|
| Position error before dropout | 0.4923 m |
| Position error during dropout | 13.4045 m |
| Position error after GPS recovery | 1.3628 m |
| Covariance trace before dropout | 0.7526 |
| Covariance trace during dropout | 1102.0521 |
| Covariance trace after recovery | 1.8278 |

### Interpretation

When GPS measurements are removed, the filter must rely entirely on its
motion model. Prediction errors accumulate over time, causing the position
error to increase substantially.

At the same time, the covariance grows from:

`0.7526 → 1102.0521`

This represents increasing uncertainty in the state estimate.

When GPS becomes available again, the measurement update reduces the
uncertainty substantially and pulls the state estimate back toward the true
trajectory.

The remaining post-recovery error demonstrates that a measurement update
does not necessarily produce an instantaneous perfect estimate.

---

## 2. Odometry Drift with GPS Dropout

The second experiment uses a nonlinear robot model with a drifting odometry
sensor.

The odometry sensor measures:

- Linear velocity `v`
- Angular velocity `ω`

The measurements contain both random noise and slowly changing bias.

The EKF state is:

```text
[x, y, θ]
