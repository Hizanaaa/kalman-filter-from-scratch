# Failure Modes

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
