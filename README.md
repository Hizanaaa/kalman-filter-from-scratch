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
