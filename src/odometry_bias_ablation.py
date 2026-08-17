import numpy as np

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from simulation.sensors import OdometrySensor
from filters.odometry_ekf import OdometryEKF


def rmse(estimated, true):
    error = estimated - true
    return np.sqrt(np.mean(np.sum(error ** 2, axis=1)))


def run(seed, bias_drift, num_steps=100):
    simulator = NonlinearRobotSimulator(
        dt=1.0,
        gps_noise_std=0.5,
        random_seed=seed,
    )

    initial_state = np.array(
        [[0.0], [0.0], [0.0], [2.0], [0.1]]
    )

    truth, gps = simulator.simulate(
        initial_state=initial_state,
        num_steps=num_steps,
    )

    odometry = OdometrySensor(
        velocity_noise_std=0.05,
        angular_velocity_noise_std=0.01,
        velocity_bias_drift_std=bias_drift[0],
        angular_velocity_bias_drift_std=bias_drift[1],
        random_seed=seed,
    )

    ekf = OdometryEKF(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
        velocity_variance=0.05,
        angular_velocity_variance=0.01,
    )

    ekf.initialize(
        np.array([[0.0], [0.0], [0.0]])
    )

    estimates = []

    for i, (state, measurement) in enumerate(zip(truth, gps)):

        v, omega = odometry.measure(
            true_velocity=state[3, 0],
            true_angular_velocity=state[4, 0],
        )

        if i > 0:
            ekf.predict(
                velocity=v,
                angular_velocity=omega,
            )

        ekf.update(measurement)

        estimates.append(ekf.x.copy())

    truth_xy = np.array(
        [[s[0, 0], s[1, 0]] for s in truth]
    )

    gps_xy = np.array(
        [[z[0, 0], z[1, 0]] for z in gps]
    )

    estimate_xy = np.array(
        [[s[0, 0], s[1, 0]] for s in estimates]
    )

    gps_error = rmse(gps_xy, truth_xy)
    ekf_error = rmse(estimate_xy, truth_xy)

    improvement = (
        (gps_error - ekf_error)
        / gps_error
        * 100
    )

    return gps_error, ekf_error, improvement


def evaluate(name, bias_drift):
    gps = []
    ekf = []
    improvement = []

    for seed in range(50):
        g, e, i = run(seed, bias_drift)

        gps.append(g)
        ekf.append(e)
        improvement.append(i)

    gps = np.array(gps)
    ekf = np.array(ekf)
    improvement = np.array(improvement)

    print(f"\n{name}")
    print("-" * len(name))

    print(
        f"GPS RMSE: "
        f"{gps.mean():.4f} +/- {gps.std():.4f} m"
    )

    print(
        f"EKF RMSE: "
        f"{ekf.mean():.4f} +/- {ekf.std():.4f} m"
    )

    print(
        f"Improvement: "
        f"{improvement.mean():.2f}% "
        f"+/- {improvement.std():.2f}%"
    )


def main():
    print("\nOdometry Bias Ablation Study")
    print("============================")
    print("50 independent simulations")
    print("100 steps per simulation")

    evaluate(
        "White Noise Only",
        bias_drift=(0.0, 0.0),
    )

    evaluate(
        "Low Bias Drift",
        bias_drift=(0.0005, 0.0002),
    )

    evaluate(
        "Medium Bias Drift",
        bias_drift=(0.002, 0.001),
    )

    evaluate(
        "High Bias Drift",
        bias_drift=(0.01, 0.005),
    )


if __name__ == "__main__":
    main()
