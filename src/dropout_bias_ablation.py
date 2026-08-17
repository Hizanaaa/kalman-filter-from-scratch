import numpy as np

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from simulation.sensors import OdometrySensor
from filters.odometry_ekf import OdometryEKF


DROPOUT_START = 40
DROPOUT_END = 70


def position_error(estimate, truth):
    return np.linalg.norm(
        estimate[:2, 0] - truth[:2, 0]
    )


def run_experiment(seed, bias_drift, num_steps=100):
    simulator = NonlinearRobotSimulator(
        dt=1.0,
        gps_noise_std=0.5,
        random_seed=seed,
    )

    initial_state = np.array(
        [[0.0], [0.0], [0.0], [2.0], [0.1]]
    )

    truth, gps_measurements = simulator.simulate(
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

    errors_before = []
    errors_during = []
    errors_after = []

    for i, (true_state, gps) in enumerate(
        zip(truth, gps_measurements)
    ):
        velocity, angular_velocity = odometry.measure(
            true_velocity=true_state[3, 0],
            true_angular_velocity=true_state[4, 0],
        )

        if i > 0:
            ekf.predict(
                velocity=velocity,
                angular_velocity=angular_velocity,
            )

        # GPS available before and after dropout.
        # GPS unavailable during steps 40-69.
        if not (
            DROPOUT_START <= i < DROPOUT_END
        ):
            ekf.update(gps)

        error = position_error(
            ekf.x,
            true_state,
        )

        if i < DROPOUT_START:
            errors_before.append(error)

        elif i < DROPOUT_END:
            errors_during.append(error)

        else:
            errors_after.append(error)

    return (
        np.mean(errors_before),
        np.mean(errors_during),
        np.mean(errors_after),
    )


def evaluate(name, bias_drift):
    before = []
    during = []
    after = []

    for seed in range(50):
        b, d, a = run_experiment(
            seed,
            bias_drift,
        )

        before.append(b)
        during.append(d)
        after.append(a)

    before = np.array(before)
    during = np.array(during)
    after = np.array(after)

    print(f"\n{name}")
    print("-" * len(name))

    print(
        f"Before dropout: "
        f"{before.mean():.4f} "
        f"+/- {before.std():.4f} m"
    )

    print(
        f"During dropout: "
        f"{during.mean():.4f} "
        f"+/- {during.std():.4f} m"
    )

    print(
        f"After recovery: "
        f"{after.mean():.4f} "
        f"+/- {after.std():.4f} m"
    )


def main():
    print("\nGPS Dropout × Odometry Bias Ablation")
    print("=====================================")
    print("50 independent simulations")
    print("100 steps per simulation")
    print("GPS dropout: steps 40-69")

    evaluate(
        "White Noise Only",
        (0.0, 0.0),
    )

    evaluate(
        "Low Bias Drift",
        (0.0005, 0.0002),
    )

    evaluate(
        "Medium Bias Drift",
        (0.002, 0.001),
    )

    evaluate(
        "High Bias Drift",
        (0.01, 0.005),
    )


if __name__ == "__main__":
    main()
