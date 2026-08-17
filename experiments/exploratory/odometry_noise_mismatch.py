import numpy as np

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from simulation.sensors import OdometrySensor
from filters.odometry_ekf import OdometryEKF


def calculate_rmse(estimated, true):
    errors = estimated - true
    return np.sqrt(np.mean(np.sum(errors ** 2, axis=1)))


def run_experiment(
    seed,
    actual_velocity_noise,
    actual_angular_noise,
    actual_velocity_bias_drift,
    actual_angular_bias_drift,
    filter_process_variance,
    num_steps=100,
):
    simulator = NonlinearRobotSimulator(
        dt=1.0,
        gps_noise_std=0.5,
        random_seed=seed,
    )

    initial_state = np.array(
        [
            [0.0],
            [0.0],
            [0.0],
            [2.0],
            [0.1],
        ]
    )

    ground_truth, gps_measurements = simulator.simulate(
        initial_state=initial_state,
        num_steps=num_steps,
    )

    odometry = OdometrySensor(
        velocity_noise_std=actual_velocity_noise,
        angular_velocity_noise_std=actual_angular_noise,
        velocity_bias_drift_std=actual_velocity_bias_drift,
        angular_velocity_bias_drift_std=actual_angular_bias_drift,
        random_seed=seed,
    )

    ekf = OdometryEKF(
        dt=1.0,
        process_variance=filter_process_variance,
        measurement_variance=0.5,
    )

    ekf.initialize(
        np.array(
            [
                [0.0],
                [0.0],
                [0.0],
            ]
        )
    )

    estimates = []

    for i, (true_state, gps) in enumerate(
        zip(ground_truth, gps_measurements)
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

        ekf.update(gps)

        estimates.append(ekf.x.copy())

    true_positions = np.array(
        [[s[0, 0], s[1, 0]] for s in ground_truth]
    )

    estimated_positions = np.array(
        [[s[0, 0], s[1, 0]] for s in estimates]
    )

    gps_positions = np.array(
        [[z[0, 0], z[1, 0]] for z in gps_measurements]
    )

    gps_rmse = calculate_rmse(
        gps_positions,
        true_positions,
    )

    ekf_rmse = calculate_rmse(
        estimated_positions,
        true_positions,
    )

    improvement = (
        (gps_rmse - ekf_rmse)
        / gps_rmse
        * 100
    )

    return gps_rmse, ekf_rmse, improvement


def evaluate_condition(
    name,
    actual_noise,
    filter_process_variance,
):
    gps_results = []
    ekf_results = []
    improvements = []

    for seed in range(50):
        gps_rmse, ekf_rmse, improvement = run_experiment(
            seed=seed,
            actual_velocity_noise=actual_noise[0],
            actual_angular_noise=actual_noise[1],
            actual_velocity_bias_drift=actual_noise[2],
            actual_angular_bias_drift=actual_noise[3],
            filter_process_variance=filter_process_variance,
        )

        gps_results.append(gps_rmse)
        ekf_results.append(ekf_rmse)
        improvements.append(improvement)

    gps_results = np.array(gps_results)
    ekf_results = np.array(ekf_results)
    improvements = np.array(improvements)

    print(f"\n{name}")
    print("-" * len(name))

    print(
        f"GPS RMSE: "
        f"{gps_results.mean():.4f} "
        f"+/- {gps_results.std():.4f} m"
    )

    print(
        f"EKF RMSE: "
        f"{ekf_results.mean():.4f} "
        f"+/- {ekf_results.std():.4f} m"
    )

    print(
        f"Improvement: "
        f"{improvements.mean():.2f}% "
        f"+/- {improvements.std():.2f}%"
    )


def main():
    print("\nOdometry Noise-Model Mismatch Study")
    print("===================================")
    print("50 independent simulations")
    print("100 steps per simulation")
    print()
    print("Filter assumes medium process variance:")
    print("process_variance = 0.05")

    medium_noise = (
        0.05,
        0.01,
        0.002,
        0.001,
    )

    high_noise = (
        0.15,
        0.03,
        0.01,
        0.005,
    )

    evaluate_condition(
        name="Matched Medium Noise",
        actual_noise=medium_noise,
        filter_process_variance=0.05,
    )

    evaluate_condition(
        name="High Actual Noise - Filter Unaware",
        actual_noise=high_noise,
        filter_process_variance=0.05,
    )

    evaluate_condition(
        name="High Actual Noise - Increased Process Variance",
        actual_noise=high_noise,
        filter_process_variance=0.20,
    )


if __name__ == "__main__":
    main()
