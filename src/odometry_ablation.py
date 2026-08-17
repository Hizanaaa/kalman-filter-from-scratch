import numpy as np

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from simulation.sensors import OdometrySensor
from filters.odometry_ekf import OdometryEKF


def calculate_rmse(estimated, true):
    errors = estimated - true

    return np.sqrt(
        np.mean(
            np.sum(errors ** 2, axis=1)
        )
    )


def run_experiment(
    seed,
    velocity_noise_std,
    angular_velocity_noise_std,
    velocity_bias_drift_std,
    angular_velocity_bias_drift_std,
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
        velocity_noise_std=velocity_noise_std,
        angular_velocity_noise_std=angular_velocity_noise_std,
        velocity_bias_drift_std=velocity_bias_drift_std,
        angular_velocity_bias_drift_std=angular_velocity_bias_drift_std,
        random_seed=seed,
    )

    ekf = OdometryEKF(
        dt=1.0,
        process_variance=0.05,
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
        [
            [state[0, 0], state[1, 0]]
            for state in ground_truth
        ]
    )

    estimated_positions = np.array(
        [
            [state[0, 0], state[1, 0]]
            for state in estimates
        ]
    )

    gps_positions = np.array(
        [
            [gps[0, 0], gps[1, 0]]
            for gps in gps_measurements
        ]
    )

    ekf_rmse = calculate_rmse(
        estimated_positions,
        true_positions,
    )

    gps_rmse = calculate_rmse(
        gps_positions,
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
    velocity_noise_std,
    angular_velocity_noise_std,
    velocity_bias_drift_std,
    angular_velocity_bias_drift_std,
):
    gps_results = []
    ekf_results = []
    improvement_results = []

    for seed in range(50):
        gps_rmse, ekf_rmse, improvement = run_experiment(
            seed=seed,
            velocity_noise_std=velocity_noise_std,
            angular_velocity_noise_std=angular_velocity_noise_std,
            velocity_bias_drift_std=velocity_bias_drift_std,
            angular_velocity_bias_drift_std=angular_velocity_bias_drift_std,
        )

        gps_results.append(gps_rmse)
        ekf_results.append(ekf_rmse)
        improvement_results.append(improvement)

    gps_results = np.array(gps_results)
    ekf_results = np.array(ekf_results)
    improvement_results = np.array(improvement_results)

    print(f"\n{name}")
    print("-" * len(name))

    print(
        f"GPS RMSE: "
        f"{gps_results.mean():.4f} "
        f"+/- {gps_results.std():.4f} m"
    )

    print(
        f"GPS + Odometry EKF RMSE: "
        f"{ekf_results.mean():.4f} "
        f"+/- {ekf_results.std():.4f} m"
    )

    print(
        f"RMSE Improvement: "
        f"{improvement_results.mean():.2f}% "
        f"+/- {improvement_results.std():.2f}%"
    )


def main():
    print("\nOdometry Noise Ablation Study")
    print("=============================")
    print("50 independent simulations")
    print("100 steps per simulation")

    evaluate_condition(
        name="Low Odometry Noise",
        velocity_noise_std=0.02,
        angular_velocity_noise_std=0.005,
        velocity_bias_drift_std=0.0005,
        angular_velocity_bias_drift_std=0.0002,
    )

    evaluate_condition(
        name="Medium Odometry Noise",
        velocity_noise_std=0.05,
        angular_velocity_noise_std=0.01,
        velocity_bias_drift_std=0.002,
        angular_velocity_bias_drift_std=0.001,
    )

    evaluate_condition(
        name="High Odometry Noise",
        velocity_noise_std=0.15,
        angular_velocity_noise_std=0.03,
        velocity_bias_drift_std=0.01,
        angular_velocity_bias_drift_std=0.005,
    )


if __name__ == "__main__":
    main()
