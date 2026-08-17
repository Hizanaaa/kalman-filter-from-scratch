import numpy as np

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from simulation.sensors import OdometrySensor
from filters.ekf import ExtendedKalmanFilter
from filters.odometry_ekf import OdometryEKF


def calculate_rmse(estimated, true):
    errors = estimated - true

    return np.sqrt(
        np.mean(
            np.sum(errors ** 2, axis=1)
        )
    )


def run_experiment(seed: int, num_steps: int = 100):
    """
    Compare GPS, GPS-only EKF, and GPS + Odometry EKF
    on the same nonlinear simulation.
    """

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

    # ---------------------------------------------------------
    # Generate odometry measurements from the SAME ground truth
    # ---------------------------------------------------------

    odometry = OdometrySensor(
        velocity_noise_std=0.05,
        angular_velocity_noise_std=0.01,
        velocity_bias_drift_std=0.002,
        angular_velocity_bias_drift_std=0.001,
        random_seed=seed,
    )

    odometry_measurements = []

    for state in ground_truth:
        measured_velocity, measured_omega = odometry.measure(
            true_velocity=state[3, 0],
            true_angular_velocity=state[4, 0],
        )

        odometry_measurements.append(
            (measured_velocity, measured_omega)
        )

    # ---------------------------------------------------------
    # 5-state GPS-only EKF
    # ---------------------------------------------------------

    ekf = ExtendedKalmanFilter(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    ekf.initialize(initial_state)

    ekf_estimates = []

    for i, gps in enumerate(gps_measurements):

        if i == 0:
            ekf.update(gps)
        else:
            ekf.predict()
            ekf.update(gps)

        ekf_estimates.append(ekf.x.copy())

    # ---------------------------------------------------------
    # 3-state GPS + Odometry EKF
    # ---------------------------------------------------------

    odometry_ekf = OdometryEKF(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    odometry_ekf.initialize(
        np.array(
            [
                [0.0],
                [0.0],
                [0.0],
            ]
        )
    )

    odometry_ekf_estimates = []

    for i, (gps, odometry_measurement) in enumerate(
        zip(gps_measurements, odometry_measurements)
    ):

        measured_velocity, measured_omega = odometry_measurement

        if i > 0:
            odometry_ekf.predict(
                velocity=measured_velocity,
                angular_velocity=measured_omega,
            )

        odometry_ekf.update(gps)

        odometry_ekf_estimates.append(
            odometry_ekf.x.copy()
        )

    # ---------------------------------------------------------
    # Extract positions
    # ---------------------------------------------------------

    true_positions = np.array(
        [
            [state[0, 0], state[1, 0]]
            for state in ground_truth
        ]
    )

    gps_positions = np.array(
        [
            [gps[0, 0], gps[1, 0]]
            for gps in gps_measurements
        ]
    )

    ekf_positions = np.array(
        [
            [state[0, 0], state[1, 0]]
            for state in ekf_estimates
        ]
    )

    odometry_ekf_positions = np.array(
        [
            [state[0, 0], state[1, 0]]
            for state in odometry_ekf_estimates
        ]
    )

    # ---------------------------------------------------------
    # Calculate RMSE
    # ---------------------------------------------------------

    gps_rmse = calculate_rmse(
        gps_positions,
        true_positions,
    )

    ekf_rmse = calculate_rmse(
        ekf_positions,
        true_positions,
    )

    odometry_ekf_rmse = calculate_rmse(
        odometry_ekf_positions,
        true_positions,
    )

    return (
        gps_rmse,
        ekf_rmse,
        odometry_ekf_rmse,
    )


def main():

    seeds = range(50)

    gps_errors = []
    ekf_errors = []
    odometry_ekf_errors = []

    for seed in seeds:

        (
            gps_rmse,
            ekf_rmse,
            odometry_ekf_rmse,
        ) = run_experiment(seed)

        gps_errors.append(gps_rmse)
        ekf_errors.append(ekf_rmse)
        odometry_ekf_errors.append(
            odometry_ekf_rmse
        )

    gps_errors = np.array(gps_errors)
    ekf_errors = np.array(ekf_errors)
    odometry_ekf_errors = np.array(
        odometry_ekf_errors
    )

    ekf_improvement = (
        (gps_errors - ekf_errors)
        / gps_errors
        * 100
    )

    odometry_ekf_improvement = (
        (gps_errors - odometry_ekf_errors)
        / gps_errors
        * 100
    )

    print("\nControlled Estimator Comparison")
    print("--------------------------------")

    print(
        f"GPS RMSE: "
        f"{gps_errors.mean():.4f} "
        f"+/- {gps_errors.std():.4f} m"
    )

    print(
        f"GPS-only EKF RMSE: "
        f"{ekf_errors.mean():.4f} "
        f"+/- {ekf_errors.std():.4f} m"
    )

    print(
        f"GPS-only EKF Improvement: "
        f"{ekf_improvement.mean():.2f}% "
        f"+/- {ekf_improvement.std():.2f}%"
    )

    print(
        f"GPS + Odometry EKF RMSE: "
        f"{odometry_ekf_errors.mean():.4f} "
        f"+/- {odometry_ekf_errors.std():.4f} m"
    )

    print(
        f"GPS + Odometry EKF Improvement: "
        f"{odometry_ekf_improvement.mean():.2f}% "
        f"+/- {odometry_ekf_improvement.std():.2f}%"
    )


if __name__ == "__main__":
    main()
