import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path

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

def plot_odometry_dropout(
    ground_truth,
    gps_measurements,
    estimates,
    position_errors,
    covariance_trace,
    dropout_start,
    dropout_end,
):
    true_x = [state[0, 0] for state in ground_truth]
    true_y = [state[1, 0] for state in ground_truth]

    gps_x = [gps[0, 0] for gps in gps_measurements]
    gps_y = [gps[1, 0] for gps in gps_measurements]

    estimate_x = [state[0, 0] for state in estimates]
    estimate_y = [state[1, 0] for state in estimates]

    steps = np.arange(len(ground_truth))

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    # Trajectory plot
    plt.figure(figsize=(10, 6))

    plt.plot(
        true_x,
        true_y,
        linewidth=2.5,
        label="Ground Truth",
    )

    plt.scatter(
        gps_x,
        gps_y,
        s=25,
        alpha=0.5,
        label="GPS",
    )

    plt.plot(
        estimate_x,
        estimate_y,
        linewidth=2.5,
        label="Odometry EKF",
    )

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Odometry EKF with GPS Dropout")

    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    trajectory_file = (
        results_dir / "odometry_dropout_trajectory.png"
    )

    plt.savefig(
        trajectory_file,
        dpi=300,
        bbox_inches="tight",
    )

    plt.close()

    # Error plot
    plt.figure(figsize=(10, 6))

    plt.plot(
        steps,
        position_errors,
        linewidth=2,
        label="Position Error",
    )

    plt.axvspan(
        dropout_start,
        dropout_end - 1,
        alpha=0.2,
        label="GPS Dropout",
    )

    plt.xlabel("Time Step")
    plt.ylabel("Position Error (m)")
    plt.title("Position Error During GPS Dropout")

    plt.legend()
    plt.grid(True)

    error_file = (
        results_dir / "odometry_dropout_error.png"
    )

    plt.savefig(
        error_file,
        dpi=300,
        bbox_inches="tight",
    )

    plt.close()

    # Covariance plot
    plt.figure(figsize=(10, 6))

    plt.plot(
        steps,
        covariance_trace,
        linewidth=2,
        label="Covariance Trace",
    )

    plt.axvspan(
        dropout_start,
        dropout_end - 1,
        alpha=0.2,
        label="GPS Dropout",
    )

    plt.xlabel("Time Step")
    plt.ylabel("Covariance Trace")
    plt.title("EKF Uncertainty During GPS Dropout")

    plt.legend()
    plt.grid(True)

    covariance_file = (
        results_dir / "odometry_dropout_covariance.png"
    )

    plt.savefig(
        covariance_file,
        dpi=300,
        bbox_inches="tight",
    )

    plt.close()

    print(
        f"\nTrajectory plot saved to: "
        f"{trajectory_file}"
    )

    print(
        f"Error plot saved to: "
        f"{error_file}"
    )

    print(
        f"Covariance plot saved to: "
        f"{covariance_file}"
    )

def main():

    num_steps = 100

    dropout_start = 40
    dropout_end = 70

    # True nonlinear robot
    simulator = NonlinearRobotSimulator(
        dt=1.0,
        gps_noise_std=0.5,
        random_seed=42,
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

    # Drifting odometry sensor
    odometry = OdometrySensor(
        velocity_noise_std=0.05,
        angular_velocity_noise_std=0.01,
        velocity_bias_drift_std=0.002,
        angular_velocity_bias_drift_std=0.001,
        random_seed=42,
    )

    odometry_measurements = []

    for state in ground_truth:

        measured_velocity, measured_omega = (
            odometry.measure(
                true_velocity=state[3, 0],
                true_angular_velocity=state[4, 0],
            )
        )

        odometry_measurements.append(
            (
                measured_velocity,
                measured_omega,
            )
        )

    # EKF
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
    covariance_trace = []

    for i, (
        gps,
        odometry_measurement,
    ) in enumerate(
        zip(
            gps_measurements,
            odometry_measurements,
        )
    ):

        measured_velocity, measured_omega = (
            odometry_measurement
        )

        if i > 0:
            ekf.predict(
                velocity=measured_velocity,
                angular_velocity=measured_omega,
            )

        # GPS dropout from step 40 through 69
        if not (
            dropout_start
            <= i
            < dropout_end
        ):
            ekf.update(gps)

        estimates.append(
            ekf.x.copy()
        )

        covariance_trace.append(
            np.trace(ekf.P)
        )

    # Evaluation
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

    estimated_positions = np.array(
        [
            [state[0, 0], state[1, 0]]
            for state in estimates
        ]
    )

    position_errors = np.linalg.norm(
        estimated_positions - true_positions,
        axis=1,
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

    print("\nGPS + Odometry EKF")
    print("------------------")

    print(
        f"GPS RMSE: "
        f"{gps_rmse:.4f} m"
    )

    print(
        f"EKF RMSE: "
        f"{ekf_rmse:.4f} m"
    )

    print(
        f"Improvement: "
        f"{improvement:.2f}%"
    )

    print("\nGPS Dropout Analysis")
    print("--------------------")

    print(
        f"Position error before dropout: "
        f"{position_errors[dropout_start - 1]:.4f} m"
    )

    print(
        f"Position error during dropout: "
        f"{position_errors[dropout_end - 1]:.4f} m"
    )

    print(
        f"Position error after recovery: "
        f"{position_errors[dropout_end]:.4f} m"
    )

    print(
        f"Covariance trace before dropout: "
        f"{covariance_trace[dropout_start - 1]:.4f}"
    )

    print(
        f"Covariance trace during dropout: "
        f"{covariance_trace[dropout_end - 1]:.4f}"
    )

    print(
        f"Covariance trace after recovery: "
        f"{covariance_trace[dropout_end]:.4f}"
    )

    plot_odometry_dropout(
        ground_truth,
        gps_measurements,
        estimates,
        position_errors,
        covariance_trace,
        dropout_start,
        dropout_end,
    )
if __name__ == "__main__":
    main()
