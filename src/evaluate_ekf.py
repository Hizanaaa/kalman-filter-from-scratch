import numpy as np

from pathlib import Path

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from filters.ekf import ExtendedKalmanFilter


def calculate_rmse(estimated, true):
    errors = estimated - true

    return np.sqrt(
        np.mean(
            np.sum(errors ** 2, axis=1)
        )
    )


def run_experiment(seed: int, num_steps: int = 100):

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

    ekf = ExtendedKalmanFilter(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    ekf.initialize(initial_state)

    estimates = []

    for i, gps in enumerate(gps_measurements):

        if i == 0:
            ekf.update(gps)
        else:
            ekf.predict()
            ekf.update(gps)

        estimates.append(ekf.x.copy())

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

    gps_rmse = calculate_rmse(
        gps_positions,
        true_positions,
    )

    ekf_rmse = calculate_rmse(
        estimated_positions,
        true_positions,
    )

    return gps_rmse, ekf_rmse


def main():

    seeds = range(50)

    gps_errors = []
    ekf_errors = []

    for seed in seeds:

        gps_rmse, ekf_rmse = run_experiment(seed)

        gps_errors.append(gps_rmse)
        ekf_errors.append(ekf_rmse)

    gps_errors = np.array(gps_errors)
    ekf_errors = np.array(ekf_errors)

    improvement = (
        (gps_errors - ekf_errors)
        / gps_errors
        * 100
    )
    
    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    output_file = results_dir / "ekf_evaluation_results.txt"

    with open(output_file, "w") as file:
        file.write("Extended Kalman Filter Evaluation\n")
        file.write("=================================\n\n")

        file.write("Number of runs: 50\n")
        file.write("Steps per run: 100\n\n")

        file.write(
            f"GPS RMSE: "
            f"{gps_errors.mean():.4f} "
            f"+/- {gps_errors.std():.4f} m\n"
        )

        file.write(
            f"EKF RMSE: "
            f"{ekf_errors.mean():.4f} "
            f"+/- {ekf_errors.std():.4f} m\n"
        )

        file.write(
            f"RMSE Improvement: "
            f"{improvement.mean():.2f}% "
            f"+/- {improvement.std():.2f}%\n"
        )

    print(
        f"\nResults saved to: {output_file}"
    )

    print("\nEKF Multi-Run Evaluation")
    print("------------------------")

    print(
        f"GPS RMSE: "
        f"{gps_errors.mean():.4f} "
        f"+/- {gps_errors.std():.4f} m"
    )

    print(
        f"EKF RMSE: "
        f"{ekf_errors.mean():.4f} "
        f"+/- {ekf_errors.std():.4f} m"
    )

    print(
        f"RMSE Improvement: "
        f"{improvement.mean():.2f}% "
        f"+/- {improvement.std():.2f}%"
    )


if __name__ == "__main__":
    main()
