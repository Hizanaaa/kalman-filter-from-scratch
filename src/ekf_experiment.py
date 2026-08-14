import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path 

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from filters.ekf import ExtendedKalmanFilter

def plot_ekf_trajectory(
    ground_truth,
    gps_measurements,
    estimates,
):
    """
    Plot nonlinear ground truth, noisy GPS,
    and EKF trajectory.
    """

    true_x = [state[0, 0] for state in ground_truth]
    true_y = [state[1, 0] for state in ground_truth]

    gps_x = [gps[0, 0] for gps in gps_measurements]
    gps_y = [gps[1, 0] for gps in gps_measurements]

    estimate_x = [state[0, 0] for state in estimates]
    estimate_y = [state[1, 0] for state in estimates]

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
        s=35,
        alpha=0.7,
        label="GPS Measurements",
    )

    plt.plot(
        estimate_x,
        estimate_y,
        linewidth=2.5,
        label="EKF Estimate",
    )

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Nonlinear Robot Localization: EKF")

    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    output_file = results_dir / "ekf_trajectory.png"

    plt.savefig(
        output_file,
        dpi=300,
        bbox_inches="tight",
    )

    plt.close()

    print(
        f"\nEKF trajectory plot saved to: "
        f"{output_file}"
    )


def main():

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
        num_steps=100,
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

    print("\nEKF Estimates")
    print("-------------")

    for i, estimate in enumerate(estimates[:10]):

        print(
            f"Step {i:02d} | "
            f"Estimate: "
            f"x={estimate[0, 0]:.2f}, "
            f"y={estimate[1, 0]:.2f}, "
            f"theta={estimate[2, 0]:.2f}, "
            f"v={estimate[3, 0]:.2f}, "
            f"omega={estimate[4, 0]:.2f}"
        )

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

    gps_rmse = np.sqrt(
        np.mean(
            np.sum(
                (gps_positions - true_positions) ** 2,
                axis=1,
            )
        )
    )

    ekf_rmse = np.sqrt(
        np.mean(
            np.sum(
                (estimated_positions - true_positions) ** 2,
                axis=1,
            )
        )
    )

    improvement = (
        (gps_rmse - ekf_rmse)
        / gps_rmse
        * 100
    )

    print("\nEKF Evaluation")
    print("--------------")
    print(f"GPS RMSE:  {gps_rmse:.4f} m")
    print(f"EKF RMSE:  {ekf_rmse:.4f} m")
    print(f"Improvement: {improvement:.2f}%")

    plot_ekf_trajectory(
        ground_truth,
        gps_measurements,
        estimates,
    )

if __name__ == "__main__":
    main()
