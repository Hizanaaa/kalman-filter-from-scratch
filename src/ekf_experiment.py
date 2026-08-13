import numpy as np

from simulation.nonlinear_simulator import NonlinearRobotSimulator
from filters.ekf import ExtendedKalmanFilter


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

if __name__ == "__main__":
    main()
