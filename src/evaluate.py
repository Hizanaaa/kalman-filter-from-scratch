import numpy as np

from simulation.simulator import RobotSimulator
from simulation.models import RobotState
from filters.kalman_filter import KalmanFilter


def calculate_rmse(estimated, true):
    """
    Calculate position RMSE.
    """

    errors = estimated - true

    return np.sqrt(
        np.mean(
            np.sum(errors ** 2, axis=1)
        )
    )


def run_experiment(seed: int, num_steps: int = 100):
    """
    Run one simulation and compare GPS against
    the Kalman Filter.
    """

    simulator = RobotSimulator(
        dt=1.0,
        random_seed=seed,
    )

    initial_state = RobotState(
        x=0.0,
        y=0.0,
        vx=2.0,
        vy=1.0,
    )

    simulation = simulator.simulate(
        initial_state=initial_state,
        num_steps=num_steps,
    )

    kf = KalmanFilter(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    kf.initialize(initial_state)

    estimates = []

    for i, gps in enumerate(simulation.gps_measurements):

        if i == 0:
            kf.update(gps)
        else:
            kf.predict()
            kf.update(gps)

        estimates.append(
            RobotState.from_vector(kf.x)
        )

    true_positions = np.array(
        [
            [state.x, state.y]
            for state in simulation.ground_truth
        ]
    )

    gps_positions = np.array(
        [
            [gps[0, 0], gps[1, 0]]
            for gps in simulation.gps_measurements
        ]
    )

    estimated_positions = np.array(
        [
            [state.x, state.y]
            for state in estimates
        ]
    )

    gps_rmse = calculate_rmse(
        gps_positions,
        true_positions,
    )

    kalman_rmse = calculate_rmse(
        estimated_positions,
        true_positions,
    )

    return gps_rmse, kalman_rmse


def main():

    seeds = range(50)

    gps_errors = []
    kalman_errors = []

    for seed in seeds:

        gps_rmse, kalman_rmse = run_experiment(seed)

        gps_errors.append(gps_rmse)
        kalman_errors.append(kalman_rmse)

    gps_errors = np.array(gps_errors)
    kalman_errors = np.array(kalman_errors)

    improvement = (
        (gps_errors - kalman_errors)
        / gps_errors
        * 100
    )

    print("\nEvaluation Results")
    print("------------------")

    print(
        f"GPS RMSE: "
        f"{gps_errors.mean():.4f} "
        f"+/- {gps_errors.std():.4f} m"
    )

    print(
        f"Kalman RMSE: "
        f"{kalman_errors.mean():.4f} "
        f"+/- {kalman_errors.std():.4f} m"
    )

    print(
        f"RMSE Improvement: "
        f"{improvement.mean():.2f}% "
        f"+/- {improvement.std():.2f}%"
    )


if __name__ == "__main__":
    main()