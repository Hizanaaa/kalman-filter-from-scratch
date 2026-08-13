from simulation.simulator import RobotSimulator
from simulation.models import RobotState
from filters.kalman_filter import KalmanFilter
from visualization import plot_trajectory

import numpy as np


def main():

    # Create simulator

    simulator = RobotSimulator(dt=1.0)

    initial_state = RobotState(
        x=0.0,
        y=0.0,
        vx=2.0,
        vy=1.0,
    )

    # Generate simulated data FIRST
    simulation = simulator.simulate(
        initial_state=initial_state,
        num_steps=10,
    )

    # 2. Create and initialize Kalman Filter

    kf = KalmanFilter(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    kf.initialize(initial_state)

    # Prediction

    # Store Kalman Filter estimates
    estimates = []

    for i, gps in enumerate(simulation.gps_measurements):

        # At t=0, use the initial GPS measurement directly.
        if i == 0:
            kf.update(gps)

        else:
            # Predict from the previous time step
            kf.predict()

            # Correct the prediction using the current GPS
            kf.update(gps)

        # Store the estimate
        estimates.append(
            RobotState.from_vector(kf.x)
        )


    #Postion Error Calculation
    for i, estimate in enumerate(estimates):
        print(
            f"Estimate {i:02d} | "
            f"x={estimate.x:.2f}, "
            f"y={estimate.y:.2f}, "
            f"vx={estimate.vx:.2f}, "
            f"vy={estimate.vy:.2f}"
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

    gps_rmse = np.sqrt(
        np.mean(
            np.sum(
                (gps_positions - true_positions) ** 2,
                axis=1,
            )
        )
    )

    kalman_rmse = np.sqrt(
        np.mean(
            np.sum(
                (estimated_positions - true_positions) ** 2,
                axis=1,
            )
        )
    )

    print(f"\nGPS RMSE:     {gps_rmse:.4f} m")
    print(f"Kalman RMSE:  {kalman_rmse:.4f} m")

    # 5. Visualize simulation

    for i, (state, gps) in enumerate(
        zip(
            simulation.ground_truth,
            simulation.gps_measurements,
        )
    ):
        print(
            f"Step {i:02d} | "
            f"True: ({state.x:.2f}, {state.y:.2f}) | "
            f"GPS: ({gps[0, 0]:.2f}, {gps[1, 0]:.2f})"
        )

    plot_trajectory(
        simulation.ground_truth,
        simulation.gps_measurements,
        estimates
    )


if __name__ == "__main__":
    main()