from simulation.simulator import RobotSimulator
from simulation.models import RobotState
from visualization import plot_trajectory
from filters.kalman_filter import KalmanFilter


def main():

    simulator = RobotSimulator(dt=1.0)

    initial_state = RobotState(
        x=0.0,
        y=0.0,
        vx=2.0,
        vy=1.0,
    )

    kf = KalmanFilter(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    kf.initialize(initial_state)

    simulation = simulator.simulate(
        initial_state=initial_state,
        num_steps=10,
    )

    for i, (state, gps) in enumerate(
        zip(
            simulation.ground_truth,
            simulation.gps_measurements,
        )
    ):
        print(
            f"Step {i:02d} | "
            f"True: ({state.x:.2f}, {state.y:.2f}) | "
            f"GPS: ({gps[0,0]:.2f}, {gps[1,0]:.2f})"
        )

    plot_trajectory(
        simulation.ground_truth,
        simulation.gps_measurements,
    )


if __name__ == "__main__":
    main()