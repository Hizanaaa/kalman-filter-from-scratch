from simulator import RobotSimulator
from models import RobotState


def main():

    simulator = RobotSimulator(dt=1.0)

    initial_state = RobotState(
        x=0.0,
        y=0.0,
        vx=2.0,
        vy=1.0,
    )

    ground_truth, gps_measurements = simulator.simulate(
        initial_state=initial_state,
        num_steps=10,
    )

    for i, (state, gps) in enumerate(zip(ground_truth, gps_measurements)):
        print(
            f"Step {i:02d} | "
            f"True: ({state.x:.2f}, {state.y:.2f}) | "
            f"GPS: ({gps[0,0]:.2f}, {gps[1,0]:.2f})"
        )


if __name__ == "__main__":
    main()