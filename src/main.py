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

    states = simulator.simulate(
        initial_state=initial_state,
        num_steps=10,
    )

    for i, state in enumerate(states):

        print(
            f"Step {i}: "
            f"x={state.x:.2f}, "
            f"y={state.y:.2f}, "
            f"vx={state.vx:.2f}, "
            f"vy={state.vy:.2f}"
        )


if __name__ == "__main__":
    main()