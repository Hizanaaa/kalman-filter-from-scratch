import numpy as np

from models import RobotState


class RobotSimulator:
    """
    Simulates the true motion of a robot in 2D.
    """

    def __init__(self, dt: float = 1.0):

        self.dt = dt

    def simulate(
        self,
        initial_state: RobotState,
        num_steps: int,
    ):

        states = []

        state = RobotState(
            initial_state.x,
            initial_state.y,
            initial_state.vx,
            initial_state.vy,
        )

        for _ in range(num_steps):

            states.append(
                RobotState(
                    state.x,
                    state.y,
                    state.vx,
                    state.vy,
                )
            )

            state.x += state.vx * self.dt
            state.y += state.vy * self.dt

        return states