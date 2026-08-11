import numpy as np

from models import RobotState
from noise import process_noise


class RobotSimulator:
    """
    Simulates the true motion of a robot using a constant velocity model.
    """

    def __init__(
        self,
        dt: float = 1.0,
        random_seed: int = 42,
    ):

        self.dt = dt

        np.random.seed(random_seed)

        self.F = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=float,
        )

    def simulate(
        self,
        initial_state: RobotState,
        num_steps: int,
    ):

        trajectory = []

        state_vector = initial_state.as_vector()

        for _ in range(num_steps):

            trajectory.append(
                RobotState.from_vector(state_vector)
            )

            state_vector = self.F @ state_vector + process_noise()

        return trajectory