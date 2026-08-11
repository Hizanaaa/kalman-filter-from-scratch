import numpy as np

from simulation.models import RobotState, SimulationResult
from simulation.noise import process_noise, gps_noise


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

        self.H = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
            ],
            dtype=float,
        )

    def simulate(
        self,
        initial_state: RobotState,
        num_steps: int,
    ):

        result = SimulationResult()

        state_vector = initial_state.as_vector()

        for _ in range(num_steps):

            result.ground_truth.append(
                RobotState.from_vector(state_vector)
            )

            gps = self.H @ state_vector + gps_noise()

            result.gps_measurements.append(gps)

            state_vector = self.F @ state_vector + process_noise()

        return result