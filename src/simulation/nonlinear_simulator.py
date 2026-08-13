import numpy as np


class NonlinearRobotSimulator:
    """
    Simulates a 2D robot using a nonlinear unicycle motion model.
    """

    def __init__(
        self,
        dt: float,
        gps_noise_std: float,
        random_seed: int | None = None,
    ):
        self.dt = dt
        self.gps_noise_std = gps_noise_std

        self.rng = np.random.default_rng(random_seed)

    def step(self, state: np.ndarray) -> np.ndarray:
        """
        Propagate the robot state by one timestep.

        State:
        [x, y, theta, v, omega]
        """

        x, y, theta, v, omega = state.flatten()

        dt = self.dt

        next_state = np.array(
            [
                [
                    x + v * np.cos(theta) * dt
                ],
                [
                    y + v * np.sin(theta) * dt
                ],
                [
                    theta + omega * dt
                ],
                [
                    v
                ],
                [
                    omega
                ],
            ],
            dtype=float,
        )

        # Keep heading in [-pi, pi]
        next_state[2, 0] = (
            next_state[2, 0] + np.pi
        ) % (2 * np.pi) - np.pi

        return next_state

    def gps_measurement(
        self,
        state: np.ndarray,
    ) -> np.ndarray:
        """
        Generate a noisy GPS measurement of x and y.
        """

        x, y = state[0, 0], state[1, 0]

        noise = self.rng.normal(
            0.0,
            self.gps_noise_std,
            size=(2, 1),
        )

        return np.array(
            [
                [x],
                [y],
            ]
        ) + noise

    def simulate(
        self,
        initial_state: np.ndarray,
        num_steps: int,
    ):
        """
        Generate nonlinear ground truth and GPS measurements.
        """

        state = initial_state.copy()

        ground_truth = []
        gps_measurements = []

        for _ in range(num_steps):

            ground_truth.append(state.copy())

            gps_measurements.append(
                self.gps_measurement(state)
            )

            state = self.step(state)

        return ground_truth, gps_measurements
