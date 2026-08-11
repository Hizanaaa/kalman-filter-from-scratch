import numpy as np

from simulation.models import RobotState


class KalmanFilter:
    """
    Linear Kalman Filter for 2D robot localization.
    """

    def __init__(
        self,
        dt: float,
        process_variance: float,
        measurement_variance: float,
    ):

        self.dt = dt

        # State estimate
        self.x = None

        # Estimate covariance
        self.P = None

        # State transition matrix
        self.F = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=float,
        )

        # Measurement matrix
        self.H = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
            ],
            dtype=float,
        )

        # Process covariance
        self.Q = np.eye(4) * process_variance

        # Measurement covariance
        self.R = np.eye(2) * measurement_variance

    def initialize(
        self,
        initial_state: RobotState,
        initial_uncertainty: float = 1.0,
    ):
        """
        Initialize the Kalman Filter.
        """

        self.x = initial_state.as_vector()

        self.P = np.eye(4) * initial_uncertainty
