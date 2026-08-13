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
        dt2 = dt ** 2
        dt3 = dt ** 3
        dt4 = dt ** 4

        self.Q = process_variance * np.array(
            [
                [dt4 / 4, 0, dt3 / 2, 0],
                [0, dt4 / 4, 0, dt3 / 2],
                [dt3 / 2, 0, dt2, 0],
                [0, dt3 / 2, 0, dt2],
            ],
            dtype=float,
        )

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

    def predict(self):
        """
        Predict the next state and covariance.
        """

        if self.x is None or self.P is None:
            raise RuntimeError(
                "Kalman Filter must be initialized before prediction."
            )

        # Predict the next state
        self.x = self.F @ self.x

        # Predict the uncertainty
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement: np.ndarray):
        """
        Update the state estimate using a measurement.
        """

        if self.x is None or self.P is None:
            raise RuntimeError(
                "Kalman Filter must be initialized before update."
            )

        # Innovation / residual
        innovation = measurement - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman Gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state estimate
        self.x = self.x + K @ innovation

        # Update covariance
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P