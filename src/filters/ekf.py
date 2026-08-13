import numpy as np


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for nonlinear 2D robot localization.
    """

    def __init__(
        self,
        dt: float,
        process_variance: float,
        measurement_variance: float,
    ):
        self.dt = dt

        # State estimate:
        # [x, y, theta, v, omega]
        self.x = None

        # State covariance
        self.P = None

        # Process noise covariance
        self.Q = process_variance * np.diag(
        [
            dt**4 / 4,
            dt**4 / 4,
            dt**2,
            dt**2,
            dt**2,
        ]
    )

        # GPS measurement noise covariance
        self.R = np.eye(2) * measurement_variance

    def motion_model(self, state: np.ndarray) -> np.ndarray:
        """
        Nonlinear unicycle motion model.

        State:
        [x, y, theta, v, omega]
        """

        x, y, theta, v, omega = state.flatten()

        dt = self.dt

        return np.array(
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
    def compute_jacobian(
        self,
        state: np.ndarray,
    ) -> np.ndarray:
        """
        Compute the Jacobian of the nonlinear
        motion model with respect to the state.
        """

        _, _, theta, v, _ = state.flatten()

        dt = self.dt

        return np.array(
            [
                [
                    1,
                    0,
                    -v * np.sin(theta) * dt,
                    np.cos(theta) * dt,
                    0,
                ],
                [
                    0,
                    1,
                    v * np.cos(theta) * dt,
                    np.sin(theta) * dt,
                    0,
                ],
                [
                    0,
                    0,
                    1,
                    0,
                    dt,
                ],
                [
                    0,
                    0,
                    0,
                    1,
                    0,
                ],
                [
                    0,
                    0,
                    0,
                    0,
                    1,
                ],
            ],
            dtype=float,
        )

    def initialize(
        self,
        initial_state: np.ndarray,
        initial_uncertainty: float = 1.0,
    ):
        """
        Initialize the EKF state and covariance.
        """

        self.x = initial_state.astype(float).reshape(5, 1)

        self.P = np.eye(5) * initial_uncertainty
    
    def predict(self):
        """
        Predict the next state and covariance.
        """

        if self.x is None or self.P is None:
            raise RuntimeError(
                "EKF must be initialized before prediction."
            )

        # Compute Jacobian around current state
        F = self.compute_jacobian(self.x)

        # Propagate state through nonlinear model
        self.x = self.motion_model(self.x)

        # Keep heading within [-pi, pi]
        self.x[2, 0] = self.normalize_angle(self.x[2, 0])

        # Propagate covariance using the Jacobian
        self.P = F @ self.P @ F.T + self.Q

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize an angle to [-pi, pi].
        """

        return (angle + np.pi) % (2 * np.pi) - np.pi
    def update(self, measurement: np.ndarray):
        """
        Update the EKF using a GPS position measurement.
        """

        if self.x is None or self.P is None:
            raise RuntimeError(
                "EKF must be initialized before update."
            )

        # GPS measurement model
        H = np.array(
            [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
            ],
            dtype=float,
        )

        # Innovation / residual
        innovation = measurement - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ innovation

        # Update covariance
        I = np.eye(5)

        self.P = (I - K @ H) @ self.P