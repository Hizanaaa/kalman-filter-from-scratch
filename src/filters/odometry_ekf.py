import numpy as np


class OdometryEKF:
    """
    3-state EKF for 2D robot localization.

    State:
        [x, y, theta]

    Control input:
        [v, omega]

    GPS measures:
        [x, y]
    """

    def __init__(
        self,
        dt: float,
        process_variance: float,
        measurement_variance: float,
    ):
        self.dt = dt

        self.x = None
        self.P = None

        self.process_variance = process_variance

        self.R = np.eye(2) * measurement_variance

    def initialize(
        self,
        initial_state: np.ndarray,
        initial_uncertainty: float = 1.0,
    ):
        self.x = initial_state.astype(float).reshape(3, 1)
        self.P = np.eye(3) * initial_uncertainty

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_jacobian(
        self,
        velocity: float,
    ) -> np.ndarray:

        theta = self.x[2, 0]
        dt = self.dt

        return np.array(
            [
                [
                    1.0,
                    0.0,
                    -velocity * np.sin(theta) * dt,
                ],
                [
                    0.0,
                    1.0,
                    velocity * np.cos(theta) * dt,
                ],
                [
                    0.0,
                    0.0,
                    1.0,
                ],
            ]
        )

    def compute_control_noise_jacobian(self) -> np.ndarray:

        theta = self.x[2, 0]
        dt = self.dt

        return np.array(
            [
                [
                    np.cos(theta) * dt,
                    0.0,
                ],
                [
                    np.sin(theta) * dt,
                    0.0,
                ],
                [
                    0.0,
                    dt,
                ],
            ]
        )

    def predict(
        self,
        velocity: float,
        angular_velocity: float,
    ):
        """
        Predict using measured odometry as the control input.
        """

        theta = self.x[2, 0]
        dt = self.dt

        self.x[0, 0] += (
            velocity * np.cos(theta) * dt
        )

        self.x[1, 0] += (
            velocity * np.sin(theta) * dt
        )

        self.x[2, 0] += (
            angular_velocity * dt
        )

        self.x[2, 0] = self.normalize_angle(
            self.x[2, 0]
        )

        F = self.compute_jacobian(
            velocity
        )

        G = self.compute_control_noise_jacobian()

        control_noise = (
            self.process_variance
            * np.eye(2)
        )

        self.P = (
            F @ self.P @ F.T
            + G @ control_noise @ G.T
        )

    def update(self, measurement: np.ndarray):
        """
        GPS position update.
        """

        H = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ]
        )

        innovation = (
            measurement
            - H @ self.x
        )

        S = (
            H @ self.P @ H.T
            + self.R
        )

        K = (
            self.P
            @ H.T
            @ np.linalg.inv(S)
        )

        self.x = (
            self.x
            + K @ innovation
        )

        I = np.eye(3)

        self.P = (
            (I - K @ H)
            @ self.P
        )
