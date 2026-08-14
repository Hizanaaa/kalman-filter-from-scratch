import numpy as np


class OdometrySensor:
    """
    Simulates noisy odometry measurements of
    linear and angular velocity.
    """

    def __init__(
        self,
        velocity_noise_std: float,
        angular_velocity_noise_std: float,
        velocity_bias_drift_std: float,
        angular_velocity_bias_drift_std: float,
        random_seed: int | None = None,
    ):
        self.velocity_noise_std = velocity_noise_std
        self.angular_velocity_noise_std = (
            angular_velocity_noise_std
        )

        self.velocity_bias_drift_std = (
            velocity_bias_drift_std
        )

        self.angular_velocity_bias_drift_std = (
            angular_velocity_bias_drift_std
        )

        self.rng = np.random.default_rng(random_seed)

        # Slowly changing sensor biases
        self.velocity_bias = 0.0
        self.angular_velocity_bias = 0.0

    def measure(
        self,
        true_velocity: float,
        true_angular_velocity: float,
    ) -> tuple[float, float]:
        """
        Generate noisy velocity and angular velocity
        measurements with slowly drifting biases.
        """

        # Random-walk bias drift
        self.velocity_bias += self.rng.normal(
            0.0,
            self.velocity_bias_drift_std,
        )

        self.angular_velocity_bias += self.rng.normal(
            0.0,
            self.angular_velocity_bias_drift_std,
        )

        # Measurement noise
        velocity_noise = self.rng.normal(
            0.0,
            self.velocity_noise_std,
        )

        angular_velocity_noise = self.rng.normal(
            0.0,
            self.angular_velocity_noise_std,
        )

        measured_velocity = (
            true_velocity
            + self.velocity_bias
            + velocity_noise
        )

        measured_angular_velocity = (
            true_angular_velocity
            + self.angular_velocity_bias
            + angular_velocity_noise
        )

        return (
            measured_velocity,
            measured_angular_velocity,
        )

