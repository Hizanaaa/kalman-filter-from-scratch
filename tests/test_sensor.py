from simulation.sensors import OdometrySensor


def test_odometry_sensor_measurement():

    sensor = OdometrySensor(
        velocity_noise_std=0.05,
        angular_velocity_noise_std=0.01,
        velocity_bias_drift_std=0.002,
        angular_velocity_bias_drift_std=0.001,
        random_seed=42,
    )

    velocity, angular_velocity = sensor.measure(
        true_velocity=2.0,
        true_angular_velocity=0.1,
    )

    assert isinstance(velocity, float)
    assert isinstance(angular_velocity, float)

    assert abs(velocity - 2.0) < 0.5
    assert abs(angular_velocity - 0.1) < 0.1
