import numpy as np

from filters.odometry_ekf import OdometryEKF


def test_odometry_ekf_prediction():

    ekf = OdometryEKF(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    initial_state = np.array(
        [
            [0.0],
            [0.0],
            [0.0],
        ]
    )

    ekf.initialize(initial_state)

    ekf.predict(
        velocity=2.0,
        angular_velocity=0.1,
    )

    assert np.isclose(ekf.x[0, 0], 2.0)
    assert np.isclose(ekf.x[1, 0], 0.0)
    assert np.isclose(ekf.x[2, 0], 0.1)


def test_odometry_ekf_gps_update():

    ekf = OdometryEKF(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    initial_state = np.array(
        [
            [0.0],
            [0.0],
            [0.0],
        ]
    )

    ekf.initialize(initial_state)

    ekf.predict(
        velocity=2.0,
        angular_velocity=0.1,
    )

    predicted_position = ekf.x[:2].copy()

    gps = np.array(
        [
            [1.8],
            [0.2],
        ]
    )

    ekf.update(gps)

    # GPS update should move the estimate toward the measurement.
    assert np.linalg.norm(
        ekf.x[:2] - gps
    ) < np.linalg.norm(
        predicted_position - gps
    )

    # Covariance should decrease after receiving GPS.
    assert np.trace(ekf.P) < 7.1
