import numpy as np


def process_noise(std: float = 0.05) -> np.ndarray:
    """
    Generate Gaussian process noise.

    Returns
    -------
    np.ndarray
        4x1 noise vector.
    """

    noise = np.random.normal(
        loc=0.0,
        scale=std,
        size=(4, 1),
    )

    return noise