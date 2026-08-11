from dataclasses import dataclass
import numpy as np


@dataclass
class RobotState:
    """
    Represents the robot state.

    State vector:
    [x,
     y,
     vx,
     vy]
    """

    x: float
    y: float
    vx: float
    vy: float

    def as_vector(self) -> np.ndarray:
        """
        Convert the state to a 4x1 NumPy column vector.
        """
        return np.array(
            [
                [self.x],
                [self.y],
                [self.vx],
                [self.vy],
            ],
            dtype=float,
        )

    @classmethod
    def from_vector(cls, vector: np.ndarray):
        """
        Create a RobotState from a 4x1 vector.
        """
        return cls(
            x=float(vector[0, 0]),
            y=float(vector[1, 0]),
            vx=float(vector[2, 0]),
            vy=float(vector[3, 0]),
        )