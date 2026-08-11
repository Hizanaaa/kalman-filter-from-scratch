from dataclasses import dataclass


@dataclass
class RobotState:
    """
    Represents the true state of the robot.
    """

    x: float
    y: float
    vx: float
    vy: float