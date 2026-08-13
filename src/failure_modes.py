import numpy as np
import matplotlib.pyplot as plt
from simulation.simulator import RobotSimulator
from simulation.models import RobotState
from filters.kalman_filter import KalmanFilter
from pathlib import Path


def run_gps_dropout_experiment(
    dropout_start: int = 40,
    dropout_end: int = 70,
    num_steps: int = 100,
):
    """
    Evaluate Kalman Filter behavior during a GPS outage.
    """

    simulator = RobotSimulator(
        dt=1.0,
        random_seed=42,
    )

    initial_state = RobotState(
        x=0.0,
        y=0.0,
        vx=2.0,
        vy=1.0,
    )

    simulation = simulator.simulate(
        initial_state=initial_state,
        num_steps=num_steps,
    )

    kf = KalmanFilter(
        dt=1.0,
        process_variance=0.05,
        measurement_variance=0.5,
    )

    kf.initialize(initial_state)

    estimates = []
    covariance_trace = []

    for i, gps in enumerate(simulation.gps_measurements):

        if i == 0:
            kf.update(gps)

        else:
            kf.predict()

            # GPS is unavailable during the dropout.
            if not (dropout_start <= i < dropout_end):
                kf.update(gps)

        estimates.append(
            RobotState.from_vector(kf.x)
        )

        covariance_trace.append(
            np.trace(kf.P)
        )

    return (
        simulation,
        estimates,
        covariance_trace,
    )

def calculate_position_errors(simulation, estimates):
    """
    Calculate Euclidean position error at each timestep.
    """

    true_positions = np.array(
        [
            [state.x, state.y]
            for state in simulation.ground_truth
        ]
    )

    estimated_positions = np.array(
        [
            [state.x, state.y]
            for state in estimates
        ]
    )

    errors = np.linalg.norm(
        estimated_positions - true_positions,
        axis=1,
    )

    return errors

def plot_failure_mode(
    position_errors,
    covariance_trace,
    dropout_start,
    dropout_end,
):
    """
    Plot position error and covariance during GPS dropout.
    """

    steps = np.arange(len(position_errors))

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    # Position error plot
    plt.figure(figsize=(10, 6))

    plt.plot(
        steps,
        position_errors,
        linewidth=2,
        label="Position Error",
    )

    plt.axvspan(
        dropout_start,
        dropout_end - 1,
        alpha=0.2,
        label="GPS Dropout",
    )

    plt.xlabel("Time Step")
    plt.ylabel("Position Error (m)")
    plt.title("Kalman Filter Position Error During GPS Dropout")
    plt.legend()
    plt.grid(True)

    plt.savefig(
        results_dir / "gps_dropout_error.png",
        dpi=300,
        bbox_inches="tight",
    )

    plt.close()

    # Covariance plot
    plt.figure(figsize=(10, 6))

    plt.plot(
        steps,
        covariance_trace,
        linewidth=2,
        label="Covariance Trace",
    )

    plt.axvspan(
        dropout_start,
        dropout_end - 1,
        alpha=0.2,
        label="GPS Dropout",
    )

    plt.xlabel("Time Step")
    plt.ylabel("Trace(P)")
    plt.title("State Uncertainty During GPS Dropout")
    plt.legend()
    plt.grid(True)

    plt.savefig(
        results_dir / "gps_dropout_covariance.png",
        dpi=300,
        bbox_inches="tight",
    )

    plt.close()

def main():

    simulation, estimates, covariance_trace = (
        run_gps_dropout_experiment()
    )

    position_errors = calculate_position_errors(
        simulation,
        estimates,
    )

    print(
        f"Position error before dropout: "
        f"{position_errors[39]:.4f} m"
    )

    print(
        f"Position error during dropout: "
        f"{position_errors[69]:.4f} m"
    )

    print(
        f"Position error after GPS recovery: "
        f"{position_errors[70]:.4f} m"
    )

    print("\nGPS Dropout Experiment")
    print("----------------------")

    print("GPS dropout: steps 40-69")

    print(
        f"Covariance trace before dropout: "
        f"{covariance_trace[39]:.4f}"
    )

    print(
        f"Covariance trace during dropout: "
        f"{covariance_trace[69]:.4f}"
    )

    print(
        f"Covariance trace after GPS recovery: "
        f"{covariance_trace[70]:.4f}"
    )

    plot_failure_mode(
        position_errors,
        covariance_trace,
        dropout_start=40,
        dropout_end=70,
    )


if __name__ == "__main__":
    main()