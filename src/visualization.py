import matplotlib.pyplot as plt
from pathlib import Path


def plot_trajectory(
    ground_truth,
    gps_measurements,
    estimates,
):
    """
    Plot ground truth, noisy GPS measurements,
    and Kalman Filter estimates.
    """

    true_x = [state.x for state in ground_truth]
    true_y = [state.y for state in ground_truth]

    gps_x = [gps[0, 0] for gps in gps_measurements]
    gps_y = [gps[1, 0] for gps in gps_measurements]

    estimate_x = [state.x for state in estimates]
    estimate_y = [state.y for state in estimates]

    plt.figure(figsize=(10, 6))

    plt.plot(
        true_x,
        true_y,
        color="blue",
        linewidth=2.5,
        label="Ground Truth",
    )

    plt.scatter(
        gps_x,
        gps_y,
        color="red",
        s=60,
        alpha=0.8,
        label="GPS Measurements",
    )

    plt.plot(
        estimate_x,
        estimate_y,
        color="green",
        linewidth=2.5,
        label="Kalman Estimate",
    )

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Ground Truth vs GPS vs Kalman Filter")

    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.axis("equal")

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    output_file = results_dir / "trajectory.png"

    plt.savefig(
        output_file,
        dpi=300,
        bbox_inches="tight",
    )

    print(f"Trajectory plot saved to: {output_file}")

    plt.close()