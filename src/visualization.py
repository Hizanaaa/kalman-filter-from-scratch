import matplotlib.pyplot as plt
from pathlib import Path


def plot_trajectory(ground_truth, gps_measurements):
    """
    Plot the true robot trajectory and noisy GPS measurements.
    """

    true_x = [state.x for state in ground_truth]
    true_y = [state.y for state in ground_truth]

    gps_x = [gps[0, 0] for gps in gps_measurements]
    gps_y = [gps[1, 0] for gps in gps_measurements]

    plt.figure(figsize=(8, 6))

    plt.plot(
        true_x,
        true_y,
        color="blue",
        linewidth=2,
        label="Ground Truth",
    )

    plt.scatter(
        gps_x,
        gps_y,
        color="red",
        s=40,
        label="GPS Measurements",
    )

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

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Ground Truth vs Noisy GPS Measurements")
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