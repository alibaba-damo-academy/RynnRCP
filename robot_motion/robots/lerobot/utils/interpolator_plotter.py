"""
Interpolator Plotter for Offline Visualization.

This module provides offline plotting capabilities for interpolator visualization,
showing original vs interpolated trajectories in a subplot layout.
Data is collected during runtime and plots are generated after completion.

Optimized for high-frequency data collection with minimal performance impact.
"""

import time
import numpy as np
from typing import List, Optional, Tuple
import logging
import matplotlib.pyplot as plt
import platform


class InterpolatorPlotter:
    """
    Optimized offline interpolator plotter for policy interpolation visualization.

    Uses pre-allocated numpy arrays for efficient data storage and minimal
    performance impact during high-frequency data collection.
    """

    def __init__(
        self, joint_names: List[str], max_trajectories: int = 100, log_dir: str = None
    ):
        """
        Initialize the interpolator plotter.

        Args:
            joint_names: List of joint names for labeling
            max_trajectories: Maximum number of trajectories to store
            log_dir: Base directory for saving plots (from config.yaml)
        """
        self.joint_names = joint_names
        self.max_trajectories = max_trajectories
        self.num_joints = len(joint_names)
        self.log_dir = log_dir or "~/logs/lerobot"
        self.logger = logging.getLogger(__name__)

        # Storage for trajectory data
        self.trajectories = []  # List of trajectory dictionaries
        self.trajectory_count = 0

        self.logger.info(
            f"Optimized interpolator plotter initialized for {self.num_joints} joints (offline mode)"
        )

    def add_trajectory_data(
        self,
        qPos_input: np.ndarray,
        qPos_output: np.ndarray,
        num_joints: int,
        chunk_size: int,
        timestamp: float = None,
    ):
        """
        Add new trajectory data efficiently.

        Args:
            qPos_input: Original joint positions from policy
            qPos_output: Interpolated joint positions
            num_joints: Number of joints
            chunk_size: Size of the trajectory chunk
            timestamp: Optional timestamp for the trajectory
        """
        if timestamp is None:
            timestamp = time.time()

        trajectory_data = {
            "timestamp": timestamp,
            "qPos_input": np.array(qPos_input).copy(),
            "qPos_output": np.array(qPos_output).copy(),
            "num_joints": num_joints,
            "chunk_size": chunk_size,
        }

        if len(self.trajectories) >= self.max_trajectories:
            self.trajectories.pop(0)

        self.trajectories.append(trajectory_data)
        self.trajectory_count += 1

    def generate_plots(self, save_path: Optional[str] = None, show: bool = True):
        """
        Generate offline plots from collected trajectory data.

        Args:
            save_path: Path to save the plot file (optional)
            show: Whether to display the plot (default: True)
        """
        if len(self.trajectories) == 0:
            self.logger.warning("No trajectory data to plot")
            return

        try:
            if platform.system() == "Darwin":  # macOS
                import matplotlib

                plt.style.use("seaborn-v0_8-whitegrid")

                matplotlib.rcParams.update(
                    {
                        "font.family": "sans-serif",
                        "font.sans-serif": ["DejaVu Sans"],
                        "font.size": 8,
                        "figure.figsize": [6.4, 4.8],
                        "figure.dpi": 100,
                        "figure.facecolor": "white",
                        "axes.facecolor": "white",
                        "savefig.facecolor": "white",
                        "text.usetex": False,
                        "mathtext.default": "regular",
                    }
                )
            else:  # Ubuntu and others
                plt.style.use("seaborn-v0_8-whitegrid")

            trajectories_to_plot = (
                self.trajectories[-3:]
                if len(self.trajectories) > 3
                else self.trajectories
            )

            for traj_idx, trajectory in enumerate(trajectories_to_plot):
                fig, axes = plt.subplots(2, 3, figsize=(15, 10))

                fig.suptitle(
                    f"Interpolator Trajectory {traj_idx + 1}: Policy (Blue) vs Interpolated (Red)",
                    fontsize=16,
                )

                axes = axes.flatten()

                original_pos = trajectory["qPos_input"]
                interpolated_pos = trajectory["qPos_output"]
                num_joints = trajectory["num_joints"]
                chunk_size = trajectory["chunk_size"]

                if len(original_pos) == num_joints * chunk_size:
                    original_reshaped = np.array(original_pos).reshape(
                        chunk_size, num_joints
                    )
                    interpolated_reshaped = np.array(interpolated_pos).reshape(
                        -1, num_joints
                    )
                else:
                    self.logger.warning(
                        f"Unexpected data shape for trajectory {traj_idx}"
                    )
                    continue

                original_time = np.arange(chunk_size)
                interpolated_time = np.linspace(
                    0, chunk_size - 1, len(interpolated_reshaped)
                )

                for i in range(min(num_joints, 6)):  # Limit to 6 joints for 2x3 layout
                    ax = axes[i]
                    joint_name = (
                        self.joint_names[i]
                        if i < len(self.joint_names)
                        else f"Joint_{i+1}"
                    )

                    ax.plot(
                        original_time,
                        original_reshaped[:, i],
                        "bo-",
                        linewidth=2,
                        markersize=8,
                        label="Original",
                    )
                    ax.plot(
                        interpolated_time,
                        interpolated_reshaped[:, i],
                        "ro--",
                        linewidth=2,
                        markersize=4,
                        alpha=0.7,
                        label="Interpolated",
                    )

                    ax.set_title(f"Joint {i+1}: {joint_name}")
                    ax.set_xlabel("Time(sec)")
                    if i == num_joints - 1:  # Assuming last joint is the gripper
                        ax.set_ylabel("Opening Degree (%)")
                    else:
                        ax.set_ylabel("Position (rad)")
                    ax.legend()
                    ax.grid(True, alpha=0.3)

                for i in range(num_joints, 6):
                    axes[i].set_visible(False)

                plt.tight_layout()

                if save_path:
                    base_path = save_path.replace(".png", f"_traj_{traj_idx + 1}.png")
                    plt.savefig(base_path, dpi=300, bbox_inches="tight")
                    self.logger.info(f"Interpolator plot saved to: {base_path}")

                if show:
                    plt.show()
                else:
                    plt.close(fig)

        except Exception as e:
            self.logger.error(f"Failed to generate interpolator plots: {e}")
            raise

    def close(self):
        """Close any open plot windows and cleanup resources."""
        try:
            plt.close("all")
            self.logger.info("Interpolator plotter closed")
        except Exception as e:
            self.logger.warning(f"Error closing interpolator plotter: {e}")

    def is_available(self) -> bool:
        """Check if plotting functionality is available."""
        return True

    def get_stats(self) -> dict:
        """Get statistics about the stored data."""
        return {
            "status": "enabled",
            "num_joints": self.num_joints,
            "stored_trajectories": len(self.trajectories),
            "max_trajectories": self.max_trajectories,
            "total_trajectories": self.trajectory_count,
        }

    def make_plots(self, mode: str, show_plots: bool = False) -> str:
        """
        Generate and save interpolator plots from collected trajectory data.

        Args:
            mode: Control mode (sim, real, mock) for filename
            show_plots: Whether to display plots or just save them

        Returns:
            Path to the saved plot file (base path, actual files will have trajectory numbers)

        Raises:
            Exception: If plot generation fails
        """
        import os
        import platform
        from datetime import datetime

        try:
            # Create interpolator plots directory if it doesn't exist
            base_log_dir = os.path.expanduser(self.log_dir)
            plots_dir = os.path.join(base_log_dir, "interpolate_plots")
            os.makedirs(plots_dir, exist_ok=True)

            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_filename = f"interpolator_positions_{mode}_{timestamp}.png"
            plot_path = os.path.join(plots_dir, plot_filename)

            # Determine whether to show plots based on platform and user preference
            should_show = show_plots and platform.system() != "Darwin"

            # Additional matplotlib configuration for macOS
            if platform.system() == "Darwin":
                import matplotlib

                matplotlib.rcParams["font.family"] = "DejaVu Sans"
                matplotlib.rcParams["figure.max_open_warning"] = 0

            # Generate the plots
            self.generate_plots(save_path=plot_path, show=should_show)

            # Get stats about generated plots
            stats = self.get_stats()
            trajectories_plotted = stats.get("stored_trajectories", 0)

            if trajectories_plotted > 0:
                print(f"✓ Interpolator Plot file: {plot_path}")
                self.logger.info(
                    f"Interpolator plots saved: {plot_path} ({trajectories_plotted} trajectories)"
                )
                return plot_path
            else:
                self.logger.info("No interpolator trajectory data to plot")
                return plot_path

        except Exception as e:
            self.logger.error(f"Failed to generate interpolator plots: {e}")
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")
            raise
