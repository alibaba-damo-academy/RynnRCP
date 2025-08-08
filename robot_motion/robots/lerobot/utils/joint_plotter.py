"""
Joint Position Plotter for Offline Visualization.

This module provides offline plotting capabilities for joint positions,
showing command vs feedback values in a 2x3 subplot layout.
Data is collected during runtime and plots are generated after completion.

Optimized for high-frequency data collection with minimal performance impact.
"""

import time
import numpy as np
from typing import List, Optional
import logging
import matplotlib.pyplot as plt
import platform


class JointPlotter:
    """
    Optimized offline joint position plotter for robot control visualization.

    Uses pre-allocated numpy arrays for efficient data storage and minimal
    performance impact during high-frequency data collection.
    """

    def __init__(
        self, joint_names: List[str], max_points: int = 10000, log_dir: str = None
    ):
        """
        Initialize the joint plotter with pre-allocated arrays.

        Args:
            joint_names: List of joint names for labeling
            max_points: Maximum number of data points to store
            log_dir: Base directory for saving plots (from config.yaml)
        """
        self.joint_names = joint_names
        self.max_points = max_points
        self.num_joints = len(joint_names)
        self.log_dir = log_dir or "~/logs/lerobot"
        self.logger = logging.getLogger(__name__)

        self.time_data = np.zeros(max_points, dtype=np.float64)
        self.command_data = np.zeros((self.num_joints, max_points), dtype=np.float64)
        self.feedback_data = np.zeros((self.num_joints, max_points), dtype=np.float64)
        self.current_idx = 0
        self.data_count = 0

        self.logger.info(
            f"Optimized joint plotter initialized for {self.num_joints} joints (offline mode)"
        )

    def generate_plots(self, save_path: Optional[str] = None, show: bool = True):
        """
        Generate offline plots from collected data.

        Args:
            save_path: Path to save the plot file (optional)
            show: Whether to display the plot (default: True)
        """
        if self.data_count == 0:
            self.logger.warning("No data to plot")
            return

        try:
            if platform.system() == "Darwin":
                import matplotlib

                plt.style.use("seaborn-v0_8-whitegrid")

                try:
                    from matplotlib import font_manager

                    font_manager._get_fontconfig_fonts.cache_clear()
                except:
                    pass

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
                        "font.serif": ["DejaVu Serif"],
                        "font.monospace": ["DejaVu Sans Mono"],
                        "font.cursive": ["DejaVu Sans"],
                        "font.fantasy": ["DejaVu Sans"],
                    }
                )
            else:
                plt.style.use("seaborn-v0_8-whitegrid")

            valid_count = min(self.data_count, self.max_points)
            if self.data_count > self.max_points:
                start_idx = self.current_idx
                time_array = np.concatenate(
                    [self.time_data[start_idx:], self.time_data[:start_idx]]
                )
                cmd_data = np.concatenate(
                    [
                        self.command_data[:, start_idx:],
                        self.command_data[:, :start_idx],
                    ],
                    axis=1,
                )
                fb_data = np.concatenate(
                    [
                        self.feedback_data[:, start_idx:],
                        self.feedback_data[:, :start_idx],
                    ],
                    axis=1,
                )
            else:
                # Simple case: use data as-is
                time_array = self.time_data[:valid_count]
                cmd_data = self.command_data[:, :valid_count]
                fb_data = self.feedback_data[:, :valid_count]

            # Create figure and subplots
            fig, axes = plt.subplots(2, 3, figsize=(15, 10))

            # Add title for both platforms
            fig.suptitle(
                "Joint Positions: Command (Blue) vs Feedback (Red Dashed)",
                fontsize=16,
            )

            # Flatten axes for easier indexing
            axes = axes.flatten()

            for i in range(min(self.num_joints, 6)):  # Limit to 6 joints for 2x3 layout
                ax = axes[i]
                joint_name = (
                    self.joint_names[i] if i < len(self.joint_names) else f"Joint_{i+1}"
                )

                # Plot data directly from arrays (no copying)
                ax.plot(time_array, cmd_data[i], "b-", linewidth=2, label="Command")
                ax.plot(
                    time_array, fb_data[i], "r--", linewidth=2, label="Feedback"
                )  # Dashed red line

                # Add styling for both platforms
                ax.set_title(f"Joint {i+1}: {joint_name}")
                ax.set_xlabel("Time (s)")
                if i == min(self.num_joints, 6) - 1:
                    ax.set_ylabel("Opening Degree (%)")
                else:
                    ax.set_ylabel("Position (rad)")
                ax.legend()
                ax.grid(True, alpha=0.3)

                # Auto-scale using min/max (no concatenation)
                if valid_count > 0:
                    data_min = min(np.min(cmd_data[i]), np.min(fb_data[i]))
                    data_max = max(np.max(cmd_data[i]), np.max(fb_data[i]))
                    data_range = data_max - data_min
                    padding = max(0.1, data_range * 0.1)
                    ax.set_ylim(data_min - padding, data_max + padding)

            # Hide unused subplots
            for i in range(self.num_joints, 6):
                axes[i].set_visible(False)

            plt.tight_layout()

            # Save plot if path provided
            if save_path:
                plt.savefig(save_path, dpi=300, bbox_inches="tight")
                self.logger.info(f"Plot saved to: {save_path}")

            # Show plot if requested
            if show:
                plt.show()

            return fig

        except Exception as e:
            self.logger.error(f"Failed to generate plots: {e}")
            raise

    def add_data(
        self,
        command_positions: np.ndarray,
        feedback_positions: np.ndarray,
        current_time: float,
    ):
        """
        Add new data points efficiently using pre-allocated arrays.

        Args:
            command_positions: Array of commanded joint positions
            feedback_positions: Array of actual joint positions
            current_time: Current time value
        """
        # Store time (single value)
        self.time_data[self.current_idx] = current_time

        # Store joint data efficiently (vectorized operations)
        num_cmd = min(len(command_positions), self.num_joints)
        num_fb = min(len(feedback_positions), self.num_joints)

        # Use array slicing for maximum performance
        self.command_data[:num_cmd, self.current_idx] = command_positions[:num_cmd]
        self.feedback_data[:num_fb, self.current_idx] = feedback_positions[:num_fb]

        # Fill remaining joints with zeros if needed
        if num_cmd < self.num_joints:
            self.command_data[num_cmd:, self.current_idx] = 0.0
        if num_fb < self.num_joints:
            self.feedback_data[num_fb:, self.current_idx] = 0.0

        # Update circular buffer index
        self.current_idx = (self.current_idx + 1) % self.max_points
        self.data_count += 1

    def update_plots(self):
        """No-op method for compatibility. Plots are generated offline."""
        pass

    def close(self):
        """Close any open plot windows and cleanup resources."""
        try:
            plt.close("all")  # Close all matplotlib figures
            self.logger.info("Joint plotter closed")
        except Exception as e:
            self.logger.warning(f"Error closing plotter: {e}")

    def is_available(self) -> bool:
        """Check if plotting functionality is available."""
        return True

    def get_stats(self) -> dict:
        """Get statistics about the stored data."""
        return {
            "status": "enabled",
            "num_joints": self.num_joints,
            "data_points": min(self.data_count, self.max_points),
            "max_points": self.max_points,
            "buffer_usage": f"{(min(self.data_count, self.max_points) / self.max_points * 100):.1f}%",
        }

    def make_plots(self, mode: str, show_plots: bool = False) -> str:
        """
        Generate and save joint plots from collected data.

        Args:
            mode: Control mode (sim, real, mock) for filename
            show_plots: Whether to display plots or just save them

        Returns:
            Path to the saved plot file

        Raises:
            Exception: If plot generation fails
        """
        import os
        import platform
        from datetime import datetime

        try:
            # Create joint plots directory if it doesn't exist
            base_log_dir = os.path.expanduser(self.log_dir)
            plots_dir = os.path.join(base_log_dir, "joint_plots")
            os.makedirs(plots_dir, exist_ok=True)

            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_filename = f"joint_positions_{mode}_{timestamp}.png"
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

            # Verify the file was actually created
            if os.path.exists(plot_path):
                file_size = os.path.getsize(plot_path)
                print(f"✓ Joint Plot file: {plot_path}")
                self.logger.info(f"Joint plot saved: {plot_path} ({file_size} bytes)")
                return plot_path
            else:
                error_msg = (
                    f"Joint plot file was not created at expected path: {plot_path}"
                )
                self.logger.warning(error_msg)
                raise Exception(error_msg)

        except Exception as e:
            self.logger.error(f"Failed to generate joint plots: {e}")
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")
            raise
