import numpy as np
from scipy.interpolate import (
    CubicSpline,
    splrep,
    BSpline,
    InterpolatedUnivariateSpline,
    LSQUnivariateSpline,
    make_interp_spline,
)

import yaml


def lerp(start_pos, target_pos, current_time, duration):
    """
    Linear interpolation from start to target over a duration.

    Args:
        start_pos: Starting position (scalar or array)
        target_pos: Target position (scalar or array)
        current_time: Current time
        duration: Total duration for interpolation

    Returns:
        Interpolated position at current_time
    """
    if duration <= 0:
        return target_pos

    if current_time <= 0:
        return start_pos

    if current_time >= duration:
        return target_pos

    t = current_time / duration
    return (1 - t) * start_pos + t * target_pos


def create_cubic_spline_with_boundary_conditions(x_values, y_values):
    """Create a cubic spline with boundary conditions."""
    try:
        with open("configs/config.yaml", "r") as f:
            config = yaml.safe_load(f)
        boundary_condition = config.get("spline", {}).get(
            "boundary_condition", "natural"
        )
    except FileNotFoundError:
        boundary_condition = "natural"
        print(f"Warning: Config file configs/config.yaml not found, using defaults")

    bc_type = boundary_condition

    cs = CubicSpline(x_values, y_values, bc_type=bc_type)
    return cs


def create_B_spline_with_boundary_conditions(x_values, y_values):
    """Create a B-spline with boundary conditions."""
    t = np.linspace(0, 1, len(x_values))
    tck = splrep(t, y_values, k=3)
    return BSpline(*tck)


def create_interpolated_univariate_spline(x_values, y_values):
    """Create an interpolated univariate spline."""
    return InterpolatedUnivariateSpline(x_values, y_values, k=5)


def create_LSQ_univariate_spline(x_values, y_values):
    """Create an LSQ univariate spline."""
    return LSQUnivariateSpline(x_values, y_values)


def create_make_interp_spline(x_values, y_values):
    """Create a make_interp_spline."""
    return make_interp_spline(x_values, y_values)


class PolicyInterpolator:
    def __init__(self):
        """Initialize the policy interpolator."""
        self._ndof = 0
        self._chunk_size = 0
        self._piecewise_curves = None

    def prepare_trajectory(
        self,
        joint_positions,
        ndof,
        chunk_size,
    ):
        """
        Prepare piecewise curve representation for trajectory interpolation.

        Converts flattened joint position data into internal spline objects for
        real-time trajectory interpolation during robot control.

        Args:
            joint_positions: Flattened trajectory data in format [j0_t0, j1_t0, ..., j0_t1, j1_t1, ...]
            ndof: Number of robot joints
            chunk_size: Number of waypoints per joint (trajectory length)
        """
        joint_positions = np.array(joint_positions)
        self._ndof = ndof
        self._chunk_size = chunk_size

        rearranged_positions = np.zeros((ndof, chunk_size))
        for i in range(ndof):
            rearranged_positions[i] = joint_positions[i::ndof]

        self._piecewise_curves = []

        for i in range(ndof):
            x_values = np.arange(chunk_size)
            cs = create_cubic_spline_with_boundary_conditions(
                x_values, rearranged_positions[i]
            )
            self._piecewise_curves.append(cs)

    def update(self, _index):
        """
        Evaluate all spline curves at a specific trajectory index.

        Used for real-time trajectory execution to get joint positions
        at any index point within the trajectory.

        Args:
            _index: Trajectory index to evaluate (0 to chunk_size-1)

        Returns:
            list: Joint positions at index _index, one value per joint
        """
        if self._piecewise_curves is None:
            raise ValueError("No trajectory prepared. Call prepare_trajectory() first.")

        return [cs(_index) for cs in self._piecewise_curves]

    def offline_trajgen(
        self,
        joint_positions,
        ndof,
        chunk_size,
        input_freq,
        output_freq,
    ):
        """
        Generate complete interpolated trajectory for offline analysis.

        Pre-computes entire trajectory at higher frequency for visualization,
        analysis, or batch processing. Not used in real-time control.

        Args:
            joint_positions: Flattened trajectory data in format [j0_t0, j1_t0, ..., j0_t1, j1_t1, ...]
            ndof: Number of robot joints
            chunk_size: Number of waypoints per joint (trajectory length)
            input_freq: Original trajectory frequency (Hz)
            output_freq: Desired output frequency (Hz)

        Returns:
            tuple: Flattened interpolated trajectory at output_freq
        """

        joint_positions = np.array(joint_positions)

        rearranged_positions = np.zeros((ndof, chunk_size))
        for i in range(ndof):
            rearranged_positions[i] = joint_positions[i::ndof]

        size = (
            chunk_size
            if chunk_size <= 1
            else int((chunk_size - 1) * 1 / input_freq * output_freq)
        )
        sampled_data = np.zeros((1, size * ndof))

        if chunk_size <= 1:
            for i in range(ndof):
                for j in range(size):
                    index = j * ndof + i
                    sampled_data[0][index] = joint_positions[i]
        else:
            for i in range(ndof):
                x_values = np.arange(chunk_size)
                cs = create_cubic_spline_with_boundary_conditions(
                    x_values, rearranged_positions[i]
                )

                t_samples = np.linspace(0, chunk_size - 1, size)
                sample = [cs(t) for t in t_samples]

                for j in range(len(sample)):
                    index = j * ndof + i
                    sampled_data[0][index] = sample[j]

        sampled_data = tuple(sampled_data[0])

        return sampled_data
