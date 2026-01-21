import numpy as np
from scipy.interpolate import (
    CubicSpline,
    splrep,
    BSpline,
    InterpolatedUnivariateSpline,
    LSQUnivariateSpline,
    make_interp_spline,
)
from scipy.optimize import minimize

import yaml


def lerp(q0, q1, tNow, tDur):
    """
    Linear interpolation from start to target over a duration.

    Args:
        q0: Starting position (scalar or array)
        q1: Target position (scalar or array)
        tNow: Current time
        tDur: Total duration for interpolation

    Returns:
        qCmd: Interpolated position at tNow
    """
    if tDur <= 0:
        return q1

    if tNow <= 0:
        return q0

    if tNow >= tDur:
        return q1

    t = tNow / tDur

    qCmd = (1 - t) * q0 + t * q1
    return qCmd


def create_cubic_spline_with_boundary_conditions(x_values, y_values, config_path):
    """Create a cubic spline with boundary conditions."""
    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        boundary_condition = config.get("spline", {}).get(
            "boundary_condition", "natural"
        )
    except FileNotFoundError:
        boundary_condition = "natural"
        print(f"Warning: Config file {config_path} not found, using defaults")

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
    def __init__(self, config_path="configs/config.yaml"):
        """Initialize the policy interpolator."""
        self.config_path = config_path
        self._ndof = 0
        self._chunk_size = 0
        self._piecewise_curves = None
        self._optimized_curves = None
        self._velocity_constrained_curves = None
        self._adjusted_curves = None

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
                x_values, rearranged_positions[i], self.config_path
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

    def prepare_adjusted_trajectory(
        self, cur_joint_positions, joint_positions, ndof, chunk_size, max_velocities
    ):
        """
        Prepare adjusted spline curve representation for trajectory interpolation.

        Converts current joint position data into internal spline objects and
        smoothly connects them to the planned trajectory.

        Args:
            joint_positions: Flattened trajectory data in format [j0_t0, j1_t0, ..., j0_t1, j1_t1, ...]
            cur_joint_positions: Current joint positions
            ndof: Number of robot joints
            chunk_size: Number of waypoints per joint (trajectory length)
            max_velocities: Maximum allowed velocities for each joint
        """
        joint_positions = np.array(joint_positions)
        cur_joint_positions = np.array(cur_joint_positions)
        self._ndof = ndof
        self._chunk_size = chunk_size

        rearranged_positions = np.zeros((ndof, chunk_size))
        for i in range(ndof):
            rearranged_positions[i] = joint_positions[i::ndof]
        rearranged_positions[:, 0] = cur_joint_positions

        self._adjusted_curves = []

        for i in range(ndof):
            x_values = np.arange(chunk_size)

            def objective(y_control_points):
                cs_temp = CubicSpline(x_values, y_control_points, bc_type="clamped")
                y_velocity = cs_temp(x_values, 1)
                penalty_velocity = np.sum(
                    (np.clip(np.abs(y_velocity) - max_velocities[i], 0, None)) ** 2
                )
                penalty_path = np.sum(
                    (cs_temp(x_values) - rearranged_positions[i]) ** 2
                )
                penalty = penalty_velocity + penalty_path
                return penalty

            result = minimize(objective, rearranged_positions[i], method="L-BFGS-B")
            y_optimized = result.x
            cs_optimized = CubicSpline(x_values, y_optimized, bc_type="clamped")
            self._adjusted_curves.append(cs_optimized)

    def update_adjusted(self, _index):
        """
        Evaluate all adjusted spline curves at a specific trajectory index.

        Args:
            _index: Trajectory index to evaluate (0 to chunk_size-1)

        Returns:
            list: Joint positions at index _index, one value per joint
        """
        if self._adjusted_curves is None:
            raise ValueError(
                "No adjusted trajectory prepared. Call prepare_adjusted_trajectory() first."
            )

        return [cs(_index) for cs in self._adjusted_curves]

    def prepare_trajectory_with_optimization(
        self, joint_positions, ndof, chunk_size, max_velocities
    ):
        """
        Prepare piecewise curve representation for trajectory interpolation with optimization.

        Args:
            joint_positions: Flattened trajectory data in format [j0_t0, j1_t0, ..., j0_t1, j1_t1, ...]
            ndof: Number of robot joints
            chunk_size: Number of waypoints per joint (trajectory length)
            max_velocities: Array of maximum allowed velocities for each joint
        """
        joint_positions = np.array(joint_positions)
        self._ndof = ndof
        self._chunk_size = chunk_size

        rearranged_positions = np.zeros((ndof, chunk_size))
        for i in range(ndof):
            rearranged_positions[i] = joint_positions[i::ndof]

        self._optimized_curves = []

        for i in range(ndof):
            x_values = np.arange(chunk_size)

            def objective(y_control_points):
                cs_temp = CubicSpline(x_values, y_control_points, bc_type="clamped")
                y_velocity = cs_temp(x_values, 1)
                penalty_velocity = np.sum(
                    (np.clip(np.abs(y_velocity) - max_velocities[i], 0, None)) ** 2
                )
                penalty_path = np.sum(
                    (cs_temp(x_values) - rearranged_positions[i]) ** 2
                )
                penalty = penalty_velocity + penalty_path
                return penalty

            result = minimize(objective, rearranged_positions[i], method="L-BFGS-B")
            y_optimized = result.x
            cs_optimized = CubicSpline(x_values, y_optimized, bc_type="clamped")
            self._optimized_curves.append(cs_optimized)

    def update_with_optimization(self, _index):
        """
        Evaluate all optimized spline curves at a specific trajectory index.

        Args:
            _index: Trajectory index to evaluate (0 to chunk_size-1)

        Returns:
            list: Joint positions at index _index, one value per joint
        """
        if self._optimized_curves is None:
            raise ValueError(
                "No optimized trajectory prepared. Call prepare_trajectory() first."
            )

        return [cs(_index) for cs in self._optimized_curves]

    def prepare_trajectory_with_vel_constrain(
        self, joint_positions, ndof, chunk_size, max_velocities
    ):
        """
        Prepare piecewise curve representation for trajectory interpolation with velocity constraint.

        Args:
            joint_positions: Flattened trajectory data in format [j0_t0, j1_t0, ..., j0_t1, j1_t1, ...]
            ndof: Number of robot joints
            chunk_size: Number of waypoints per joint (trajectory length)
            max_velocities: Array of maximum allowed velocities for each joint
        """
        joint_positions = np.array(joint_positions)
        self._ndof = ndof
        self._chunk_size = chunk_size

        rearranged_positions = np.zeros((ndof, chunk_size))
        for i in range(ndof):
            rearranged_positions[i] = joint_positions[i::ndof]

        self._velocity_constrained_curves = []

        for i in range(ndof):
            x_values = np.arange(chunk_size)
            t = np.linspace(0, chunk_size, 100)
            cs_original = CubicSpline(
                x_values, rearranged_positions[i], bc_type="clamped"
            )
            y_spline_original = cs_original(t)
            y_velocity = cs_original(t, 1)

            y_adjusted = np.copy(y_spline_original)
            t_adjusted = np.copy(t)

            for j in range(1, len(t)):
                if np.abs(y_velocity[j]) > max_velocities[i]:
                    delta_y = y_spline_original[j] - y_adjusted[j - 1]
                    delta_t = np.abs(delta_y) / max_velocities[i]
                    t_adjusted[j] = t_adjusted[j - 1] + delta_t
                    y_adjusted[j] = (
                        y_adjusted[j - 1]
                        + np.sign(delta_y) * max_velocities[i] * delta_t
                    )
                else:
                    t_adjusted[j] = t_adjusted[j - 1] + (t[j] - t[j - 1])
                    y_adjusted[j] = y_spline_original[j]

                if t_adjusted[j] <= t_adjusted[j - 1]:
                    t_adjusted[j] = t_adjusted[j - 1] + 1e-5

            cs_constrained = CubicSpline(t_adjusted, y_adjusted, bc_type="clamped")
            self._velocity_constrained_curves.append(cs_constrained)

    def update_with_vel_constrain(self, _index):
        """
        Evaluate all velocity-constrained spline curves at a specific trajectory index.

        Args:
            _index: Trajectory index to evaluate (0 to chunk_size-1)

        Returns:
            list: Joint positions at index _index, one value per joint
        """
        if self._velocity_constrained_curves is None:
            raise ValueError(
                "No velocity-constrained trajectory prepared. Call prepare_trajectory_with_vel_constrain() first."
            )

        return [cs(_index) for cs in self._velocity_constrained_curves]

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
                    x_values, rearranged_positions[i], self.config_path
                )

                t_samples = np.linspace(0, chunk_size - 1, size)
                sample = [cs(t) for t in t_samples]

                for j in range(len(sample)):
                    index = j * ndof + i
                    sampled_data[0][index] = sample[j]

        sampled_data = tuple(sampled_data[0])

        return sampled_data
