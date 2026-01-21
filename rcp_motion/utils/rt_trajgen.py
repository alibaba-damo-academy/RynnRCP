import numpy as np

import ruckig


class RealtimeTrajGenRuckig:
    def __init__(
        self, dof: int = 6, input_freq: float = 30.0, output_freq: float = 200.0
    ):
        """Initialize the trajectory generation base class."""
        self._ndof = dof
        self._input_freq = input_freq
        self._output_freq = output_freq
        self._input_dt = 1.0 / self._input_freq
        self._output_dt = 1.0 / self._output_freq
        self._chunk_size = 0

        self._q_plan = np.zeros(self._ndof)
        self._qd_plan = np.zeros(self._ndof)
        self._qdd_plan = np.zeros(self._ndof)
        self._q_plan_last = np.zeros(self._ndof)
        self._qd_plan_last = np.zeros(self._ndof)
        self._qdd_plan_last = np.zeros(self._ndof)

        self._q_target = np.zeros(self._ndof)
        self._qd_target = np.zeros(self._ndof)
        self._qdd_target = np.zeros(self._ndof)
        self._q_target_last = np.zeros(self._ndof)
        self._qd_target_last = np.zeros(self._ndof)
        self._qdd_target_last = np.zeros(self._ndof)

        self._ruckig_otg = ruckig.Ruckig(self._ndof, self._output_dt)
        self._ruckig_input = ruckig.InputParameter(self._ndof)
        self._ruckig_output = ruckig.OutputParameter(self._ndof)

        self._set_default_limits()

    def set_position_limits(self, position_upper_limits, position_lower_limits):
        """Set position limits for trajectory generation."""
        self._position_upper_limits = position_upper_limits
        self._position_lower_limits = position_lower_limits

    def set_robot_ability(
        self, velocity_limits, acceleration_limits=None, jerk_limits=None
    ):
        """Set robot ability for trajectory generation."""
        self._velocity_upper_limits = velocity_limits
        self._velocity_lower_limits = np.negative(velocity_limits)

        if acceleration_limits is not None:
            self._acceleration_upper_limits = acceleration_limits
            self._acceleration_lower_limits = np.negative(acceleration_limits)
        else:
            self._acceleration_upper_limits = np.multiply(velocity_limits, 10)
            self._acceleration_lower_limits = np.multiply(velocity_limits, -10)

        if jerk_limits is not None:
            self._jerk_limits = jerk_limits
        else:
            self._jerk_limits = np.multiply(velocity_limits, 1000)

        # print(
        #    "limit velocity:", self._velocity_upper_limits, self._velocity_lower_limits
        # )
        # print(
        #    "limit acceleration:",
        #    self._acceleration_upper_limits,
        #    self._acceleration_lower_limits,
        # )

        self._set_ruckig_limits()

    def reset_state(self, q_state, q_target, qd_state=None, qd_target=None):
        """Reset state for trajectory generation."""
        self._q_plan = q_state
        self._q_target = q_target

        if qd_state is None:
            self._qd_plan = np.zeros(self._ndof)
        else:
            self._qd_plan = qd_state

        if qd_target is None:
            self._qd_target = np.zeros(self._ndof)
        else:
            self._qd_target = qd_target

        self._qdd_plan = np.zeros(self._ndof)
        self._qdd_target = np.zeros(self._ndof)
        self._update_state_data()

        # print("q_state:", self._q_plan)
        # print("q_target:", self._q_target)

    def set_input_target(self, q_target, qd_target=None):
        """Set input for trajectory generation."""
        self._q_target = q_target
        if qd_target is None:
            self._qd_target = (self._q_target - self._q_target_last) * self._input_freq
        else:
            self._qd_target = qd_target
        self._qdd_target = (self._qd_target - self._qd_target_last) * self._input_freq
        # self._qdd_target = np.zeros(self._ndof)

    def update(self):
        """
        update trajectory and output positions command
        Returns:
            list: positions command
        """
        self._input_target_limit()
        self._ruckig_rt_traj_generation()
        self._update_state_data()
        return np.array(self._q_plan)

    def _update_state_data(self):
        """Initialize state for trajectory generation."""
        self._q_target_last = self._q_target
        self._qd_target_last = self._qd_target
        self._qdd_target_last = self._qdd_target
        self._q_plan_last = self._q_plan
        self._qd_plan_last = self._qd_plan
        self._qdd_plan_last = self._qdd_plan

    def _set_ruckig_limits(self):
        """Set ruckig limits"""
        self._ruckig_input.max_velocity = self._velocity_upper_limits
        self._ruckig_input.max_acceleration = self._acceleration_upper_limits
        self._ruckig_input.max_jerk = self._jerk_limits

    def _set_default_limits(self):
        """Set default limits for trajectory generation."""
        self._position_upper_limits = np.array([np.pi] * self._ndof)
        self._position_lower_limits = np.array([-np.pi] * self._ndof)
        self._velocity_upper_limits = np.array([np.pi] * self._ndof)
        self._velocity_lower_limits = np.array([-np.pi] * self._ndof)
        self._acceleration_upper_limits = np.multiply(self._velocity_upper_limits, 10)
        self._acceleration_lower_limits = np.multiply(self._velocity_upper_limits, -10)
        self._jerk_limits = np.multiply(self._velocity_upper_limits, 1000)
        self._set_ruckig_limits()

    def _input_target_limit(self):
        self._q_target = np.clip(
            self._q_target, self._position_lower_limits, self._position_upper_limits
        )
        self._qd_target = np.clip(
            self._qd_target, self._velocity_lower_limits, self._velocity_upper_limits
        )
        self._qdd_target = np.clip(
            self._qdd_target,
            self._acceleration_lower_limits,
            self._acceleration_upper_limits,
        )

    def _ruckig_rt_traj_generation(self):
        """Ruckig realtime trajectory generation."""
        self._ruckig_input.current_position = self._q_plan_last
        self._ruckig_input.current_velocity = self._qd_plan_last
        self._ruckig_input.current_acceleration = self._qdd_plan_last
        self._ruckig_input.target_position = self._q_target
        self._ruckig_input.target_velocity = self._qd_target
        self._ruckig_input.target_acceleration = self._qdd_target
        # print("ruckig input:", self._ruckig_input)

        result = self._ruckig_otg.update(self._ruckig_input, self._ruckig_output)
        if result == ruckig.Result.Working:
            # qd_plan and qdd_plan is scaled by 0.999, aviodance of numerical error
            self._q_plan = self._ruckig_output.new_position
            self._qd_plan = np.multiply(self._ruckig_output.new_velocity, 0.99)
            self._qdd_plan = np.multiply(self._ruckig_output.new_acceleration, 0.99)
            # self._ruckig_output.pass_to_input(self._ruckig_input)
        else:
            self._q_plan = self._q_plan_last
            self._qd_plan = self._qd_plan_last
            self._qdd_plan = self._qdd_plan_last
            # print("ruckig trajectory generation ", result)
