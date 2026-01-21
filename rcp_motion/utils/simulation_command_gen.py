"""
simulation Command Generator

generation simulation command
-- generate simulation trajectory with chunk
-- generate simulation periodic trajectory command

"""

import time
import threading
import numpy as np


class SimCommandGenerator:
    """
    generate simulation trajectory.
    """

    def __init__(self, dof=7, chunk_size=20, update_freq=100, command_freq=30):
        """
        generate simulation trajectory.

        Args:
            type: periodic or chunk
        """
        if command_freq < 0.0001:
            raise ValueError("command_freq must be greater than 0.0001")

        self.chunk_size = chunk_size
        self.command_freq = command_freq
        self.command_dt = 1.0 / self.command_freq
        self.update_freq = update_freq
        self.update_dt = 1.0 / self.update_freq
        self.signal_genloop_time = 0
        self.gen_command_time = 0
        self.seq = -1

        self.dof = dof
        self.signal_time = 0
        self.home_position = None
        self.signal_amplitude = None
        self.signal_frequency = None
        self.signal_type = None
        self.random_seed = None
        self.random_amplitude = 0.01
        self.trajectory = np.zeros((self.chunk_size, self.dof))

    def set_traj_parameter(
        self,
        home_position,
        signal_amplitude,
        signal_frequency,
        block_time=0.0,
        signal_type="sin",
        random_seed=5,
        random_ratio=0.1,
    ):
        """set trajectory parameter."""
        if len(home_position) != self.dof:
            raise ValueError("home_position must be match dof")

        if signal_type not in ["sin", "cos", "triangle"]:
            raise ValueError("signal_type must be sin, cos, triangle")

        if block_time < 0:
            raise ValueError("block_time must be greater than 0")

        if random_ratio < 0 or random_ratio > 1:
            raise ValueError("random_ratio must be between 0 and 1")

        self.signal_type = signal_type
        self.home_position = home_position
        self.signal_amplitude = signal_amplitude
        self.signal_frequency = signal_frequency
        self.gen_command_time = block_time + self.chunk_size * self.command_dt

        if random_seed is not None:
            np.random.seed(random_seed)
            self.random_seed = random_seed
            self.random_amplitude = self.signal_amplitude * random_ratio

    def signal_update(self):
        """update signal, update every loop and it will count in inner loop."""
        if self.signal_genloop_time >= self.gen_command_time:
            self.signal_genloop_time = 0
        if self.signal_genloop_time == 0:
            self.generate_commands()
            self.seq = self.seq + 1
        self.signal_genloop_time = self.signal_genloop_time + self.update_dt
        return self.trajectory, self.seq

    def generate_commands(self):
        """generate simulation trajectory."""
        for i in range(self.chunk_size):
            self.signal_time = self.signal_time + self.command_dt
            if self.signal_type == "sin":
                self.trajectory[i, :] = self.generate_sine_commands(self.signal_time)
            elif self.signal_type == "cos":
                self.trajectory[i, :] = self.generate_cosine_commands(self.signal_time)
            elif self.signal_type == "triangle":
                self.trajectory[i, :] = self.generate_triangle_commands(
                    self.signal_time
                )
        return self.trajectory

    def generate_sine_commands(self, signal_t):
        """generate simulation sine trajectory command."""
        trajectory_point = np.zeros(self.dof)
        for i in range(self.dof):
            trajectory_point[i] = self.home_position[
                i
            ] + self.signal_amplitude * np.sin(
                2 * np.pi * self.signal_frequency * signal_t
            )
            if self.random_seed is not None:
                trajectory_point[i] += np.random.uniform(
                    -self.random_amplitude, self.random_amplitude
                )
        return trajectory_point

    def generate_cosine_commands(self, signal_t):
        """generate simulation cos trajectory command."""
        trajectory_point = np.zeros(self.dof)
        for i in range(self.dof):
            trajectory_point[i] = self.home_position[
                i
            ] + self.signal_amplitude * np.cos(
                2 * np.pi * self.signal_frequency * signal_t
            )
            if self.random_seed is not None:
                trajectory_point[i] += np.random.uniform(
                    -self.random_amplitude, self.random_amplitude
                )
        return trajectory_point

    def generate_triangle_commands(self, signal_t):
        """generate simulation triangle trajectory command."""
        trajectory_point = np.zeros(self.dof)
        for i in range(self.dof):
            trajectory_point[i] = self.home_position[i] + self.signal_amplitude * (
                1 - 2 * np.abs(signal_t - np.floor(signal_t + 0.5))
            )
            if self.random_seed is not None:
                trajectory_point[i] += np.random.uniform(
                    -self.random_amplitude, self.random_amplitude
                )
        return trajectory_point
