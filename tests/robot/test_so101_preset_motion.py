from __future__ import annotations

import pytest

from rynnrcp_robot_so101.preset_motion import preset_motion_frame


def test_preset_motion_frames_are_six_axis_and_gripper_is_normalized() -> None:
    initial = [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

    for motion in range(1, 6):
        frame = preset_motion_frame(motion, 3.0, initial)

        assert len(frame) == 6
        assert 0.0 <= frame[-1] <= 1.0


def test_preset_motion_rejects_unknown_motion_id() -> None:
    with pytest.raises(ValueError, match="preset_motion.motion"):
        preset_motion_frame(99, 1.0, [0.0] * 6)
