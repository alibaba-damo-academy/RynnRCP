from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.services.policy_service import PolicyService


def test_policy_service_runs_policy_and_updates_inputs(tmp_path: Path) -> None:
    _write_policy(tmp_path)
    actions: list[dict[str, Any]] = []
    bus = ToolBus()
    bus.add_tool(
        "list_observations",
        lambda: ToolBus.make_result(
            True,
            {"observations": [{"name": "observation.robot.joint_state"}]},
        ),
    )
    bus.add_tool(
        "get_observations",
        lambda names: ToolBus.make_result(
            True,
            {
                "observations": [
                    {
                        "name": names[0],
                        "timestamp": time.time(),
                        "value": {"joint_positions": [1.0, 2.0, 3.0]},
                    }
                ]
            },
        ),
    )
    bus.add_tool(
        "list_actions",
        lambda: ToolBus.make_result(
            True,
            {"actions": [{"name": "action.robot.joint_position"}]},
        ),
    )
    bus.add_tool(
        "run_action_chunk",
        lambda name, frames, frame_rate: actions.append(
            {"name": name, "frames": frames, "frame_rate": frame_rate}
        )
        or ToolBus.make_result(True, {"accepted_frames": len(frames)}),
    )
    bus.add_tool("stop_action", lambda reason=None: ToolBus.make_result(True, {"stopped": True}))

    service = PolicyService(bus, config={"policies": {"paths": ["policies"]}}, robot_root_dir=tmp_path)
    service.bind()
    try:
        listed = service.list_policies()
        assert listed["result"]["policies"][0]["policy_id"] == "echo"

        bad_start = service.start_policy("echo", runtime_inputs={"unknown": 1})
        assert bad_start["success"] is False
        assert "Unknown runtime input" in bad_start["message"]

        started = service.start_policy("echo")
        assert started["success"] is True
        _wait_for(lambda: actions)
        assert actions[-1]["frames"][0]["joint_positions"] == [0.1, 2.0, 3.0]

        updated = service.update_policy_inputs(runtime_inputs={"target": 0.5})
        assert updated["success"] is True
        _wait_for(lambda: actions[-1]["frames"][0]["joint_positions"][0] == 0.5)
    finally:
        service.unbind()


def _write_policy(root: Path) -> None:
    policy_dir = root / "policies" / "echo"
    policy_dir.mkdir(parents=True)
    (policy_dir / "policy.yaml").write_text(
        """
policy_id: echo
name: Echo
entrypoint: policy.py:Policy
inputs:
  observations:
    - component: robot
      name: joint_state
  runtime_inputs:
    target:
      type: float
      default: 0.1
outputs:
  actions:
    - component: robot
      name: joint_position
""".strip(),
        encoding="utf-8",
    )
    (policy_dir / "policy.py").write_text(
        """
class Policy:
    def reset(self, runtime_inputs):
        self.runtime_inputs = dict(runtime_inputs)

    def step(self, obs):
        state = list(obs["observation.robot.joint_state"]["joint_positions"])
        state[0] = float(obs["target"])
        return {
            "name": "action.robot.joint_position",
            "frame_rate": 50,
            "frames": [{"joint_positions": state}],
        }
""".strip(),
        encoding="utf-8",
    )


def _wait_for(predicate, timeout_s: float = 2.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition was not met before timeout")
