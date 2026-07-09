"""
Protocol action output adapter.

Converts one protocol action step into connector messages.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Tuple

from .base_output_adapter import BaseOutputAdapter
from rynnrcp.protocol.action_codecs import protocol_action_value
from rynnrcp.utils.imports import get_message_class


class ProtocolActionOutputAdapter(BaseOutputAdapter):
    """Step-mode output: one message per step per output.

    Configuration: each output dict is the connector params.
    """

    def build_step_output(
        self,
        step_output: Any,
        outputs: List[Dict[str, Any]],
        fps: float,
    ) -> List[List[Tuple[str, Dict, Any, float, int]]]:
        """Build one frame group for the current action step.

        Returns:
            ``[[(...), ...]]`` where the inner list = outputs for this step.
        """
        interval = 1.0 / fps if fps > 0 else 0.0
        frame_group: List[Tuple[str, Dict, Any, float, int]] = []

        for params in outputs:
            action_name = str(params["rcp_action_name"])
            action_type = str(params["rcp_action_type"])
            step_value = step_output[action_name] if isinstance(step_output, dict) else step_output

            connector = str(params["connector"])
            if connector == "module":
                frame_group.append((connector, params, {action_name: step_value}, interval, 1))
                continue

            if params.get("payload_mode") == "protocol_json":
                protocol_value = protocol_action_value(action_type, step_value, action_name)
                payload = {
                    "name": action_name,
                    "type": action_type,
                    "value": protocol_value,
                }
                if connector == "ros2":
                    msg = _new_message(params)
                    if not hasattr(msg, "data"):
                        raise TypeError("ROS2 protocol_json outputs must use a message with a data field")
                    msg.data = json.dumps(payload, ensure_ascii=False)
                else:
                    msg = payload
                frame_group.append(
                    (
                        connector,
                        params,
                        msg,
                        interval,
                        1,
                    )
                )
                continue

            mappings = params.get("mappings")
            if not mappings:
                raise ValueError(f"{connector} output requires mappings when payload_mode is not protocol_json")

            msg: Any = _new_message(params)
            for m in mappings:
                name = str(m["name"])
                if name != action_name:
                    raise ValueError(f"output mapping name {name} does not match action {action_name}")
                out_field = str(m.get("field") or m["out_field"])
                source_path = str(m.get("from") or "")
                _assign_path(msg, out_field, _extract_path(step_value, source_path))
            frame_group.append((connector, params, msg, interval, 1))

        return [frame_group] if frame_group else []


def _new_message(params: Dict[str, Any]) -> Any:
    return get_message_class(str(params["msg_type"]))()


def _extract_path(value: Any, path: str) -> Any:
    if not path:
        return value
    cur = value
    for part in path.split("."):
        if isinstance(cur, dict):
            cur = cur[part]
        elif isinstance(cur, (list, tuple)) and part.isdigit():
            cur = cur[int(part)]
        else:
            cur = getattr(cur, part)
    return cur


def _assign_path(target: Any, path: str, value: Any) -> None:
    parts = path.split(".")
    cur = target
    for part in parts[:-1]:
        if isinstance(cur, dict):
            cur = cur.setdefault(part, {})
        else:
            cur = getattr(cur, part)
    last = parts[-1]
    if isinstance(cur, dict):
        cur[last] = value
    else:
        setattr(cur, last, value)
