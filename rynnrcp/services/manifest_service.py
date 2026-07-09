"""ManifestService: expose the RCP robot manifest."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Dict, Iterable

from .action_service import _build_action_descriptors, _public_action_descriptor
from .base_service import BaseService
from .observation_service import _build_observation_descriptors
from rynnrcp.config.runner_config import RunnerInputSpec, RunnerOutputSpec
from rynnrcp.protocol.methods import GET_MANIFEST
from rynnrcp.runtime.tool_bus import ToolBus


class ManifestService(BaseService):
    """Generate the protocol manifest from resolved runtime configuration."""

    def __init__(
        self,
        bus: ToolBus,
        *,
        config: Mapping[str, Any],
        robot_id: str,
        inputs: Iterable[RunnerInputSpec],
        outputs: Iterable[RunnerOutputSpec],
        model_refs: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(bus, "manifest_service")
        self._config = config
        self._robot_id = robot_id
        self._inputs = list(inputs)
        self._outputs = list(outputs)
        self._model_refs = dict(model_refs or {})

    def bind(self) -> None:
        self._register_tool(
            GET_MANIFEST.name,
            self.get_manifest,
            input_schema=GET_MANIFEST.input_schema,
            output_schema=GET_MANIFEST.output_schema,
            description=GET_MANIFEST.description,
        )

    def get_manifest(self) -> Dict[str, Any]:
        observations = [_manifest_object(item) for item in _build_observation_descriptors(self._inputs)]
        actions = [_manifest_object(_public_action_descriptor(item)) for item in _build_action_descriptors(self._outputs)]
        return {
            "robot_id": self._robot_id,
            "robot_name": str(_manifest_config(self._config)["robot_name"]),
            "embodiment_type": str(_manifest_config(self._config)["embodiment_type"]),
            "components": _components(_manifest_config(self._config)),
            "observations": observations,
            "actions": actions,
            "capabilities": dict(_manifest_config(self._config).get("capabilities") or {}),
            "model_refs": dict(self._model_refs),
            "metadata": dict(_manifest_config(self._config).get("metadata") or {}),
        }


def _manifest_object(item: Mapping[str, Any]) -> Dict[str, Any]:
    summary = {
        "name": item["name"],
        "type": item["type"],
    }
    for key in ("component_name", "description"):
        if item.get(key):
            summary[key] = item[key]
    return summary


def _components(manifest: Mapping[str, Any]) -> list[Dict[str, Any]]:
    configured = manifest.get("components")
    if isinstance(configured, list):
        return [dict(item) for item in configured if isinstance(item, Mapping)]
    return []


def _server_config(config: Mapping[str, Any]) -> Mapping[str, Any]:
    server = config.get("server")
    return server if isinstance(server, Mapping) else {}


def _manifest_config(config: Mapping[str, Any]) -> Mapping[str, Any]:
    manifest = config.get("manifest")
    return manifest if isinstance(manifest, Mapping) else {}
