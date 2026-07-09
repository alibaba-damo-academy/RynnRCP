"""Runtime: configuration-driven orchestrator for runners and server services."""

import asyncio
import logging
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from rynnrcp.ipc.channel import ChannelManager
from rynnrcp.config.resolver import resolve_refs
from rynnrcp.config.runtime_config import RuntimeConfig
from rynnrcp.config.runner_config import RunnerInputSpec, RunnerOutputSpec
from rynnrcp.process.node_launcher import NodeLauncher
from rynnrcp.runtime.runner_manager import (
    RunnerManager,
    setup_runner_manager_process,
)
from rynnrcp.services.base_service import BaseService
from rynnrcp.services.action_service import ActionService
from rynnrcp.services.collection_service import CollectionService
from rynnrcp.services.health_service import HealthService
from rynnrcp.services.manifest_service import ManifestService
from rynnrcp.services.observation_service import ObservationService
from rynnrcp.services.policy_service import PolicyService
from rynnrcp.services.resource_service import ResourceRegistry, ResourceService
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.utils.user_paths import logs_dir, resolve_robot_path, robot_root_from_config, tmp_dir

logger = logging.getLogger(__name__)


class Runtime:
    """Runtime orchestrator for runners, services, and protocol tools."""

    def __init__(self, config: RuntimeConfig):
        """Initialize Runtime from an already loaded RuntimeConfig."""
        if not isinstance(config, RuntimeConfig):
            raise TypeError("Runtime requires a RuntimeConfig")
        self._init_empty()
        self._init_from_config(config)

    @property
    def bus(self) -> ToolBus:
        """Access the ToolBus instance."""
        return self._bus

    @property
    def services(self) -> Dict[str, BaseService]:
        """Access all registered rynnrcp.services."""
        return dict(self._services)

    # ─── Tool API ─────────────────────────────────────────────────────

    def tool_list(self) -> Dict[str, Dict[str, Any]]:
        """Return all available tools exposed by Services.

        Returns:
            Dict mapping tool name to {description, input, output}.
        """
        return self._bus.list_tools()

    def tool_call(self, tool_name: str, *args, **kwargs) -> Any:
        """Invoke a registered tool by name.

        This is the primary entry point for AI Agent interaction.

        Args:
            tool_name: Name of the tool to call.
            *args, **kwargs: Arguments forwarded to the tool handler.

        Returns:
            Tool result (typically a dict with success/message/result).
        """
        return self._bus.call_tool(tool_name, *args, **kwargs)

    # ─── Lifecycle ───────────────────────────────────────────────────

    def stop(self) -> None:
        if self._runner_launcher is not None:
            logger.info("[RuntimeShutdown] step=1/4 stop runner processes")
            self._runner_launcher.shutdown()
            self._runner_launcher = None
        if self._runner_manager is not None and self._runner_mode == "thread":
            logger.info("[RuntimeShutdown] step=1/4 stop runner manager")
            self._runner_manager.stop()
        self._runner_manager = None

        logger.info("[RuntimeShutdown] step=2/4 unbind services count=%d", len(self._services))
        for name, svc in list(self._services.items()):
            try:
                logger.info("[RuntimeShutdown] unbind service '%s'", name)
                svc.unbind()
            except Exception as e:
                logger.error("Error unbinding service %s: %s", name, e, exc_info=True)
        self._services.clear()

        logger.info("[RuntimeShutdown] step=3/4 close channels")
        ChannelManager.reset()

        logger.info("[RuntimeShutdown] step=4/4 complete")
        logger.info("Runtime stopped")

    def start(self) -> None:
        """Start configured runners."""
        try:
            logger.info(
                "[RuntimeStartup] step=1/2 start runners mode=%s",
                self._runner_mode,
            )
            if self._runner_mode == "process" and self._runner_launcher is not None:
                self._runner_launcher.start()
            elif self._runner_manager is not None:
                self._runner_manager.start()

            logger.info("[RuntimeStartup] step=2/2 completed")
        except KeyboardInterrupt:
            logger.info("Runtime start interrupted; stopping runtime")
            self.stop()
            raise
        except asyncio.CancelledError as exc:
            logger.info("Runtime start cancelled; stopping runtime")
            self.stop()
            raise KeyboardInterrupt from exc
        except Exception as exc:
            logger.exception("Runtime start failed; stopping runtime: %s", exc)
            self.stop()
            raise

    # ─── Private ──────────────────────────────────────────────────────

    def _init_empty(self) -> None:
        self._bus = ToolBus()
        self._services: Dict[str, BaseService] = {}
        self._config: Dict[str, Any] = {}
        self._runner_manager: Optional[RunnerManager] = None
        self._runner_launcher: Optional[NodeLauncher] = None
        self._runtime_config: Optional[RuntimeConfig] = None
        self._runner_mode = "thread"
        self._channel_registry_name = "rynnrcp_channel_registry"

    def _init_from_config(self, config: RuntimeConfig) -> None:
        """Initialize Runtime in the standard order.

        Flow:
        load source configs -> build runner -> build runner services.
        start/stop only handle lifecycle after initialization.
        """
        self._runtime_config = config
        self._config = config.runtime_context
        self._runner_mode = config.runner_mode
        if self._runner_mode == "process":
            self._prepare_process_mode(config)
        runner_manager = self._build_runner(config=config)
        self._build_runner_services(runner_manager, config=config)

        logger.info(
            "Runtime initialized with %d services, %d tools",
            len(self._services),
            len(self.tool_list()),
        )

    def _prepare_process_mode(self, config: RuntimeConfig) -> None:
        self._channel_registry_name = config.channel_registry_name
        self._runner_launcher = NodeLauncher(
            config=self._config,
            startup_timeout_s=config.startup_timeout_s,
            registry_name=self._channel_registry_name,
        )
        ChannelManager.instance().attach_registry(self._runner_launcher.prepare_registry())
        logger.info(
            "Runtime process channel registry: name=%s robot_id=%s",
            self._channel_registry_name,
            config.robot_id,
        )

    def _build_runner(
        self,
        *,
        config: RuntimeConfig,
    ) -> RunnerManager:
        """Build runner manager directly from source configs."""
        runner_manager = RunnerManager(
            config=config,
        )
        self._runner_manager = runner_manager
        if self._runner_mode == "process":
            self._register_runner_processes(
                config=config,
                runner_names=runner_manager.runner_names,
            )
        logger.info(
            "RunnerManager built with %d inputs and %d outputs (mode=%s)",
            len(runner_manager.input_specs),
            len(runner_manager.output_specs),
            self._runner_mode,
        )
        _log_runner_specs(runner_manager)
        return runner_manager

    def _register_runner_processes(
        self,
        *,
        config: RuntimeConfig,
        runner_names: list[str],
    ) -> None:
        if self._runner_launcher is None:
            raise RuntimeError("Runner process launcher is not initialized")
        for runner_name in runner_names:
            node_config = {
                "runtime_config": config,
                "runner_filter": [runner_name],
                "channel_registry_name": self._channel_registry_name,
            }
            self._runner_launcher.add_node(
                runner_name,
                setup_runner_manager_process,
                node_config=node_config,
            )

    def _build_runner_services(
        self,
        runner_manager: RunnerManager,
        *,
        config: RuntimeConfig,
    ) -> None:
        """Bind standard services for runner inputs and outputs."""
        input_specs = list(runner_manager.input_specs)
        health_specs = list(getattr(runner_manager, "health_specs", []))
        output_specs = list(runner_manager.output_specs)
        capabilities = _server_capabilities(config.runtime_context)
        robot_root_dir = robot_root_from_config(config.runtime_context)
        resource_registry = ResourceRegistry(str(tmp_dir(robot_root_dir) / "resources"))
        resource_registry.add_catalog_root(
            "collections",
            config.collection_root_dir,
            domain="data",
            name="collections",
            metadata={"kind": "collections_root"},
        )
        resource_registry.add_catalog_root(
            "logs",
            str(logs_dir(robot_root_dir)),
            domain="log",
            name="logs",
            metadata={"kind": "logs_root"},
        )
        model_refs = _register_model_refs(
            resource_registry,
            config.runtime_context,
            robot_root_dir=robot_root_dir,
        )

        self._bind_service(
            "manifest_service",
            ManifestService(
                self._bus,
                config=self._config,
                robot_id=config.robot_id,
                inputs=input_specs,
                outputs=output_specs,
                model_refs=model_refs,
            ),
        )

        if capabilities.get("observations", False) and input_specs:
            self._bind_service("observation_service", ObservationService(self._bus, input_specs))

        if capabilities.get("actions", False) and output_specs:
            self._bind_service("action_service", ActionService(self._bus, output_specs))

        if capabilities.get("policy_service", False) and input_specs and output_specs:
            self._bind_service(
                "policy_service",
                PolicyService(
                    self._bus,
                    config=config.runtime_context,
                    robot_root_dir=robot_root_dir,
                ),
            )

        if capabilities.get("resources", False):
            self._bind_service("resource_service", ResourceService(self._bus, resource_registry))

        if capabilities.get("data_collection", False) and (input_specs or output_specs):
            self._bind_service(
                "collection_service",
                CollectionService(
                    self._bus,
                    _collection_inputs(input_specs, output_specs),
                    root_dir=config.collection_root_dir,
                    resources=resource_registry,
                ),
            )

        if capabilities.get("health", False):
            self._bind_service(
                "health_service",
                HealthService(
                    self._bus,
                    inputs=input_specs,
                    healths=health_specs,
                ),
            )

        logger.info(
            "Runner services built: %s",
            sorted(self._services.keys()),
        )

    def _bind_service(self, name: str, service: BaseService) -> None:
        service.bind()
        self._services[name] = service

def _collection_inputs(
    input_specs: list[RunnerInputSpec],
    output_specs: list[RunnerOutputSpec],
) -> list[RunnerInputSpec]:
    collection_inputs = list(input_specs)
    action_channels: set[str] = set()
    for output in output_specs:
        if output.channel in action_channels:
            continue
        action_channels.add(output.channel)
        collection_inputs.append(
            RunnerInputSpec(
                name=f"{output.name}_action",
                runner_name="action_service",
                protocol="internal",
                adapter="ActionChannel",
                channel=output.channel,
                msg_size=output.msg_size,
                object_name=str(output.params["rcp_action_name"]),
                channel_transport=output.channel_transport,
                payload_type="json",
                info={
                    "rcp_action_name": str(output.params["rcp_action_name"]),
                    "rcp_action_type": str(output.params["rcp_action_type"]),
                    "component_name": str(output.params["component_name"]),
                    "description": output.params.get("description"),
                    "frame_rate": output.params.get("frame_rate"),
                },
            )
        )
    return collection_inputs


def _server_capabilities(config: Mapping[str, Any]) -> dict[str, bool]:
    manifest = config.get("manifest") if isinstance(config, Mapping) else {}
    capabilities = manifest.get("capabilities") if isinstance(manifest, Mapping) else {}
    return {str(key): bool(value) for key, value in dict(capabilities or {}).items()}


def _register_model_refs(
    registry: ResourceRegistry,
    config: Mapping[str, Any],
    *,
    robot_root_dir: Path,
) -> dict[str, Any]:
    manifest = config.get("manifest") if isinstance(config, Mapping) else {}
    raw_refs = manifest.get("model_refs") if isinstance(manifest, Mapping) else None
    if not isinstance(raw_refs, Mapping):
        return {}

    try:
        refs = resolve_refs(raw_refs, config)
    except KeyError as exc:
        raise ValueError("manifest.model_refs cannot be resolved") from exc
    if not _server_capabilities(config).get("resources", False):
        return {}

    model_refs: dict[str, Any] = {}
    for key in ("urdf", "mjcf", "calibration"):
        if key not in refs:
            continue
        value = refs[key]
        if value in (None, ""):
            model_refs[key] = None
        elif isinstance(value, str):
            model_refs[key] = _register_model_ref_file(
                registry,
                value,
                key=key,
                robot_root_dir=robot_root_dir,
            )
        elif isinstance(value, Mapping):
            model_refs[key] = dict(value)
        else:
            raise TypeError(f"manifest.model_refs.{key} must be a path string, object, or null")

    assets = refs.get("assets")
    if assets is not None:
        if not isinstance(assets, list):
            raise TypeError("manifest.model_refs.assets must be a list")
        model_refs["assets"] = [
            _register_model_asset(registry, item, index=index, robot_root_dir=robot_root_dir)
            for index, item in enumerate(assets)
        ]
    return model_refs


def _register_model_ref_file(
    registry: ResourceRegistry,
    path: str,
    *,
    key: str,
    robot_root_dir: Path,
) -> dict[str, Any]:
    resource_path = resolve_robot_path(path, robot_root_dir)
    resource = registry.register_path(
        str(resource_path),
        resource_type="file",
        domain="calibration" if key == "calibration" else "model",
        name=resource_path.name,
        format=_model_ref_format(key, resource_path),
        mode="snapshot",
        metadata={"kind": "model_ref", "model_ref": key},
    )
    return {
        field: resource[field]
        for field in ("format", "resource_id", "hash", "size_bytes")
        if field in resource
    }


def _register_model_asset(
    registry: ResourceRegistry,
    value: Any,
    *,
    index: int,
    robot_root_dir: Path,
) -> dict[str, Any]:
    if isinstance(value, str):
        resource_path = resolve_robot_path(value, robot_root_dir)
        return registry.register_path(
            str(resource_path),
            resource_type="file",
            domain="model",
            name=resource_path.name,
            mode="snapshot",
            metadata={"kind": "model_asset", "model_ref": "assets", "asset_index": index},
        )
    if isinstance(value, Mapping):
        return dict(value)
    raise TypeError(f"manifest.model_refs.assets[{index}] must be a path string or Resource object")


def _model_ref_format(key: str, path: Path) -> str:
    if key in {"urdf", "mjcf"}:
        return key
    suffix = path.suffix.lstrip(".")
    return suffix or key


def _log_runner_specs(runner_manager: RunnerManager) -> None:
    for spec in runner_manager.input_specs:
        logger.info(
            "Runner input configured: runner=%s name=%s protocol=%s object=%s channel=%s transport=%s",
            spec.runner_name,
            spec.name,
            spec.protocol,
            spec.object_name,
            spec.channel,
            spec.channel_transport,
        )
    for spec in runner_manager.output_specs:
        logger.info(
            "Runner output configured: runner=%s name=%s protocol=%s action=%s channel=%s transport=%s",
            spec.runner_name,
            spec.name,
            spec.protocol,
            spec.params.get("rcp_action_name"),
            spec.channel,
            spec.channel_transport,
        )
