# rcp_core/common/config_manager/validator.py

from __future__ import annotations

from typing import Any, Dict, List, Tuple

from rcp_core.common.utils.logger import server_logger
from ..adapter import INPUT_ADAPTER_REGISTRY, ADAPTER_REGISTRY
from ..utils.get_message_class import get_message_class

logger = server_logger()


class ConfigValidator:
    """
    Strict validator for robot_config YAML.

    Top-level expected:
      config_type: robot_config (optional, default robot_config)
      robot_type:  str           (required)
      servers:     list[server]  (required)

    server expected:
      name: str (required)
      inputs:  list[input]  (optional, default [])
      outputs: list[output] (optional, default [])
      compose: list[...]    (optional, default [])

    input expected (common):
      protocol: str (required)   # module/port/ros2/lcm
      adapter:  str (required)   # must exist in INPUT_ADAPTER_REGISTRY
      params:   dict (required)  # protocol-specific

    output expected (common):
      protocol: str (required)   # module/ros2/lcm
      adapter:  str (required)   # must exist in ADAPTER_REGISTRY
      + protocol-specific fields
    """

    # -------------------------
    # helpers
    # -------------------------
    def _require(
        self,
        obj: Dict[str, Any],
        key: str,
        where: str,
        typ: Any | Tuple[Any, ...] | None = None,
    ) -> Any:
        if key not in obj or obj[key] is None:
            raise ValueError(f"[config error] missing required '{key}' at {where}")
        val = obj[key]
        if typ is not None and not isinstance(val, typ):
            raise ValueError(
                f"[config error] '{key}' type invalid at {where}: expected {typ}, got {type(val)}"
            )
        return val

    def _optional(
        self,
        obj: Dict[str, Any],
        key: str,
        where: str,
        default_desc: str,
        typ: Any | Tuple[Any, ...] | None = None,
    ) -> Any:
        if key not in obj or obj[key] is None:
            logger.warning(
                f"[config warn] optional '{key}' not set at {where}, default={default_desc}"
            )
            return None
        val = obj[key]
        if typ is not None and not isinstance(val, typ):
            raise ValueError(
                f"[config error] optional '{key}' type invalid at {where}: expected {typ}, got {type(val)}"
            )
        return val

    def _validate_message_class(self, type_str: str, where: str) -> None:
        try:
            get_message_class(type_str)
        except (ImportError, AttributeError, ModuleNotFoundError) as e:
            raise ImportError(
                f"[config error] msg type '{type_str}' cannot import at {where}: {e}"
            )

    def _is_image_input_adapter(self, adapter_name: str) -> bool:
        # covers: Ros2ImageInputAdapter / LcmImageInputAdapter / ModuleImageInputAdapter / PortImageInputAdapter
        a = adapter_name.lower()
        return "image" in a and "generic" not in a

    # -------------------------
    # validate inputs
    # -------------------------
    def _validate_input(self, inp: Dict[str, Any], where: str) -> None:
        protocol = str(self._require(inp, "protocol", where, str)).lower()
        adapter_name = str(self._require(inp, "adapter", where, str))
        params = self._require(inp, "params", where, dict)

        # adapter must exist
        if adapter_name not in INPUT_ADAPTER_REGISTRY:
            raise ValueError(
                f"[config error] unknown input adapter '{adapter_name}' at {where}. "
                f"available={list(INPUT_ADAPTER_REGISTRY.keys())}"
            )

        # -------- protocol: ros2/lcm --------
        if protocol in ("ros2", "lcm"):
            topic = self._require(params, "topic", f"{where}.params", str)
            msg_type = self._require(params, "msg_type", f"{where}.params", str)
            self._validate_message_class(msg_type, f"{where}.params.msg_type")

            mappings = self._require(params, "mappings", f"{where}.params", list)
            if not mappings:
                raise ValueError(f"[config error] empty mappings at {where}.params")

            # image adapters: mappings[0].out_key required
            if self._is_image_input_adapter(adapter_name):
                m0 = mappings[0]
                if (
                    not isinstance(m0, dict)
                    or not isinstance(m0.get("out_key"), str)
                    or not m0["out_key"]
                ):
                    raise ValueError(
                        f"[config error] {adapter_name} requires mappings[0].out_key at {where}.params"
                    )
            else:
                # generic adapters: every mapping needs field + out_key
                for i, m in enumerate(mappings):
                    if not isinstance(m, dict):
                        raise ValueError(
                            f"[config error] mappings[{i}] must be dict at {where}.params"
                        )
                    if not isinstance(m.get("field"), str) or not m["field"]:
                        raise ValueError(
                            f"[config error] {adapter_name} requires mappings[{i}].field at {where}.params"
                        )
                    if not isinstance(m.get("out_key"), str) or not m["out_key"]:
                        raise ValueError(
                            f"[config error] {adapter_name} requires mappings[{i}].out_key at {where}.params"
                        )

        # -------- protocol: module --------
        elif protocol == "module":
            # Required
            self._require(params, "module_name", f"{where}.params", str)
            self._require(params, "out_key", f"{where}.params", str)

            # Optional (warn if absent)
            self._optional(
                params,
                "sys_path",
                f"{where}.params",
                default_desc="[]",
                typ=(list, str),
            )
            self._optional(
                params, "init_args", f"{where}.params", default_desc="{}", typ=dict
            )
            self._optional(
                params,
                "start_call",
                f"{where}.params",
                default_desc="auto start() if exists",
                typ=list,
            )
            self._optional(
                params,
                "destroy_call",
                f"{where}.params",
                default_desc="auto destroy() if exists",
                typ=list,
            )

            # sub-mode polling settings
            self._optional(
                params, "method_name", f"{where}.params", default_desc="read", typ=str
            )
            self._optional(
                params, "method_kwargs", f"{where}.params", default_desc="{}", typ=dict
            )
            self._optional(
                params,
                "interval",
                f"{where}.params",
                default_desc="0.0",
                typ=(int, float),
            )

        # -------- protocol: port --------
        elif protocol == "port":
            self._require(params, "port_type", f"{where}.params", str)
            self._require(params, "init_args", f"{where}.params", dict)
            self._require(params, "out_key", f"{where}.params", str)

            # Optional: init_args fields depend on port_type, just warn common ones
            init_args = params["init_args"]
            self._optional(
                init_args,
                "device_id",
                f"{where}.params.init_args",
                default_desc="(port-specific)",
                typ=(str, int),
            )
            self._optional(
                init_args,
                "width",
                f"{where}.params.init_args",
                default_desc="(port-specific)",
                typ=(int,),
            )
            self._optional(
                init_args,
                "height",
                f"{where}.params.init_args",
                default_desc="(port-specific)",
                typ=(int,),
            )
            self._optional(
                init_args,
                "encoding",
                f"{where}.params.init_args",
                default_desc="bgr8/rgb8/jpeg/png",
                typ=str,
            )
            self._optional(
                init_args,
                "fps",
                f"{where}.params.init_args",
                default_desc="(port-specific)",
                typ=(int, float),
            )

        else:
            raise ValueError(
                f"[config error] unsupported input protocol '{protocol}' at {where}"
            )

    # -------------------------
    # validate outputs
    # -------------------------
    def _validate_output(self, out: Dict[str, Any], where: str) -> None:
        protocol = str(self._require(out, "protocol", where, str)).lower()
        adapter_name = str(self._require(out, "adapter", where, str))

        # adapter must exist
        if adapter_name not in ADAPTER_REGISTRY:
            raise ValueError(
                f"[config error] unknown output adapter '{adapter_name}' at {where}. "
                f"available={list(ADAPTER_REGISTRY.keys())}"
            )

        # -------- protocol: ros2/lcm --------
        if protocol in ("ros2", "lcm"):
            self._require(out, "topic", where, str)
            self._require(out, "type", where, str)
            self._validate_message_class(out["type"], f"{where}.type")

            mappings = self._require(out, "mappings", where, list)
            if not mappings:
                raise ValueError(f"[config error] empty mappings at {where}")

            # current output adapters mostly use mappings[0]; warn if not 1
            if len(mappings) != 1:
                logger.warning(
                    f"[config warn] {where}.mappings has len={len(mappings)}; "
                    f"most output adapters use mappings[0] only"
                )

            m0 = mappings[0]
            if not isinstance(m0, dict):
                raise ValueError(f"[config error] mappings[0] must be dict at {where}")
            if not isinstance(m0.get("key"), str) or not m0["key"]:
                raise ValueError(
                    f"[config error] output mappings[0].key required at {where}"
                )
            if not isinstance(m0.get("out_field"), str) or not m0["out_field"]:
                raise ValueError(
                    f"[config error] output mappings[0].out_field required at {where}"
                )

        # -------- protocol: module --------
        elif protocol == "module":
            params = self._require(out, "params", where, dict)

            # Required
            self._require(params, "module_name", f"{where}.params", str)
            self._require(params, "method_name", f"{where}.params", str)

            # Optional
            self._optional(
                params,
                "sys_path",
                f"{where}.params",
                default_desc="[]",
                typ=(list, str),
            )
            self._optional(
                params, "init_args", f"{where}.params", default_desc="{}", typ=dict
            )
            self._optional(
                params,
                "instance_lock",
                f"{where}.params",
                default_desc="false",
                typ=bool,
            )
            self._optional(
                params,
                "start_call",
                f"{where}.params",
                default_desc="auto start() if exists",
                typ=list,
            )
            self._optional(
                params,
                "destroy_call",
                f"{where}.params",
                default_desc="auto destroy() if exists",
                typ=list,
            )

            # ModuleStepOutputAdapter requires dynamic_arg list (non-empty in your implementation)
            dyn = self._optional(
                params, "dynamic_arg", f"{where}.params", default_desc="[]", typ=list
            )
            self._optional(
                params, "static_args", f"{where}.params", default_desc="[]", typ=list
            )
            self._optional(
                params,
                "post_chunk_delay_s",
                f"{where}.params",
                default_desc="0.0",
                typ=(int, float),
            )

            if adapter_name == "ModuleStepOutputAdapter" and not dyn:
                raise ValueError(
                    f"[config error] {adapter_name} requires params.dynamic_arg (non-empty list) at {where}"
                )

        elif protocol == "port":
            # PortAdapter.pub is not supported
            raise ValueError(
                f"[config error] outputs does not support protocol 'port' at {where} (PortAdapter.pub not supported)"
            )
        else:
            raise ValueError(
                f"[config error] unsupported output protocol '{protocol}' at {where}"
            )

    # -------------------------
    # server-level validation
    # -------------------------
    def _validate_server(self, srv: Dict[str, Any], where: str) -> None:
        self._require(srv, "name", where, str)

        inputs = srv.get("inputs", []) or []
        outputs = srv.get("outputs", []) or []
        compose = srv.get("compose", []) or []

        if not isinstance(inputs, list):
            raise ValueError(f"[config error] {where}.inputs must be list")
        if not isinstance(outputs, list):
            raise ValueError(f"[config error] {where}.outputs must be list")
        if not isinstance(compose, list):
            raise ValueError(f"[config error] {where}.compose must be list")

        if srv.get("inputs") is None:
            logger.warning(
                f"[config warn] optional '{where}.inputs' not set, default=[]"
            )
        if srv.get("outputs") is None:
            logger.warning(
                f"[config warn] optional '{where}.outputs' not set, default=[]"
            )
        if srv.get("compose") is None:
            logger.warning(
                f"[config warn] optional '{where}.compose' not set, default=[]"
            )

        for i, inp in enumerate(inputs):
            if not isinstance(inp, dict):
                raise ValueError(f"[config error] {where}.inputs[{i}] must be dict")
            self._validate_input(inp, f"{where}.inputs[{i}]")

        for i, out in enumerate(outputs):
            if not isinstance(out, dict):
                raise ValueError(f"[config error] {where}.outputs[{i}] must be dict")
            self._validate_output(out, f"{where}.outputs[{i}]")

    # -------------------------
    # top-level validation
    # -------------------------
    def validate_robot_config(self, config: Dict[str, Any]) -> None:
        # required
        self._require(config, "robot_type", "root", str)
        servers = self._require(config, "servers", "root", list)
        if not servers:
            raise ValueError("[config error] root.servers is empty")

        # optional
        if config.get("config_type") is None:
            logger.warning(
                "[config warn] optional 'config_type' not set, default='robot_config'"
            )

        for i, srv in enumerate(servers):
            if not isinstance(srv, dict):
                raise ValueError(f"[config error] servers[{i}] must be dict")
            self._validate_server(srv, f"servers[{i}]")

        logger.info(f"[ConfigValidator] robot_config OK, servers={len(servers)}")

    def validate_rcp_config(self, config: Dict[str, Any]) -> None:
        raise NotImplementedError("[ConfigValidator] rcp_config is not implemented yet")

    def validate_config(self, config: Dict[str, Any]) -> None:
        config_type = config.get("config_type", "robot_config")
        if config_type == "robot_config":
            self.validate_robot_config(config)
        elif config_type == "rcp_config":
            self.validate_rcp_config(config)
        else:
            raise ValueError(f"[config error] unknown config_type '{config_type}'")
